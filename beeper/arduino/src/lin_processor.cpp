// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// TODO: file is too long. Refactor.

#include "lin_processor.h"

#include "avr_util.h"
#include "custom_defs.h"

// ----- Baud rate related parameters. ---

// If an out of range speed is specified, using this one.
static const uint16_t kDefaultBaud = 9600;

// Wait at most N bits from the end of the stop bit of previous byte
// to the start bit of next byte.
//
// TODO: seperate values for pre response space (longer, e.g. 8) and pre
// regular byte space (shorter, e.g. 4).
//
static const uint8_t kMaxSpaceBits = 6;

namespace lin_processor {

  class Config : public Printable {
   public:
    // Initialized to given baud rate. 
    void setup() {
      static_assert(baud_ > 1000 && baud_ < 20000, "ERROR: kLinSpeed out of range");
    }

    inline uint16_t baud() const { 
      return baud_; 
    }

    inline uint8_t prescaling() const {
      return prescaling_;
    }

    inline uint16_t counts_per_bit() const { 
      return counts_per_bit_; 
    }
    inline uint16_t counts_per_half_bit() const { 
      return counts_per_half_bit_; 
    }
    inline uint8_t millis_per_bit() const { 
      return millis_per_bit_; 
    }
    inline uint8_t millis_per_half_bit() const { 
      return millis_per_half_bit_; 
    }
    inline uint8_t millis_until_start_bit() const { 
      return millis_until_start_bit_; 
    }

    virtual size_t printTo(Print& p) const final {
      char buffer [128];
      snprintf_P(buffer, sizeof(buffer), PSTR("LIN: %u, %u, %u, %u, %u, %u, %u"), baud_,
                  custom_defs::kUseLinChecksumVersion2, prescaler_x64_, counts_per_bit_,
                  counts_per_half_bit_, millis_per_bit_, millis_per_half_bit_,
                  millis_until_start_bit_);

      return p.println(buffer);
    }
   private:
    static constexpr uint16_t baud_ = custom_defs::kLinSpeed;
    // False -> x8, true -> x64.
    // TODO: timer2 also have x32 scalingl Could use it for better 
    // accuracy in the mid baude range.
    static constexpr bool prescaler_x64_= (baud_ < 8000);
    static constexpr uint8_t prescaling_ = prescaler_x64_ ? 64 : 1;
    static constexpr uint16_t counts_per_bit_ = (((F_CPU / prescaling_) / baud_)) - 1;
    static constexpr uint16_t counts_per_half_bit_ = (counts_per_bit_ / 2) + 2;
    static constexpr uint8_t millis_per_bit_ = baud_ / 1000;
    static constexpr uint8_t millis_per_half_bit_ = millis_per_bit_ / 2;
    static constexpr uint8_t millis_until_start_bit_ = millis_per_bit_ * kMaxSpaceBits;
  };

  // The actual configurtion. Initialized in setup() based on baud rate.  
  Config config;

  // ----- Digital I/O pins
  //
  // NOTE: we use direct register access instead the abstractions in io_pins.h. 
  // This way we shave a few cycles from the ISR.
    
  // LIN interface.
  io_pins::InputPin<io_pins::PORTD_ADDR, 2, true> rx_pin;
  // TODO: tie this pin to the TX pin of the ata6631 ic, for future applications.
  io_pins::OutputPin<io_pins::PORTC_ADDR, 2, true> tx1_pin;
  
  // Debugging signals.
  io_pins::OutputPin<io_pins::PORTC_ADDR, 0, false> break_pin;
  io_pins::OutputPin<io_pins::PORTB_ADDR, 4, false> sample_pin;
  io_pins::OutputPin<io_pins::PORTB_ADDR, 3, false> error_pin;
  io_pins::OutputPin<io_pins::PORTC_ADDR, 3, false> isr_pin;
  io_pins::OutputPin<io_pins::PORTD_ADDR, 6, false> gp_pin;

  // Called one during initialization.
  static inline void setupPins() {
    rx_pin.setup();
    break_pin.setup();
    sample_pin.setup();
    error_pin.setup();
    isr_pin.setup();
    gp_pin.setup();
  }

  // ----- ISR RX Ring Buffers -----

  // Frame buffer queue size.
  static constexpr uint8_t kMaxFrameBuffers = 8;

  // RX Frame buffers queue. Read/Writen by ISR only. 
  static LinFrame rx_frame_buffers[kMaxFrameBuffers];

  // Index [0, kMaxFrameBuffers) of the current frame buffer being
  // written (newest). Read/Written by ISR only.
  static uint8_t  head_frame_buffer;

  // Index [0, kMaxFrameBuffers) of the next frame to be read (oldest).
  // If equals head_frame_buffer then there is no available frame.
  // Read/Written by ISR only.
  static uint8_t tail_frame_buffer;

  // Called once from main.
  static inline void setupBuffers() {
    head_frame_buffer = 0;
    tail_frame_buffer = 0;
  }

  // Called from ISR or from main with interrupts disabled.
  static inline void incrementTailFrameBuffer() {
    if (++tail_frame_buffer >= kMaxFrameBuffers) {
      tail_frame_buffer = 0;
    }
  }

  // Called from ISR. If stepping on tail buffer, caller needs to 
  // increment raise frame overrun error.
  static inline void incrementHeadFrameBuffer() {
    if (++head_frame_buffer >= kMaxFrameBuffers) {
      head_frame_buffer = 0;
    }
  }

  // ----- ISR To Main Data Transfer -----
  
  // Public. Called from main. See .h for description.
  boolean readNextFrame(LinFrame* buffer) {
    boolean result = false;
    avr::interrupt::atomic<> lock;
    if (tail_frame_buffer != head_frame_buffer) {
      //led::setHigh();
      // This copies the request buffer struct.
      *buffer = rx_frame_buffers[tail_frame_buffer];
      incrementTailFrameBuffer();
      result = true;
      //led::setLow();
    }
    return result; 
  }

  // ----- State Machine Declaration -----

  // Like enum but 8 bits only.
  enum class StateMachine : uint8_t {
    DETECT_BREAK = 1,
    CLOCK_SYNC,
    READ_DATA,
  };
  static StateMachine state;

  class StateDetectBreak {
   public:
    static inline void enter() ;
    static inline void handleIsr();
    
   private:
    static uint8_t low_bits_counter_;
    static bool break_detected;
    static uint8_t wait_high_counter_;
  };

  class StateClockSync {
    public:
      static inline void enter();
      static inline void handleIsr();

    private:
      static uint8_t bits_read_in_byte_;
      static uint16_t byte_buffer_;
  };

  class StateReadData {
   public:
    // Should be called after the break stop bit was detected.
    static inline void enter();
    static inline void handleIsr();

    static inline void finish();
    
   private:
    // Number of complete bytes read so far. Includes all bytes, even
    // sync, id and checksum.
    static uint8_t bytes_read_;
    
    // Number of bits read so far in the current byte. Includes start bit, 
    // 8 data bits and one stop bits.
    static uint8_t bits_read_in_byte_;

    // Buffer for the current byte we collect (including start and stop bits).
    static uint16_t byte_buffer_;
   
    // When collecting the data bits, this goes (1 << 0) to (1 << 7). Could
    // be computed as (1 << (bits_read_in_byte_ - 1)). We use this cached value
    // recude ISR computation.
    static uint8_t byte_buffer_bit_mask_;
  };

  // ----- Error Flag. -----

  // Written from ISR. Read/Write from main.
  static volatile boolean error_flags;

  // Private. Called from ISR and from setup (beofe starting the ISR).
  static inline void setErrorFlags(Errors flags) {
    error_pin.high();
    // Non atomic when called from setup() but should be fine since ISR is not running yet.
    error_flags |= flags;
    error_pin.low();
  }

  // Called from main. Public. Assumed interrupts are enabled. 
  // Do not call from ISR.
  boolean getAndClearErrorFlags() {
    // Disabling interrupts for a brief for atomicity. Need to pay attention to
    // ISR jitter due to disabled interrupts.
    avr::interrupt::atomic<> lock;
    const boolean result = error_flags;
    error_flags = 0;
    return result;
  }

  struct BitName {
    const Errors mask;
    const char* const name;  
  };

  static const BitName kErrorBitNames[] PROGMEM = {
    { Errors::FRAME_TOO_SHORT, "SHRT" },
    { Errors::FRAME_TOO_LONG, "LONG" },
    { Errors::START_BIT, "STRT" },
    { Errors::STOP_BIT, "STOP" },
    { Errors::SYNC_BYTE, "SYNC" },
    { Errors::BUFFER_OVERRUN, "OVRN" },
    { Errors::BREAK_TOO_LONG, "BRKL" },
    { Errors::OTHER, "OTHR" },
  };

  // Given a byte with lin processor error bitset, print the list
  // of set errors.
  void printErrorFlags(uint8_t lin_errors) {
    const uint8_t n = ARRAY_SIZE(kErrorBitNames); 
    boolean any_printed = false;
    for (uint8_t i = 0; i < n; i++) {
      const uint8_t mask = pgm_read_byte(&kErrorBitNames[i].mask);
      if (lin_errors & mask) {
        if (any_printed) {
          Serial.print(' ');
        }
        const char* const name = (const char*)pgm_read_word(&kErrorBitNames[i].name);
        Serial.print(name);
        any_printed = true;
      }
    }
  }

  // ----- Initialization -----

  // setup Timer1 for SW UART
  static void setupTimer() {
    // Mode #4 for Timer1 
    // prescaler 1:1
    // IC Noise Cancel 
    // IC on Falling Edge 
    TCCR1A = 0;
    TCCR1B = _BV(WGM12) | _BV(CS10) | _BV(ICES1) | _BV(ICNC1);

    // Determines baud rate (transmit)
    OCR1A = config.counts_per_bit() - 1;

    // setup rx part
    TIMSK1 |= _BV(ICIE1);
    TIFR1 |= _BV(ICF1) | _BV(OCF1B);
  }

  // Call once from main at the begining of the program.
  void setup() {
    // Should be done first since some of the steps below depends on it.
    config.setup();

    setupPins();
    setupBuffers();
    StateDetectBreak::enter();
    setupTimer();
    error_flags = 0;

    // TODO: move this to config class.
    Serial.println(config);
  }

  // ----- ISR Utility Functions -----

  // Set timer value to zero.
  static inline void setIC1Interrupt() {
    TIFR1 = _BV(ICF1);
    TIMSK1 = _BV(ICIE1) | (TIMSK1 & ~_BV(OCIE1B)) | _BV(TOIE1);
  }

  static inline void setOC1BInterrupt() {
    TIFR1 = _BV(OCF1B);
    TIMSK1 = (TIMSK1 & ~_BV(ICIE1)) | _BV(OCIE1B) | _BV(TOIE1);
  }

  // ----- Detect-Break State Implementation -----

  uint8_t StateDetectBreak::low_bits_counter_;
  bool StateDetectBreak::break_detected;
  uint8_t StateDetectBreak::wait_high_counter_;

  inline void StateDetectBreak::enter() {
    state = StateMachine::DETECT_BREAK;
    setIC1Interrupt();
    low_bits_counter_ = 0;
    break_detected = false;
    wait_high_counter_ = 0;
  }

  // Return true if enough time to service rx request.
  inline void StateDetectBreak::handleIsr() {

    if(break_detected) {
      if(rx_pin.isHigh()) {
        // we found the "stop" bit of the break signal
        break_pin.low();
        StateClockSync::enter();
      }

      if(++wait_high_counter_ < 100) {
        return;
      }

      // we propably have a short circuit to ground
      break_pin.low();
      setErrorFlags(Errors::BREAK_TOO_LONG);
      StateDetectBreak::enter();
    } else {
      // we are still counting zero's
      if(rx_pin.isHigh()) {
        // not good, reset counter
        low_bits_counter_ = 0;
        wait_high_counter_ = 0;
        return;
      }

      // here RX is low, count the zero's
      if(++low_bits_counter_ < 10)
        return;

      break_pin.high();
      break_detected = true;
    }
  }

  // ----- Clock Sync State Implementation -----

  uint8_t StateClockSync::bits_read_in_byte_;
  uint16_t StateClockSync::byte_buffer_;

  void StateClockSync::enter() {
    state = StateMachine::CLOCK_SYNC;
    bits_read_in_byte_ = 10;
    byte_buffer_ = 0;
  }

  void StateClockSync::handleIsr() {

    sample_pin.high();
    const bool rxPin = rx_pin.isHigh();
    sample_pin.low();

    uint16_t localInFrame = byte_buffer_ >> 1;
    if(rxPin)
      localInFrame |= _BV(9);

    if(!--bits_read_in_byte_) {
      // test for start and stopbit
      bool err = false;
      if( !(localInFrame & _BV(0)) && (localInFrame & _BV(9))) {
        // correct data received
        uint8_t data = localInFrame >> 1;

        if(data != 0x55) {
          // we did not get the sync byte
          setErrorFlags(Errors::SYNC_BYTE);
          err = true;
        }
      } else {
        err = true;
        if(localInFrame & _BV(0))   // start bit was 1
          setErrorFlags(Errors::START_BIT);
        if(!(localInFrame & _BV(9)))  // stop bit was 0
          setErrorFlags(Errors::STOP_BIT);
      }

      if(err) {
        StateDetectBreak::enter();
      } else {
        StateReadData::enter();
      }
    } else {
      byte_buffer_ = localInFrame;
    }
  }

  // ----- Read-Data State Implementation -----

  uint8_t StateReadData::bytes_read_;
  uint8_t StateReadData::bits_read_in_byte_;
  uint16_t StateReadData::byte_buffer_;
  uint8_t StateReadData::byte_buffer_bit_mask_;

  // Called on the low to high transition at the end of the break.
  inline void StateReadData::enter() {
    state = StateMachine::READ_DATA;
    bytes_read_ = 0;
    bits_read_in_byte_ = 10;
    rx_frame_buffers[head_frame_buffer].reset();
    setIC1Interrupt();
  }

  inline void StateReadData::finish() {
    // we are called from the timer overflow, if we did not get a 
    // falling edge for at least 16 bits (10 bit data + 6 bit frame interface)

    // Verify min byte count.
    if (bytes_read_ < LinFrame::kMinBytes) {
      setErrorFlags(Errors::FRAME_TOO_SHORT);
      StateDetectBreak::enter();
      return;
    }

    // Frame looks ok so far. Move to next frame in the ring buffer.
    // NOTE: we will reset the byte_count of the new frame buffer next time we will enter data detect state.
    // NOTE: verification of sync byte, id, checksum, etc is done latter by the main code, not the ISR.
    incrementHeadFrameBuffer();
    if (tail_frame_buffer == head_frame_buffer) {
      // Frame buffer overrun. We drop the oldest frame and continue with this one.       
      setErrorFlags(Errors::BUFFER_OVERRUN);
      incrementTailFrameBuffer();
    }

    StateDetectBreak::enter();
  }

  inline void StateReadData::handleIsr() {
    // Sample data bit ASAP to avoid jitter.
    sample_pin.high();
    const bool is_rx_high = rx_pin.isHigh();
    sample_pin.low();

    uint16_t localInFrame = byte_buffer_ >> 1;
    if(is_rx_high)
      localInFrame |= _BV(9);

    if(!--bits_read_in_byte_) {
      // we got a whole byte, test start and stop bit
      bool err = false;
      if( !(localInFrame & _BV(0)) && (localInFrame & _BV(9))) {
        bytes_read_++;
        bits_read_in_byte_ = 10;
        byte_buffer_ = 0x00;

        if(bytes_read_ > LinFrame::kMaxBytes) {
          // we read more bytes than we can store!
          setErrorFlags(Errors::FRAME_TOO_LONG);
          err = true;
        } else {
          // everything is ok
          rx_frame_buffers[head_frame_buffer].append_byte(localInFrame >> 1);
        }

        // frame end is handled elsewhere, since we are stateless here

      } else {
        err = true;
        if(localInFrame & _BV(0))   // start bit was 1
          setErrorFlags(Errors::START_BIT);
        if(!(localInFrame & _BV(9)))  // stop bit was 0
          setErrorFlags(Errors::STOP_BIT);
      }

      if(err) {
        StateDetectBreak::enter();
      }

    } else {
      byte_buffer_ = localInFrame;
    }
  }

  // ----- ISR Handler -----

  static volatile uint32_t icr_millis;

  ISR(TIMER1_CAPT_vect) {
    // Add half bit duration to value from start bit (we lost four bits for noise cancelling)
    uint16_t ocr1b = ICR1 + config.counts_per_half_bit() - 4;
    if(ocr1b > config.counts_per_bit())
      ocr1b -= config.counts_per_bit();
    OCR1B = ocr1b;

    icr_millis = millis();

    // Disable InputCapture interrupt and enable CompareB interrupt to collect bits
    setOC1BInterrupt();
  }

  // Interrupt on Timer 1 A-match.
  ISR(TIMER1_COMPB_vect)
  {
    isr_pin.high();
    switch (state) {
    case StateMachine::DETECT_BREAK:
      StateDetectBreak::handleIsr();
      break;
    case StateMachine::CLOCK_SYNC:
      StateClockSync::handleIsr();
      break;
    case StateMachine::READ_DATA:
      StateReadData::handleIsr();
      break;
    default:
      setErrorFlags(Errors::OTHER);
      StateDetectBreak::enter();
    }

    isr_pin.low();
  }

  ISR(TIMER1_OVF_vect) {
    if(state == StateMachine::READ_DATA) {
      // we are currently reading bytes
      
      // detect end of frame by checking the current time against the last time we got a falling edge interrupt
      const auto offset = millis() - icr_millis;

      if(offset > (uint32_t)(config.millis_until_start_bit() + 10 * config.millis_per_bit()) ) {
        // last Input Capture interrupt was at least a frame + 6 frameBits ago
        StateReadData::finish(); 
      }
    }
  }
}  // namespace lin_processor






