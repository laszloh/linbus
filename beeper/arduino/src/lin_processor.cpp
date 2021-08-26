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

    inline boolean prescaler_x64() const {
      return prescaler_x64_;
    }

    inline uint8_t counts_per_bit() const { 
      return counts_per_bit_; 
    }
    inline uint8_t counts_per_half_bit() const { 
      return counts_per_half_bit_; 
    }
    inline uint8_t clock_ticks_per_bit() const { 
      return clock_ticks_per_bit_; 
    }
    inline uint8_t clock_ticks_per_half_bit() const { 
      return clock_ticks_per_half_bit_; 
    }
    inline uint8_t clock_ticks_per_until_start_bit() const { 
      return clock_ticks_per_until_start_bit_; 
    }

    virtual size_t printTo(Print& p) const final {
      char buffer [128];
      snprintf_P(buffer, sizeof(buffer), PSTR("LIN: %u, %u, %u, %u, %u, %u, %u, %u, "), baud_,
                  custom_defs::kUseLinChecksumVersion2, prescaler_x64_, counts_per_bit_,
                  counts_per_half_bit_, clock_ticks_per_bit_, clock_ticks_per_bit_,
                  clock_ticks_per_half_bit_, clock_ticks_per_until_start_bit_);

      return p.println(buffer);
    }
   private:
    static constexpr uint16_t baud_ = custom_defs::kLinSpeed;
    // False -> x8, true -> x64.
    // TODO: timer2 also have x32 scalingl Could use it for better 
    // accuracy in the mid baude range.
    static constexpr boolean prescaler_x64_= (baud_ < 8000);
    static constexpr uint8_t prescaling_ = prescaler_x64_ ? 64 : 8;
    static constexpr uint8_t counts_per_bit_ = (((F_CPU / prescaling_) / baud_));
    static constexpr uint8_t counts_per_half_bit_ = (counts_per_bit_ / 2) + 2;
    static constexpr uint8_t clock_ticks_per_bit_ = 250000 / baud_;
    static constexpr uint8_t clock_ticks_per_half_bit_ = clock_ticks_per_bit_ / 2;
    static constexpr uint8_t clock_ticks_per_until_start_bit_ = clock_ticks_per_bit_ * kMaxSpaceBits;
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
  };

  class StateClockSync {
    public:
      static inline void enter();
      static inline void handleIsr();

    private:
      uint16_t counter_;
  };

  class StateReadData {
   public:
    // Should be called after the break stop bit was detected.
    static inline void enter();
    static inline void handleIsr();
    
   private:
    // Number of complete bytes read so far. Includes all bytes, even
    // sync, id and checksum.
    static uint8_t bytes_read_;
    
    // Number of bits read so far in the current byte. Includes start bit, 
    // 8 data bits and one stop bits.
    static uint8_t bits_read_in_byte_;

    // Buffer for the current byte we collect.
    static uint8_t byte_buffer_;
   
    // When collecting the data bits, this goes (1 << 0) to (1 << 7). Could
    // be computed as (1 << (bits_read_in_byte_ - 1)). We use this cached value
    // recude ISR computation.
    static uint8_t byte_buffer_bit_mask_;
  };

  // ----- Error Flag. -----

  // Written from ISR. Read/Write from main.
  static volatile boolean error_flags;

  // Private. Called from ISR and from setup (beofe starting the ISR).
  static inline void setErrorFlags(uint8_t flags) {
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
    const uint8_t mask;
    const char* const name;  
  };

  static const BitName kErrorBitNames[] PROGMEM = {
    { errors::FRAME_TOO_SHORT, "SHRT" },
    { errors::FRAME_TOO_LONG, "LONG" },
    { errors::START_BIT, "STRT" },
    { errors::STOP_BIT, "STOP" },
    { errors::SYNC_BYTE, "SYNC" },
    { errors::BUFFER_OVERRUN, "OVRN" },
    { errors::OTHER, "OTHR" },
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

  // setup Timer2 for SW UART
  static void setupTimer() {
    // Mode #4 for Timer1 
    // prescaler 1:1
    // IC Noise Cancel 
    // IC on Falling Edge 
    TCCR1A = 0;
    TCCR1B = _BV(WGM12) | _BV(CS10) | _BV(ICES1) | _BV(ICNC1);

    // Determines baud rate
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
  static inline void resetTickTimer() {
    // TODO: also clear timer2 prescaler.
    TCNT1 = 0;
  }

  // Set timer value to half a tick. Called at the begining of the
  // start bit to generate sampling ticks at the middle of the next
  // 10 bits (start, 8 * data, stop).
  static inline void setTimerToHalfTick() {
    // Adding 2 to compensate for pre calling delay. The goal is
    // to have the next ISR data sampling at the middle of the start
    // bit.
    TCNT1 = config.counts_per_half_bit();
  }

  // Perform a tight busy loop until RX is low or the given number
  // of clock ticks passed (timeout). Retuns true if ok,
  // false if timeout. Keeps timer reset during the wait.
  // Called from ISR only.
  static inline boolean waitForRxLow(uint16_t max_clock_ticks) {
    // const uint16_t base_clock = hardware_clock::ticksForIsr();
    for(;;) {
      // Keep the tick timer not ticking (no ISR).
      resetTickTimer();

      // If rx is low we are done.
      if (!rx_pin.isHigh()) {
        return true;
      }

      // // Test for timeout.
      // // Should work also in case of 16 bit clock overflow.
      // const uint16_t clock_diff = hardware_clock::ticksForIsr() - base_clock;
      // if (clock_diff >= max_clock_ticks) {
      //   return false; 
      // }
    } 
  }

  // Same as waitForRxLow but with reversed polarity.
  // We clone to code for time optimization.
  // Called from ISR only.
  static inline boolean waitForRxHigh(uint16_t max_clock_ticks) {
    // const uint16_t base_clock = hardware_clock::ticksForIsr();
    for(;;) {
      resetTickTimer();
      if (rx_pin.isHigh()) {
        return true;
      }
      // // Should work also in case of an clock overflow.
      // const uint16_t clock_diff = hardware_clock::ticksForIsr() - base_clock;
      // if (clock_diff >= max_clock_ticks) {
      //   return false; 
      // }
    } 
  }

  // ----- Detect-Break State Implementation -----

  uint8_t StateDetectBreak::low_bits_counter_;

  inline void StateDetectBreak::enter() {
    state = StateMachine::DETECT_BREAK;
    low_bits_counter_ = 0;
  }

  // Return true if enough time to service rx request.
  inline void StateDetectBreak::handleIsr() {
    if (rx_pin.isHigh()) {
      low_bits_counter_ = 0;
      return;
    } 

    // Here RX is low (active)  

    if (++low_bits_counter_ < 10) {
      return;
    }

    // Detected a break. Wait for rx high and enter data reading.
    break_pin.high();

    // TODO: set actual max count
    waitForRxHigh(255);
    break_pin.low();
   
    // Go process the data
    StateReadData::enter();
  }

  // ----- Read-Data State Implementation -----

  uint8_t StateReadData::bytes_read_;
  uint8_t StateReadData::bits_read_in_byte_;
  uint8_t StateReadData::byte_buffer_;
  uint8_t StateReadData::byte_buffer_bit_mask_;

  // Called on the low to high transition at the end of the break.
  inline void StateReadData::enter() {
    state = StateMachine::READ_DATA;
    bytes_read_ = 0;
    bits_read_in_byte_ = 0;
    rx_frame_buffers[head_frame_buffer].reset();

    // TODO: handle post break timeout errors.
    // TODO: set a reasonable time limit.
    waitForRxLow(255);
    setTimerToHalfTick();   
  }

  inline void StateReadData::handleIsr() {
    // Sample data bit ASAP to avoid jitter.
    sample_pin.high();
    const uint8_t is_rx_high = rx_pin.isHigh();
    sample_pin.low();
    
    // Handle start bit.
    if (bits_read_in_byte_ == 0) {
      // Start bit error.
      if (is_rx_high) {
        // If in sync byte, report as a sync error.
        setErrorFlags(bytes_read_ == 0 ? errors::SYNC_BYTE : errors::START_BIT);
        StateDetectBreak::enter();
        return;
      }  
      // Start bit ok.
      bits_read_in_byte_++;
      // Prepare buffer and mask for data bit collection.
      byte_buffer_ = 0;
      byte_buffer_bit_mask_ = (1 << 0);
      return;
    }

    // Handle next data bit, 1 out of total of 8. 
    // Collect the current bit into byte_buffer_, lsb first.
    if (bits_read_in_byte_ <= 8) {
      if (is_rx_high) {
        byte_buffer_ |= byte_buffer_bit_mask_;
      }
      byte_buffer_bit_mask_ = byte_buffer_bit_mask_ << 1;
      bits_read_in_byte_++;
      return;
    }
    
    // Here when in a stop bit. 
    bytes_read_++;
    bits_read_in_byte_ = 0;

    // Error if stop bit is not high.
    if (!is_rx_high) {
      // If in sync byte, report as sync error.
      setErrorFlags(bytes_read_ == 0 ? errors::SYNC_BYTE : errors::STOP_BIT);
      StateDetectBreak::enter();
      return;
    }  
    
    // Here when we just finished reading a byte. 
    // bytes_read is already incremented for this byte.
    
    // If this is the sync byte, verify that it has the expected value.
    if (bytes_read_ == 1) {
      // Should be exactly 0x55. We don't append this byte to the buffer.
      if (byte_buffer_ != 0x55) {
        setErrorFlags(errors::SYNC_BYTE);
        StateDetectBreak::enter();
        return;
      }
    } else {
      // If this is the id, data or checksum bytes, append it to the frame buffer.
      // NOTE: the byte limit count is enforeced somewhere else so we can assume safely here that this 
      // will not cause a buffer overlow.
      rx_frame_buffers[head_frame_buffer].append_byte(byte_buffer_);
    }

    // Wait for the high to low transition of start bit of next byte.
    const boolean has_more_bytes =  waitForRxLow(config.clock_ticks_per_until_start_bit());

    // Handle the case of no more bytes in this frame.
    if (!has_more_bytes) {
      // Verify min byte count.
      if (bytes_read_ < LinFrame::kMinBytes) {
        setErrorFlags(errors::FRAME_TOO_SHORT);
        StateDetectBreak::enter();
        return;
      }

      // Frame looks ok so far. Move to next frame in the ring buffer.
      // NOTE: we will reset the byte_count of the new frame buffer next time we will enter data detect state.
      // NOTE: verification of sync byte, id, checksum, etc is done latter by the main code, not the ISR.
      incrementHeadFrameBuffer();
      if (tail_frame_buffer == head_frame_buffer) {
        // Frame buffer overrun. We drop the oldest frame and continue with this one.       
        setErrorFlags(errors::BUFFER_OVERRUN);
        incrementTailFrameBuffer();
      }

      StateDetectBreak::enter();
      return;
    }

    // Here when there is at least one more byte in this frame. Error if we already had
    // the max number of bytes.
    if (rx_frame_buffers[head_frame_buffer].num_bytes() >= LinFrame::kMaxBytes) {
      setErrorFlags(errors::FRAME_TOO_LONG);
      StateDetectBreak::enter();
      return;  
    }

    // Everything is ready for the next byte. Have a tick in the middle of its
    // start bit.
    //
    // TODO: move this to above the num_bytes check above for more accurate 
    // timing of mid bit tick?
    setTimerToHalfTick();
  }

  // ----- ISR Handler -----

  ISR(TIMER1_CAPT_vect) {
    uint16_t icr1  = ICR1;
    uint16_t ocr1a = OCR1A;
   
    // Eine halbe Bitzeit zu ICR1 addieren (modulo OCR1A) und nach OCR1B
    uint16_t ocr1b = icr1 + ocr1a/2;
    if (ocr1b >= ocr1a)
        ocr1b -= ocr1a;
    OCR1B = ocr1b;
   
    TIFR1 = (1 << OCF1B);
    TIMSK1 = (TIMSK1 & ~(1 << ICIE1)) | (1 << OCIE1B);
  }

  // Interrupt on Timer 1 A-match.
  ISR(TIMER1_COMPB_vect)
  {
    isr_pin.high();
    // TODO: make this state a boolean instead of enum? (efficency).
    switch (state) {
    case StateMachine::DETECT_BREAK:
      StateDetectBreak::handleIsr();
      break;
    case StateMachine::READ_DATA:
      StateReadData::handleIsr();
      break;
    default:
      setErrorFlags(errors::OTHER);
      StateDetectBreak::enter();
    }

    isr_pin.low();
  }
}  // namespace lin_processor






