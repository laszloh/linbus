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

#ifndef IO_PINS_H
#define IO_PINS_H

#include "avr_util.h"

namespace io_pins {
  // A class to abstract an output pin that is not necesarily an arduino 
  // digital pin. Also optimized for fast setOn/Off.
  //
  // Assumes that interrupts are enabled and thus should not be called
  // from ISRs.

  template<uint8_t port_addr, int pin_nr, bool pullUp>
  class InputPin {
    protected:
      struct PORT_t {
        uint8_t port;
        uint8_t ddr;
        uint8_t pin;
      };

      PORT_t *p = reinterpret_cast<PORT_t *>(port_addr);

    public:
      void setup() {
        p->ddr &= ~_BV(pin_nr);
        if(pullUp) {
          p->port |= _BV(pin_nr);
        } else {
          p->port &= ~_BV(pin_nr);
        }
      }

      bool isHigh() {
        return p->pin & _BV(pin_nr);
      }
  };

  template<uint8_t port_addr, uint8_t pin_nr, bool inital>
  class OutputPin {
    protected:
      struct PORT_t {
        uint8_t port;
        uint8_t ddr;
        uint8_t pin;
      };

      PORT_t *p = reinterpret_cast<PORT_t *>(port_addr);

    public:
      void setup(){
        p->ddr |= _BV(pin_nr);
        set(inital);
      }

      void low() {
        avr::interrupt::atomic<> lock;
        p->port &=~_BV(pin_nr);
      }

      void high() {
        avr::interrupt::atomic<> lock;
        p->port |= _BV(pin_nr);
      }

      void set(bool level) {
        if(level)
          high();
        else
          low();
      }

      void toggle(){
        p->pin |= _BV(pin_nr);
      }
  };

  static const int PORTB_ADDR = (int) &PINB;
  static const int PORTC_ADDR = (int) &PINC;
  static const int PORTD_ADDR = (int) &PIND;
  
}  // namespace io_pins

#endif


