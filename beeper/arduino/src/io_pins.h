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
class OutputPin {
public:
    // port is either PORTB, PORTC, or PORTD.
    // bit index is one of [7, 6, 5, 4, 3, 2, 1, 0] (lsb).
    // NOTE: ddr port is is always one address below portx.
    // NOTE: pin port is is always two addresses below portx.
    OutputPin(volatile uint8_t& port, uint8_t bitIndex) : port_(port), pin_(*((&port) - 2)), bit_mask_(1 << bitIndex) {
        // NOTE: ddr port is is always one address below port.
        volatile uint8_t& ddr = *((&port) - 1);
        ddr |= bit_mask_;
        low(); // default state.
    }

    inline void high() {
        // Wrapping in cli/sei in case any pin of this port is changed
        // from an ISR. Keeping this as short as possible to avoid jitter
        // in the ISR invocation.
        cli();
        port_ |= bit_mask_;
        sei();
    }

    inline void low() {
        // Wrapping in cli/sei in case any pin of this port is changed
        // from an ISR. Keeping this as short as possible to avoid jitter
        // in the ISR invocation.
        cli();
        port_ &= ~bit_mask_;
        sei();
    }

    inline void set(bool v) {
        if(v) {
            high();
        } else {
            low();
        }
    }

    inline void toggle() { set(!isHigh()); }

    inline bool isHigh() { return pin_ & bit_mask_; }

private:
    volatile uint8_t& port_;
    volatile uint8_t& pin_;
    const uint8_t bit_mask_;
};

// A class to abstract an input pin that is not necesarily an arduino
// digital pin. Also optimized for quick access.
class InputPin {
public:
    // See OutputPin() for description of the args.
    InputPin(volatile uint8_t& port, uint8_t bitIndex) : pin_(*((&port) - 2)), bit_mask_(1 << bitIndex) {
        volatile uint8_t& ddr = *((&port) - 1);
        ddr &= ~bit_mask_; // input
        port |= bit_mask_; // pullup
    }

    inline bool isHigh() { return pin_ & bit_mask_; }

private:
    volatile uint8_t& pin_;
    const uint8_t bit_mask_;
};

} // namespace io_pins

#endif
