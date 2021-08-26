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

#ifndef LIN_PROCESSOR_H
#define LIN_PROCESSOR_H

#include "avr_util.h"
#include "lin_frame.h"

// Uses
// * Timer2 - used to generate the bit ticks.
// * OC2B (PD3) - timer output ticks. For debugging. If needed, can be changed
//   to not using this pin.
// * PD2 - LIN RX input.
// * PC0, PC1, PC2, PC3 - debugging outputs. See .cpp file for details.
namespace lin_processor {
  // Call once in program setup.
  extern void setup();

  // Try to read next available rx frame. If available, return true and set
  // given buffer. Otherwise, return false and leave *buffer unmodified.
  // The sync, id and checksum bytes of the frame as well as the total byte
  // count are not verified.
  extern boolean readNextFrame(LinFrame* buffer);

  // Errors byte masks for the individual error bits.
  enum class Errors : uint8_t {
    NONE = 0,
    FRAME_TOO_SHORT = _BV(0),
    FRAME_TOO_LONG = _BV(1),
    START_BIT = _BV(2),
    STOP_BIT = _BV(3),
    SYNC_BYTE = _BV(4),
    BUFFER_OVERRUN = _BV(5),
    BREAK_TOO_LONG = _BV(6),
    OTHER = _BV(7),
  };

  inline bool operator|=(bool, Errors& e) { return (e == Errors::NONE); }

  // Get current error flag and clear it.
  extern boolean getAndClearErrorFlags();

  // Print to sio a list of error flags.
  extern void printErrorFlags(uint8_t lin_errors);
}

#endif
