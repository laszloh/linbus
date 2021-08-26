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
#include <Arduino.h>

#include "action_led.h"
#include "avr_util.h"
#include "custom_module.h"
#include "io_pins.h"
#include "lin_processor.h"

// ERRORS LED - blinks when detecting errors.
static ActionLed<io_pins::PORTB_ADDR, 1> errors_activity_led;

// Arduino setup function. Called once during initialization.
void setup()
{
  // Hard coded to 115.2k baud. Uses URART0, no interrupts.
  // Initialize this first since some setup methods uses it.
  Serial.begin(115200);

  // Uses Timer2 with interrupts, and a few i/o pins. See source code for details.
  lin_processor::setup();
  
  custom_module::setup();

  // Enable global interrupts. We expect to have only timer1 interrupts by
  // the lin decoder to reduce ISR jitter.
  avr::interrupt::sei();
}

// Arduino loop() method. Called after setup(). Never returns.
// This is a quick loop that does not use delay() or other busy loops or 
// blocking calls.
void loop()
{
  // Having our own loop shaves about 4 usec per iteration. It also eliminate
  // any underlying functionality that we may not want.
  for(;;) {    
    // Periodic updates.   
    errors_activity_led.loop();  
    custom_module::loop();

    // Print a periodic text messages if no activiy.
    static PassiveTimer idle_timer;
    if (idle_timer.timeMillis() >= 5000) {
      Serial.println(F("waiting..."));
      idle_timer.restart();
    }

    // Handle LIN decoder error flags.
    {
      // Used to trigger periodic error printing.
      static PassiveTimer lin_errors_timeout;
      // Accomulates error flags until next printing.
      static uint8_t pending_lin_errors = 0;
      
      const uint8_t new_lin_errors = lin_processor::getAndClearErrorFlags();
      if (new_lin_errors) {
        // Make the ERRORS led blinking.
        errors_activity_led.action();
        idle_timer.restart();
      }

      // If pending errors and time to print then print and clear.
      pending_lin_errors |= new_lin_errors;
      if (pending_lin_errors && lin_errors_timeout.timeMillis() > 1000) {
        Serial.println(F("LIN errors: "));
        lin_processor::printErrorFlags(pending_lin_errors);
        Serial.println();
        lin_errors_timeout.restart();
        pending_lin_errors = 0;
      }
    }

    // Handle recieved LIN frames.
    LinFrame frame;
    if (lin_processor::readNextFrame(&frame)) {
      const boolean frameOk = frame.isValid();
      
      if (!frameOk) {
        // Make the ERRORS frame blinking.
        errors_activity_led.action();
      }
      
      // Print frame to serial port.
      for (int i = 0; i < frame.num_bytes(); i++) {
        if (i > 0) {
          Serial.print(' ');  
        }
        Serial.print(frame.get_byte(i), 16);
      }
      if (!frameOk) {
        Serial.print(F(" ERR"));
      }
      Serial.println();  
      // Supress the 'waiting' messages.
      idle_timer.restart(); 
    
      if (frameOk) {
        // Inform the custom logic about the incoming frame.
        custom_module::frameArrived(frame);
      }
    }
  }
}

