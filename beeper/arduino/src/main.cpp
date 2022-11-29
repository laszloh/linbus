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

#include "action_led.h"
#include "avr_util.h"
#include "custom_module.h"
#include "hardware_clock.h"
#include "io_pins.h"
#include "lin_processor.h"
#include "sio.h"
#include "system_clock.h"

// ERRORS LED - blinks when detecting errors.
static ActionLed errors_activity_led(PORTB, 1);

// Arduino setup function. Called once during initialization.
void setup() {
    // Hard coded to 115.2k baud. Uses URART0, no interrupts.
    // Initialize this first since some setup methods uses it.
    sio::setup();

    // Uses Timer1, no interrupts.
    hardware_clock::setup();

    // Uses Timer2 with interrupts, and a few i/o pins. See source code for details.
    lin_processor::setup();

    custom_module::setup();

    // Enable global interrupts. We expect to have only timer1 interrupts by
    // the lin decoder to reduce ISR jitter.
    sei();
}

// Arduino loop() method. Called after setup(). Never returns.
// This is a quick loop that does not use delay() or other busy loops or
// blocking calls.
void loop() {
    // Periodic updates.
    system_clock::loop();
    sio::loop();
    errors_activity_led.loop();
    custom_module::loop();

    // Print a periodic text messages if no activiy.
    static PassiveTimer idle_timer;
    if(idle_timer.timeMillis() >= 5000) {
        sio::println(F("waiting..."));
        idle_timer.restart();
    }

    // Handle LIN decoder error flags.
    {
        // Used to trigger periodic error printing.
        static PassiveTimer lin_errors_timeout;
        // Accomulates error flags until next printing.
        static uint8_t pending_lin_errors = 0;

        const uint8_t new_lin_errors = lin_processor::getAndClearErrorFlags();
        if(new_lin_errors) {
            // Make the ERRORS led blinking.
            errors_activity_led.action();
            idle_timer.restart();
        }

        // If pending errors and time to print then print and clear.
        pending_lin_errors |= new_lin_errors;
        if(pending_lin_errors && lin_errors_timeout.timeMillis() > 1000) {
            sio::print(F("LIN errors: "));
            lin_processor::printErrorFlags(pending_lin_errors);
            sio::println();
            lin_errors_timeout.restart();
            pending_lin_errors = 0;
        }
    }

    // Handle recieved LIN frames.
    LinFrame frame;
    if(lin_processor::readNextFrame(&frame)) {
        const bool frameOk = frame.isValid();

        if(!frameOk) {
            // Make the ERRORS frame blinking.
            errors_activity_led.action();
        }

        // Print frame to serial port.
        for(int i = 0; i < frame.num_bytes(); i++) {
            if(i > 0) {
                sio::printchar(' ');
            }
            sio::printhex2(frame.get_byte(i));
        }
        if(!frameOk) {
            sio::print(F(" ERR"));
        }
        sio::println();
        // Supress the 'waiting' messages.
        idle_timer.restart();

        if(frameOk) {
            // Inform the custom logic about the incoming frame.
            custom_module::frameArrived(frame);
        }
    }
}

__attribute__((OS_main)) int main(void) {
    setup();

    while(1) {
        loop();
    }
}


// lin_stack lin = lin_stack(Serial, 19200, 7);
// SoftwareSerial swSerial(SW_RX_PIN, SW_TX_PIN);

// #define NUM_LEDS 10

// int counter = 0;
// int i = 0;
// CRGB leds[NUM_LEDS];

// enum Color : uint8_t { black = 0, red, green, blue, turquoise, magenta, yellow, white, max };

// Color operator++(Color& orig, int inc) {
//     Color next = static_cast<Color>(orig + inc);
//     if(next == max)
//         next = white;
//     return next;
// }

// Color operator--(Color& orig, int dec) {
//     if(static_cast<uint8_t>(orig) <= (dec - 1))
//         return static_cast<Color>(max - dec);
//     return static_cast<Color>(orig - dec);
// }


// void setup() {
//     swSerial.begin(9600);

//     pinMode(HW_RX_PIN, INPUT_PULLUP);

//     lin.setupSerial();
//     lin.sleep(false);

//     FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
//     FastLED.setBrightness(50);
// }

// void loop() {
//     static Color color = black;
//     uint8_t buffer[32];

//     // wait for break condition
//     lin.waitBreak(lin_stack::MAX_DELAY);

//     auto read = lin.readStream(buffer, sizeof(buffer));
//     if(read == 0) {
//         // we did not get anything, retry
//         return;
//     }

//     auto sync = buffer;
//     while(*sync != 0x55)
//         sync++;
//     const auto id = sync + 1;
//     const auto data = id + 1;

//     // swSerial.println(*id, 16);
//     // swSerial.println(data[1],16);

//     if(*id == 0x03) {
//         uint8_t taste56 = data[1];
//         if(taste56 & _BV(0)) {
//             // 5 er touch
//             swSerial.println("5 touch");
//         } else if(taste56 & _BV(1)) {
//             // 5 er drück
//             swSerial.println("5 drück");
//             color--;
//         }
//         if(taste56 & _BV(2)) {
//             // 6 er touch
//             swSerial.println("6 touch");
//         } else if(taste56 & _BV(3)) {
//             // 6 er drück
//             swSerial.println("6 drück");
//             color++;
//         }
//     }

//     swSerial.println(counter);
//     swSerial.print(F("Rec:"));
//     for(auto i = 0; i < read; i++) {
//         swSerial.print(F(" 0x"));
//         swSerial.print(buffer[i], 16);
//     }
//     swSerial.println();
//     // delay(100);
//     // put your main code here, to run repeatedly:7

//     CRGB c = CRGB::Black;
//     switch(color) {
//         case black:
//             c = CRGB(0, 0, 0);
//             break;

//         case red:
//             c = CRGB(255, 0, 0);
//             break;

//         case green:
//             c = CRGB(0, 255, 0);
//             break;

//         case blue:
//             c = CRGB(0, 0, 255);
//             break;

//         case turquoise:
//             c = CRGB(0, 255, 255);
//             break;

//         case magenta:
//             c = CRGB(255, 0, 255);
//             break;

//         case yellow:
//             c = CRGB(255, 255, 0);
//             break;

//         case white:
//             c = CRGB(255, 255, 255);
//             break;

//         default:
//             swSerial.print("Color is shit!!!! Laszlo fucked up!!!");
//             return;
//     }

//     fill_solid(leds, sizeof(leds), c);
//     FastLED.show();
// }