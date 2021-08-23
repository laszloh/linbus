#pragma once

#include <Arduino.h>

struct InterruptLock {
public:
    static inline void on() { sei(); }
    static inline void off() { cli(); }

protected:
    InterruptLock() : sreg_(SREG) { asm("cli"); }
    ~InterruptLock() { SREG = sreg_; }

private:
    uint8_t sreg_;
};
