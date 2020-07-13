#ifndef INTERRUPTS_H
#define INTERRUPTS_H

#include <iostream>

// vblank  = 1 << 0
// lcdstat = 1 << 1
// time    = 1 << 2
// serial  = 1 << 3
// joypad  = 1 << 4
enum InterruptTypes { vblank = 0x1, lcdstat = 0x2, timer = 0x4, serial = 0x8, joypad = 0x10 };
enum InterruptFlag { Master, Enable, Pending, Flags };

class Interrupts {
    private:
        uint8_t master;
        uint8_t enable;
        uint8_t flags;
        uint8_t pending;
    public:
        void doCycle();
        uint8_t getFlags();
        void updateFlags(InterruptTypes interrupt);
        void set(InterruptFlag flag, bool value);
};  

#endif