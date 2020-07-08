#ifndef INTERRUPTS_H
#define INTERRUPTS_H

#include <iostream>

class Interrupts {
    private:
        const uint8_t vblank = 0x1; // 1 << 0
        const uint8_t lcdstat = 0x2; // 1 << 1
        const uint8_t timer = 0x4; // 1 << 2
        const uint8_t serial = 0x8; // 1 << 3
        const uint8_t joypad = 0x10; // 1 << 4

        uint8_t master;
        uint8_t enable;
        uint8_t flags;
        uint8_t pending;
    public:
        void doCycle();
};

#endif