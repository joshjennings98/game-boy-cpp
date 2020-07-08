#include <iostream>
#include "interrupts.h"

void Interrupts::doCycle() {
    if (pending == 1) {
        pending = 0;
    } else if (master && enable && flags) {

        if (enable & flags & vblank) {
            flags &= ~vblank; // turn off specific flag
            // do cpuInterrupt 0x40
        }

        if (enable & flags & lcdstat) {
            flags &= ~lcdstat;
            // do cpuInterrupt(0x48
        }

        if (enable & flags & timer) {
            flags &= ~timer;
            // do cpuInterrupt 0x50
        }
        
        if (enable & flags & serial) {
            flags &= ~serial;
            // do cpuInterrupt 0x58
        }

        if (enable & flags & joypad) {
            flags &= ~joypad;
            // do cpuInterrupt 0x60
        }
    }
}

uint8_t Interrupts::getFlags()
{
    return flags;
}

void Interrupts::updateFlags(InterruptTypes interrupt)
{
    flags |= interrupt;
}