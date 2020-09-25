#include <iostream>
#include "interrupts.h"
#include "cpu.h"

void Interrupts::doCycle(CPU * cpu) {
    if (pending == 1) {
        pending -= 1;
    } else if (master && enable && flags) {

        if (enable & flags & vblank) {
            flags &= ~vblank; // turn off specific flag
            cpu->cpuInterrupt(0x40);
        }

        if (enable & flags & lcdstat) {
            flags &= ~lcdstat;
            cpu->cpuInterrupt(0x48);
        }

        if (enable & flags & timer) {
            flags &= ~timer;
            cpu->cpuInterrupt(0x50);
        }
        
        if (enable & flags & serial) {
            flags &= ~serial;
            cpu->cpuInterrupt(0x58);
        }

        if (enable & flags & joypad) {
            flags &= ~joypad;
            cpu->cpuInterrupt(0x60);
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

void Interrupts::set(InterruptFlag flag, bool value)
{
    switch(flag) {
        case(Master):
            master = value;
            break;
        case(Enable):
            enable = value;
            break;
        case(Pending):
            pending = value;
            break;
        case(Flags):
            flags = value;
            break;
    }
}

uint8_t Interrupts::get(InterruptFlag flag)
{
    switch(flag) {
        case(Master):
            return master;
        case(Enable):
            return enable;
        case(Pending):
            return pending;
        case(Flags):
            return flags;
    }
}
