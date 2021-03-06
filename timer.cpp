#include <iostream>
#include "timer.h"

Timer::Timer(Interrupts * interrupt, unsigned int * cycles)
{
    time = 0;
    change = 0;
    
    this->interrupt = interrupt;
    this->cycles = cycles;
}

void Timer::setDiv(uint8_t value)
{
    div = 0x0; // setting div to anything makes it zero
}

uint8_t Timer::getDiv()
{
    return div;
}

void Timer::setTima(uint8_t value)
{
    tima = value;
}

uint8_t Timer::getTima()
{
    return tima;
} 

void Timer::setTma(uint8_t value)
{
    tma = value;
}

uint8_t Timer::getTma()
{
    return tma;
}

void Timer::setTac(uint8_t value)
{
    tac = value;
    started = value & 4;

    switch (value & 3) {
        case 0:
            speed = 1;
            break;
        case 1:
            speed = 64;
            break;
        case 2:
            speed = 16;
            break;
        case 3:
            speed = 4;
            break;
    }
}

uint8_t Timer::getTac()
{
    return tac;
}

void Timer::doTick()
{
    tick++;
    
    if (started && tick == 0x10) {
        div++;
        tick = 0x0;
    }

    if (started && tick == speed) {
        tima++;
        tick = 0x0;
    }

    if (started && tima == 0x100) {
        interrupt->updateFlags(timer);
        tima = tma;
    }
}

void Timer::doCycle()
{
    unsigned int delta = *cycles - time;
    time = *cycles;

    change += delta * 4;

    if (change >= 0x10) {
        doTick();
        change -= 0x10;
    }
}