#ifndef TIMER_H
#define TIMER_H

#include <iostream>
#include "interrupts.h"

class Timer {
    private:
        uint8_t div;    // clock divider
        uint8_t tima;   // timer counter
        uint8_t tma;    // timer module
        uint8_t tac;    // timer controller

        uint8_t speed;
        uint8_t started; 
        uint8_t tick;

        Interrupts * interrupt;
        unsigned int * cycles;
    public:
        unsigned int time;
        unsigned int change;

        Timer(Interrupts * interrupt, unsigned int * cycles);

        void setDiv(uint8_t value);
        uint8_t getDiv();

        void setTima(uint8_t value);
        uint8_t getTima(); 

        void setTma(uint8_t value);
        uint8_t getTma();

        void setTac(uint8_t value);
        uint8_t getTac();

        void doTick();
        void doCycle();
};

#endif