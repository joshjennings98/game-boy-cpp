#ifndef GAMEBOY_H
#define GAMEBOY_H

#include <string>

#include "ram.h"
#include "rom.h"
#include "interrupts.h"
#include "timer.h"
#include "lcd.h"
#include "sdl.h"
#include "cpu.h"

class GameBoy {
    private:
        unsigned int cycles;
        bool debug;
        ROM rom;

        Interrupts * interrupts;
        Display * display;
        LCD * lcd;
        Timer * timers;
        RAM * ram;
        CPU * cpu;
    public:
        GameBoy(DebugSetting debug = NoDebug);
        void load(std::string filename);
        void run();
};

#endif