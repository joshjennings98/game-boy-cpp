#ifndef CPU_H
#define CPU_H

#include<iostream>
#include<array>
#include "interrupts.h"
#include "ram.h"

enum Registers8 { A, F, B, C, D, E, H, L };
enum Registers16{ AF, BC, DE, HL, SP, PC };
enum Flag { Zero, Subtract, HalfCarry, Carry };

enum DebugSetting { Debug, NoDebug };

class CPU {
    private:
        std::array<uint8_t, 8> registers;
        uint16_t stackPointer;
        uint16_t programCounter;
        unsigned int * cycles;
        bool debug;
    public:
        Interrupts * interrupts;
        Timer * timers;
        LCD * lcd;
        RAM * ram;

        bool halted;

        CPU(Interrupts * interrupts, Timer * timers, LCD * lcd, RAM * ram, unsigned int * cycles, DebugSetting debug = NoDebug);
        void set(Registers8 reg, uint8_t value);
        void set(Registers16 reg, uint16_t value);
        void set(Flag flag, bool value);
        uint8_t get(Registers8 reg);
        uint16_t get(Registers16 reg);
        bool get(Flag flag);
        void increment(Registers8 reg, int i = 1);
        void increment(Registers16 reg, int i = 1);
        void decrement(Registers8 reg, int i = 1);
        void decrement(Registers16 reg, int i = 1);
        void cpuInterrupt(uint16_t address);
        void interruptCycle();
        unsigned int getCycles();
        void cpuCycle();
        void cbPrefix(uint8_t inst);
};

#endif