#include <iostream>
#include <array>
#include "cpu.h"
#include "interrupts.h"
#include "ram.h"

CPU::CPU()
{
    interrupts.setMaster(1);
    interrupts.setEnable(0);
    interrupts.setFlags(0);

    set(AF, 0x01B0);
  	set(BC, 0x0013);
  	set(DE, 0x00D8);
  	set(HL, 0x014D);

    stackPointer = 0xFFFE;
    programCounter = 0x0100;
    cycles = 0;
}

void CPU::set(Registers8 reg, uint8_t value)
{
    registers[reg] = value;
}

void CPU::set(Registers16 reg, uint16_t value)
{
    uint8_t lo = value & 0xFF;
    uint8_t hi = (value & 0xFF00) >> 8;
    
    switch (reg) {
        case(AF):
            set(A, hi);
            set(F, lo);
            break;
        case(BC):
            set(B, hi);
            set(C, lo);
            break;
        case(DE):
            set(D, hi);
            set(E, lo);
            break;
        case(HL):
            set(H, hi);
            set(L, lo);
            break;
        case(SP):
            stackPointer = value;
            break;
        case(PC):
            programCounter = value;
            break;
        default:
            break;
    }
}

void CPU::set(Flag flag, bool value)
{
    registers[F] |= value ? (0x80 >> flag) : ~(0x80 >> flag);
}

uint8_t CPU::get(Registers8 reg)
{
    return registers[reg];
}

uint16_t CPU::get(Registers16 reg)
{
    switch (reg) {
        case(AF):
            return (registers[A] << 8) | registers[F];
        case(BC):
            return (registers[B] << 8) | registers[C];
        case(DE):
            return (registers[D] << 8) | registers[E];
        case(HL):
            return (registers[H] << 8) | registers[L];
        case(SP):
            return stackPointer;
        case(PC):
            return programCounter;
        default:
            return 0x0;
    }
}

bool CPU::get(Flag flag)
{
    return registers[F] & (0x80 >> flag);
}

void CPU::cpuInterrupt(uint16_t address)
{
    uint16_t sp = get(SP), pc = get(PC);
    
    interrupts.setMaster(0);
    set(SP, sp - 2);
    ram.writeWord(sp - 2, pc);
    set(PC, address);
}

unsigned int CPU::getCycles()
{
    return cycles;
}

void CPU::cpuCycle()
{

}

void CPU::cbPrefix(uint8_t inst)
{

}
