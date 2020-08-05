#include <iostream>
#include <array>
#include "cpu.h"
#include "interrupts.h"
#include "ram.h"

CPU::CPU(Interrupts * interrupts, Timer * timers, LCD * lcd, RAM * ram, unsigned int * cycles, DebugSetting debug)
{
    this->debug = (debug == Debug);
    this->ram = ram;
    this->interrupts = interrupts;
    this->timers = timers;
    this->lcd = lcd;

    interrupts->set(Master, 1);
    interrupts->set(Enable, 0);
    interrupts->set(Flags, 0);

    set(AF, 0x01B0);
  	set(BC, 0x0013);
  	set(DE, 0x00D8);
  	set(HL, 0x014D);

    stackPointer = 0xFFFE;
    programCounter = 0x0100;
    this->cycles = cycles;
    halted = false;
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
    //registers[F] |= value ? (0x80 >> flag) : ~(0x80 >> flag);
    //registers[F] = (registers[F] & ((~(0x80 >> flag) << 4) & 0xff)) | (value << (7 - flag));
    // registers[F] = (registers[F] & ~(1 << (7 - value)) | (value << (7 - flag)));
    
    switch(flag) {
        case(Zero): 
            registers[F] = (registers[F] & 0x7F) | (value << 7);
            break;
        case(Subtract): 
            registers[F] = (registers[F] & 0xBF) | (value << 6);
            break;
        case(HalfCarry): 
            registers[F] = (registers[F] & 0xDF) | (value << 5);
            break;
        case(Carry): 
            registers[F] = (registers[F] & 0xEF) | (value << 4);
            break;
    }
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
    interrupts->set(Master, 0);
    decrement(SP, 2);
    ram->writeWord(get(SP), get(PC));
    set(PC, address);
}

void CPU::interruptCycle() {
    uint8_t master = interrupts->get(Master);
    uint8_t enable = interrupts->get(Enable);
    uint8_t pending = interrupts->get(Pending);
    uint8_t flags = interrupts->get(Flags);

    if (pending == 1) {
        interrupts->set(Pending, 0);
    } else if (master && enable && flags) {

        if (enable & flags & vblank) {
            flags &= ~vblank; // turn off specific flag
            interrupts->set(Flags, flags);
            cpuInterrupt(0x40);
        }

        if (enable & flags & lcdstat) {
            flags &= ~lcdstat;
            interrupts->set(Flags, flags);
            cpuInterrupt(0x48);
        }

        if (enable & flags & timer) {
            flags &= ~timer;
            interrupts->set(Flags, flags);
            cpuInterrupt(0x50);
        }
        
        if (enable & flags & serial) {
            flags &= ~serial;
            interrupts->set(Flags, flags);
            cpuInterrupt(0x58);
        }

        if (enable & flags & joypad) {
            flags &= ~joypad;
            interrupts->set(Flags, flags);
            cpuInterrupt(0x60);
        }
    }
}

unsigned int CPU::getCycles()
{
    return *cycles;
}

void CPU::increment(Registers8 reg, int i)
{
    set(reg, get(reg) + i);
}

void CPU::increment(Registers16 reg, int i)
{
    set(reg, get(reg) + i);
}

void CPU::decrement(Registers8 reg, int i)
{
    set(reg, get(reg) - i);
}

void CPU::decrement(Registers16 reg, int i)
{
    set(reg, get(reg) - i);
}

void CPU::cpuCycle()
{
    if (halted) {
        *cycles += 1;
        return;
    }

    int i;
    uint8_t s;
    uint16_t t;
    unsigned int u;
    uint16_t pc = get(PC);
    uint8_t instruction = ram->readByte(pc);

    if (debug) {
        //std::cin.ignore();

        std::cout << std::hex << (int) ram->readByte(pc) << " " << (int) ram->readByte(pc + 1) << " " << (int) ram->readByte(pc + 2) << std::endl << std::endl;
        std::cout << std::hex << "PC: 0x" << (int) get(PC) << std::endl << "SP: 0x" << (int) get(SP) << std::endl;
        std::cout << std::hex << "HL: 0x" << (((int) get(H)) < 0x10 ? "0" : "") << (int) get(H) << "" << (((int) get(L)) < 0x10 ? "0" : "") << (int) get(L) << std::endl;
        std::cout << std::hex 
            << "A: 0x" << (((int) get(A)) < 0x10 ? "0" : "") << (int) get(A) 
            << " F: 0x" << (((int) get(F)) < 0x10 ? "0" : "") << (int) get(F) 
            << " B: 0x" << (((int) get(B)) < 0x10 ? "0" : "") << (int) get(B) 
            << " C: 0x" << (((int) get(C)) < 0x10 ? "0" : "") << (int) get(C)
            << " D: 0x" << (((int) get(D)) < 0x10 ? "0" : "") << (int) get(D) 
            << " E: 0x" << (((int) get(E)) < 0x10 ? "0" : "") << (int) get(E) << std::endl;
    }

    switch (instruction) {
        case 0x00: if (debug) std::cout << "NOP" << std::endl;
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x01: if (debug) std::cout << std::endl << "LD BC, 0x" << (int) ram->readByte(pc+2) << (int) ram->readByte(pc+1) <<"" << std::endl;
            set(BC, ram->readWord(pc + 1));
            increment(PC, 3);
            *cycles += 3;
            break;
        case 0x02: if (debug) std::cout << std::endl << "LD (BC), A" << std::endl;
            ram->writeByte(get(BC), get(A));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x03: if (debug) std::cout << std::endl << "INC BC" << std::endl;
            set(BC, (get(BC) + 1));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x04: if (debug) std::cout << std::endl << "INC B" << std::endl;
            increment(B, 1);
            set(Zero, !get(B));
            set(Subtract, 0);
            set(HalfCarry, ((get(B) & 0xF) < ((get(B) - 1) & 0xF)));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x05: if (debug) std::cout << std::endl << "DEC B" << std::endl;
            decrement(B, 1);
            set(Zero, !get(B));
            set(Subtract, 1);
            set(HalfCarry, ((get(B) & 0xF) == 0xF));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x06: if (debug) std::cout << std::endl << "LD B, 0x" << (int) ram->readByte(pc+1) <<"" << std::endl;
            set(B, ram->readByte(pc + 1));
            increment(PC, 2);
            *cycles += 2;
            break;
        case 0x07: if (debug) std::cout << std::endl << "RLCA" << std::endl;
            s = get(A);
            s = (s >> 7);
            set(A, (get(A) << 1) | s);
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x08: if (debug) std::cout << std::endl << "LD (" << (int) ram->readByte(pc+2) << (int) ram->readByte(pc+1) <<"), SP" << std::endl;
            ram->writeWord(ram->readWord(pc + 1), get(SP));
            increment(PC, 3);
            *cycles += 5;
            break;
        case 0x09: if (debug) std::cout << std::endl << "ADD HL, BC" << std::endl;
            t = get(HL);
            set(HL, (t + get(BC)));
            set(Subtract, 0);
            set(HalfCarry, ((get(HL) & 0xFFF) < (t & 0xFFF)));
            set(Carry, ((get(HL) & 0xFFFF) < (t & 0xFFFF)));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x0A: if (debug) std::cout << std::endl << "LD A, (BC)" << std::endl;
            set(A, ram->readByte(get(BC)));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x0B: if (debug) std::cout << std::endl << "DEC BC" << std::endl;
            set(BC, (get(BC) - 1));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x0C: if (debug) std::cout << std::endl << "INC C" << std::endl;
            increment(C, 1);
            set(Zero, !get(C));
            set(Subtract, 0);
            set(HalfCarry, ((get(C) & 0xF) < ((get(C) - 1) & 0xF)));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x0D: if (debug) std::cout << std::endl << "DEC C" << std::endl;
            decrement(C, 1);
            set(Zero, !get(C));
            set(Subtract, 1);
            set(HalfCarry, ((get(C) & 0xF) == 0xF));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x0E: if (debug) std::cout << std::endl << "LD C, 0x" << (int) ram->readByte(pc+1) << "" << std::endl;
            set(C, ram->readByte(pc + 1));
            increment(PC, 2);
            *cycles += 2;
            break;
        case 0x0F: if (debug) std::cout << std::endl << "RRCA" << std::endl;
            s = (get(A) & 0x1);
            set(A, ((get(A) >> 1) | (s << 7)));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x10: if (debug) std::cout << std::endl << "STOP" << std::endl;
            halted = 1;
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x11: if (debug) std::cout << std::endl << "LD DE, 0x" << (int) ram->readByte(pc+2) << (int) ram->readByte(pc+1) <<"" << std::endl;
            set(DE, ram->readWord(pc + 1));
            increment(PC, 3);
            *cycles += 3;
            break;
        case 0x12: if (debug) std::cout << std::endl << "LD (DE), A" << std::endl;
            ram->writeByte(get(DE), get(A));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x13: if (debug) std::cout << std::endl << "INC DE" << std::endl;
            set(DE, (get(DE) + 1));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x14: if (debug) std::cout << std::endl << "INC D" << std::endl;
            increment(D, 1);
            set(Zero, !get(D));
            set(Subtract, 0);
            set(HalfCarry, ((get(D) & 0xF) < ((get(D) - 1) & 0xF)));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x15: if (debug) std::cout << std::endl << "DEC D" << std::endl;
            decrement(D, 1);
            set(Zero, !get(D));
            set(Subtract, 1);
            set(HalfCarry, ((get(D) & 0xF) == 0xF));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x16: if (debug) std::cout << std::endl << "LD D, 0x" << (int) ram->readByte(pc+1) <<"" << std::endl;
            set(D, ram->readByte(pc + 1));
            increment(PC, 2);
            *cycles += 2;
            break;
        case 0x17: if (debug) std::cout << std::endl << "RLA" << std::endl;
            s = get(A);
            set(A, ((get(A) << 1) | get(Carry)));
            set(Carry, s >> 7);
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x18: if (debug) std::cout << std::endl << "JR 0x" << (int) ram->readByte(pc + 1) << std::endl;
            increment(PC, (signed char)ram->readByte(pc + 1) + 2);
            *cycles += 3;
            break;
        case 0x19: if (debug) std::cout << std::endl << "ADD HL, DE" << std::endl;
            t = get(HL);
            set(HL, (t + get(DE)));
            set(Subtract, 0);
            set(HalfCarry, ((get(HL) & 0xFFF) < (t & 0xFFF)));
            set(Carry, ((get(HL) & 0xFFFF) < (t & 0xFFFF)));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x1A: if (debug) std::cout << std::endl << "LD A, (DE)" << std::endl;
            set(A, ram->readByte(get(DE)));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x1B: if (debug) std::cout << std::endl << "DEC DE" << std::endl;
            set(DE, (get(DE) - 1));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x1C: if (debug) std::cout << std::endl << "INC E" << std::endl;
            increment(E, 1);
            set(Zero, !get(E));
            set(Subtract, 0);
            set(HalfCarry, ((get(E) & 0xF) < ((get(E) - 1) & 0xF)));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x1D: if (debug) std::cout << std::endl << "DEC E" << std::endl;
            decrement(E, 1);
            set(Zero, !get(E));
            set(Subtract, 1);
            set(HalfCarry, ((get(E) & 0xF) == 0xF));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x1E: if (debug) std::cout << std::endl << "LD E, 0x" << (int) ram->readByte(pc+1) <<"" << std::endl;
            set(E, ram->readByte(pc + 1));
            increment(PC, 2);
            *cycles += 2;
            break;
        case 0x1F: if (debug) std::cout << std::endl << "RRA" << std::endl;
            s = (get(A) & 0x1);
            set(A, (get(A) >> 1) | (get(Carry) << 7));
            set(Carry, s);
            set(Zero, 0);
            set(Subtract, 0);
            set(HalfCarry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x20: if (debug) std::cout << std::endl << "JP NZ" << std::endl;
            if (get(Zero) == 0) {
              increment(PC, (signed char)ram->readByte(pc + 1) + 2);
              *cycles += 3;
            } else {
              increment(PC, 2);
              *cycles += 2;
            }
            break;
        case 0x21: if (debug) std::cout << std::endl << "LD HL, 0x" << (int) ram->readByte(pc+2) << (int) ram->readByte(pc+1) <<"" << std::endl;
            set(HL, ram->readWord(pc + 1));
            increment(PC, 3);
            *cycles += 3;
            break;
        case 0x22: if (debug) std::cout << std::endl << "LDI (HL), A" << std::endl;
            ram->writeByte(get(HL), get(A));
            set(HL, (get(HL) + 1));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x23: if (debug) std::cout << std::endl << "INC HL" << std::endl;
            set(HL, (get(HL) + 1));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x24: if (debug) std::cout << std::endl << "INC H" << std::endl;
            increment(H, 1);
            set(Zero, !get(H));
            set(Subtract, 0);
            set(HalfCarry, ((get(H) & 0xF) < ((get(H) - 1) & 0xF)));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x25: if (debug) std::cout << std::endl << "DEC H" << std::endl;
            decrement(H, 1);
            set(Zero, !get(H));
            set(Subtract, 1);
            set(HalfCarry, ((get(H) & 0xF) == 0xF));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x26: if (debug) std::cout << std::endl << "LD H, 0x" << (int) ram->readByte(pc+1) <<"" << std::endl;
            set(H, ram->readByte(pc + 1));
            increment(PC, 2);
            *cycles += 2;
            break;
        case 0x27: if (debug) std::cout << std::endl << "DAA" << std::endl;
            u = get(A);
            if (get(Subtract)) {
              if(get(HalfCarry))
                u = (u - 0x06)&0xFF;
              if(get(Carry))
                u -= 0x60;
            } else {
              if(get(HalfCarry) || (u & 0xF) > 9)
                u += 0x06;
              if(get(Carry) || u > 0x9F)
                u += 0x60;
            }
            set(A, u);
            set(HalfCarry, 0);
            set(Zero, !get(A));
            set(Carry, (u >= 0x100));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x28: if (debug) std::cout << std::endl << "JP Z" << std::endl;
            if (get(Zero) == 1) {
              increment(PC, (signed char)ram->readByte(pc + 1) + 2);
              *cycles += 3;
            } else {
              increment(PC, 2);
              *cycles += 2;
            }
            break;
        case 0x29: if (debug) std::cout << std::endl << "ADD HL, HL" << std::endl;
            t = get(HL) * 2;
            set(Subtract, 0);
            set(HalfCarry, ((get(HL) & 0x7FF) > (t & 0x7FF)));
            set(Carry, ((get(HL) & 0xFFFF) > (t & 0xFFFF)));
            set(HL, t);
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x2A: if (debug) std::cout << std::endl << "LDI A, (HL)" << std::endl;
            set(A, ram->readByte(get(HL)));
            set(HL, (get(HL) + 1));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x2B: if (debug) std::cout << std::endl << "DEC HL" << std::endl;
            set(HL, (get(HL) - 1));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x2C: if (debug) std::cout << std::endl << "INC L" << std::endl;
            increment(L, 1);
            set(Zero, !get(L));
            set(Subtract, 0);
            set(HalfCarry, ((get(L) & 0xF) < ((get(L) - 1) & 0xF)));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x2D: if (debug) std::cout << std::endl << "DEC L" << std::endl;
            decrement(L, 1);
            set(Zero, !get(L));
            set(Subtract, 1);
            set(HalfCarry, ((get(L) & 0xF) == 0xF));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x2E: if (debug) std::cout << std::endl << "LD L, 0x" << (int) ram->readByte(pc+1) <<"" << std::endl;
            set(L, ram->readByte(pc + 1));
            increment(PC, 2);
            *cycles += 2;
            break;
        case 0x2F:  // CPL
            set(A, ~get(A));
            set(Subtract, 1);
            set(HalfCarry, 1);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x30: if (debug) std::cout << std::endl << "JP NC" << std::endl;
            if (get(Carry) == 0) {
              increment(PC, (signed char)ram->readByte(pc + 1) + 2);
              *cycles += 3;
            } else {
              increment(PC, 2);
              *cycles += 2;
            }
            break;
        case 0x31: if (debug) std::cout << std::endl << "LD SP, 0x" << (int) ram->readByte(pc+2) << (int) ram->readByte(pc+1) <<"" << std::endl;
            set(SP, ram->readWord(pc + 1));
            increment(PC, 3);
            *cycles += 3;
            break;
        case 0x32: if (debug) std::cout << std::endl << "LDD (HL), A" << std::endl;
            t = get(HL);
            ram->writeByte(t, get(A));
            set(HL, (t - 1));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x33: if (debug) std::cout << std::endl << "INC SP" << std::endl;
            increment(SP, 1);
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x34: if (debug) std::cout << std::endl << "INC (HL)" << std::endl;
            s = ram->readByte(get(HL)) + 1;
            ram->writeByte(get(HL), s);
            set(Zero, !s);
            set(Subtract, 0);
            set(HalfCarry, ((s & 0xF) < ((s-1) & 0xF)));
            increment(PC, 1);
            *cycles += 3;
            break;
        case 0x35: if (debug) std::cout << std::endl << "DEC (HL)" << std::endl;
            s = ram->readByte(get(HL)) - 1;
            ram->writeByte(get(HL), s);
            set(Zero, !s);
            set(Subtract, 1);
            set(HalfCarry, ((s & 0xF) == 0xF));
            increment(PC, 1);
            *cycles += 3;
            break;
        case 0x36: if (debug) std::cout << std::endl << "LD (HL), 0x" << (int) ram->readByte(pc+1) <<"" << std::endl;
            ram->writeByte(get(HL), ram->readByte(pc + 1));
            increment(PC, 2);
            *cycles += 3;
            break;
        case 0x37: if (debug) std::cout << std::endl << "SCF" << std::endl;
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 1);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x38: if (debug) std::cout << std::endl << "JP C" << std::endl;
            if (get(Carry) == 1) {
              increment(PC, (signed char)ram->readByte(pc + 1) + 2);
              *cycles += 3;
            } else {
              increment(PC, 2);
              *cycles += 2;
            }
            break;
        case 0x39: if (debug) std::cout << std::endl << "ADD HL, SP" << std::endl;
            t = get(HL);
            set(HL, t + get(SP));
            set(Subtract, 0);
            set(HalfCarry, ((get(HL) & 0xFFF) < (t & 0xFFF)));
            set(Carry, ((get(HL) & 0xFFFF) < (t & 0xFFFF)));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x3A: if (debug) std::cout << std::endl << "LDD A, (HL)" << std::endl;
            set(A, ram->readByte(get(HL)));
            set(HL, get(HL) - 1);
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x3B: if (debug) std::cout << std::endl << "DEC SP" << std::endl;
            decrement(SP, 1);
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x3C: if (debug) std::cout << std::endl << "INC A" << std::endl;
            increment(A, 1);
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, ((get(A) & 0xF) < ((get(A) - 1) & 0xF)));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x3D: if (debug) std::cout << std::endl << "DEC A" << std::endl;
            decrement(A, 1);
            set(Zero, !get(A));
            set(Subtract, 1);
            set(HalfCarry, ((get(A) & 0xF) == 0xF));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x3E: if (debug) std::cout << std::endl << "LD A, 0x" << (int) ram->readByte(pc+1) <<"" << std::endl;
            set(A, ram->readByte(pc + 1));
            increment(PC, 2);
            *cycles += 2;
            break;
        case 0x3F: if (debug) std::cout << std::endl << "CCF" << std::endl;
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, !get(Carry));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x40: if (debug) std::cout << std::endl << "LD B, B" << std::endl;
            set(B, get(B));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x41: if (debug) std::cout << std::endl << "LD B, C" << std::endl;
            set(B, get(C));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x42: if (debug) std::cout << std::endl << "LD B, D" << std::endl;
            set(B, get(D));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x43: if (debug) std::cout << std::endl << "LD B, E" << std::endl;
            set(B, get(E));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x44: if (debug) std::cout << std::endl << "LD B, H" << std::endl;
            set(B, get(H));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x45: if (debug) std::cout << std::endl << "LD B, L" << std::endl;
            set(B, get(L));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x46: if (debug) std::cout << std::endl << "LD B, (HL)" << std::endl;
            set(B, ram->readByte(get(HL)));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x47: if (debug) std::cout << std::endl << "LD B, A" << std::endl;
            set(B, get(A));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x48: if (debug) std::cout << std::endl << "LD C, B" << std::endl;
            set(C, get(B));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x49: if (debug) std::cout << std::endl << "LD C, C" << std::endl;
            set(C, get(C));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x4A: if (debug) std::cout << std::endl << "LD C, D" << std::endl;
            set(C, get(D));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x4B: if (debug) std::cout << std::endl << "LD C, E" << std::endl;
            set(C, get(E));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x4C: if (debug) std::cout << std::endl << "LD C, H" << std::endl;
            set(C, get(H));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x4D: if (debug) std::cout << std::endl << "LD C, L" << std::endl;
            set(C, get(L));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x4E: if (debug) std::cout << std::endl << "LD C, (HL)" << std::endl;
            set(C, ram->readByte(get(HL)));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x4F: if (debug) std::cout << std::endl << "LD C, A" << std::endl;
            set(C, get(A));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x50: if (debug) std::cout << std::endl << "LD D, B" << std::endl;
            set(D, get(B));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x51: if (debug) std::cout << std::endl << "LD D, C" << std::endl;
            set(D, get(C));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x52: if (debug) std::cout << std::endl << "LD D, D" << std::endl;
            set(D, get(D));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x53: if (debug) std::cout << std::endl << "LD D, E" << std::endl;
            set(D, get(E));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x54: if (debug) std::cout << std::endl << "LD D, H" << std::endl;
            set(D, get(H));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x55: if (debug) std::cout << std::endl << "LD D, L" << std::endl;
            set(D, get(L));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x56: if (debug) std::cout << std::endl << "LD D, (HL)" << std::endl;
            set(D, ram->readByte(get(HL)));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x57: if (debug) std::cout << std::endl << "LD D, A" << std::endl;
            set(D, get(A));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x58: if (debug) std::cout << std::endl << "LD E, B" << std::endl;
            set(E, get(B));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x59: if (debug) std::cout << std::endl << "LD E, C" << std::endl;
            set(E, get(C));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x5A: if (debug) std::cout << std::endl << "LD E, D" << std::endl;
            set(E, get(D));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x5B: if (debug) std::cout << std::endl << "LD E, E" << std::endl;
            set(E, get(E));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x5C: if (debug) std::cout << std::endl << "LD E, H" << std::endl;
            set(E, get(H));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x5D: if (debug) std::cout << std::endl << "LD E, L" << std::endl;
            set(E, get(L));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x5E: if (debug) std::cout << std::endl << "LD E, (HL)" << std::endl;
            set(E, ram->readByte(get(HL)));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x5F: if (debug) std::cout << std::endl << "LD E, A" << std::endl;
            set(E, get(A));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x60: if (debug) std::cout << std::endl << "LD H, B" << std::endl;
            set(H, get(B));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x61: if (debug) std::cout << std::endl << "LD H, C" << std::endl;
            set(H, get(C));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x62: if (debug) std::cout << std::endl << "LD H, D" << std::endl;
            set(H, get(D));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x63: if (debug) std::cout << std::endl << "LD H, E" << std::endl;
            set(H, get(E));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x64: if (debug) std::cout << std::endl << "LD H, H" << std::endl;
            set(H, get(H));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x65: if (debug) std::cout << std::endl << "LD H, L" << std::endl;
            set(H, get(L));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x66: if (debug) std::cout << std::endl << "LD H, (HL)" << std::endl;
            set(H, ram->readByte(get(HL)));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x67: if (debug) std::cout << std::endl << "LD H, A" << std::endl;
            set(H, get(A));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x68: if (debug) std::cout << std::endl << "LD L, B" << std::endl;
            set(L, get(B));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x69: if (debug) std::cout << std::endl << "LD L, C" << std::endl;
            set(L, get(C));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x6A: if (debug) std::cout << std::endl << "LD L, D" << std::endl;
            set(L, get(D));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x6B: if (debug) std::cout << std::endl << "LD L, E" << std::endl;
            set(L, get(E));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x6C: if (debug) std::cout << std::endl << "LD L, H" << std::endl;
            set(L, get(H));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x6D: if (debug) std::cout << std::endl << "LD L, L" << std::endl;
            set(L, get(L));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x6E: if (debug) std::cout << std::endl << "LD L, (HL)" << std::endl;
            set(L, ram->readByte(get(HL)));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x6F: if (debug) std::cout << std::endl << "LD L, A" << std::endl;
            set(L, get(A));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x70: if (debug) std::cout << std::endl << "LD (HL), B" << std::endl;
            ram->writeByte(get(HL), get(B));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x71: if (debug) std::cout << std::endl << "LD (HL), C" << std::endl;
            ram->writeByte(get(HL), get(C));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x72: if (debug) std::cout << std::endl << "LD (HL), D" << std::endl;
            ram->writeByte(get(HL), get(D));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x73: if (debug) std::cout << std::endl << "LD (HL), E" << std::endl;
            ram->writeByte(get(HL), get(E));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x74: if (debug) std::cout << std::endl << "LD (HL), H" << std::endl;
            ram->writeByte(get(HL), get(H));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x75: if (debug) std::cout << std::endl << "LD (HL), L" << std::endl;
            ram->writeByte(get(HL), get(L));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x76: if (debug) std::cout << std::endl << "HALT" << std::endl;
            halted = 1;
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x77: if (debug) std::cout << std::endl << "LD (HL), A" << std::endl;
            ram->writeByte(get(HL), get(A));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x78: if (debug) std::cout << std::endl << "LD A, B" << std::endl;
            set(A, get(B));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x79: if (debug) std::cout << std::endl << "LD A, C" << std::endl;
            set(A, get(C));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x7A: if (debug) std::cout << std::endl << "LD A, D" << std::endl;
            set(A, get(D));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x7B: if (debug) std::cout << std::endl << "LD A, E" << std::endl;
            set(A, get(E));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x7C: if (debug) std::cout << std::endl << "LD A, H" << std::endl;
            set(A, get(H));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x7D: if (debug) std::cout << std::endl << "LD A, L" << std::endl;
            set(A, get(L));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x7E: if (debug) std::cout << std::endl << "LD A, (HL)" << std::endl;
            set(A, ram->readByte(get(HL)));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x7F: if (debug) std::cout << std::endl << "LD A, A" << std::endl;
            set(A, get(A));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x80: if (debug) std::cout << std::endl << "ADD A, B" << std::endl;
            i = get(A) + get(B);
            set(Zero, !i);
            set(Subtract, 0);
            set(HalfCarry, ((i & 0xF) < (get(A) & 0xF)));
            set(Carry, ((i & 0xFF) < (get(A) & 0xFF)));
            set(A, i);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x81: if (debug) std::cout << std::endl << "ADD A, C" << std::endl;
            i = get(A) + get(C);
            set(Zero, !i);
            set(Subtract, 0);
            set(HalfCarry, ((i & 0xF) < (get(A) & 0xF)));
            set(Carry, ((i & 0xFF) < (get(A) & 0xFF)));
            set(A, i);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x82: if (debug) std::cout << std::endl << "ADD A, D" << std::endl;
            i = get(A) + get(D);
            set(Zero, !i);
            set(Subtract, 0);
            set(HalfCarry, ((i & 0xF) < (get(A) & 0xF)));
            set(Carry, ((i & 0xFF) < (get(A) & 0xFF)));
            set(A, i);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x83: if (debug) std::cout << std::endl << "ADD A, E" << std::endl;
            i = get(A) + get(E);
            set(Zero, !i);
            set(Subtract, 0);
            set(HalfCarry, ((i & 0xF) < (get(A) & 0xF)));
            set(Carry, ((i & 0xFF) < (get(A) & 0xFF)));
            set(A, i);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x84: if (debug) std::cout << std::endl << "ADD A, H" << std::endl;
            i = get(A) + get(H);
            set(Zero, !i);
            set(Subtract, 0);
            set(HalfCarry, ((i & 0xF) < (get(A) & 0xF)));
            set(Carry, ((i & 0xFF) < (get(A) & 0xFF)));
            set(A, i);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x85: if (debug) std::cout << std::endl << "ADD A, L" << std::endl;
            i = get(A) + get(L);
            set(Zero, !i);
            set(Subtract, 0);
            set(HalfCarry, ((i & 0xF) < (get(A) & 0xF)));
            set(Carry, ((i & 0xFF) < (get(A) & 0xFF)));
            set(A, i);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x86: if (debug) std::cout << std::endl << "ADD A, (HL)" << std::endl;
            i = get(A) + ram->readByte(get(HL));
            set(Zero, !i);
            set(Subtract, 0);
            set(HalfCarry, ((i & 0xF) < (get(A) & 0xF)));
            set(Carry, ((i & 0xFF) < (get(A) & 0xFF)));
            set(A, i);
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x87: if (debug) std::cout << std::endl << "ADD A, A" << std::endl;
            i = get(A) + get(A);
            set(Zero, !i);
            set(Subtract, 0);
            set(HalfCarry, ((i & 0xF) < (get(A) & 0xF)));
            set(Carry, ((i & 0xFF) < (get(A) & 0xFF)));
            set(A, i);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x88: if (debug) std::cout << std::endl << "ADC A, B" << std::endl;
            i = get(A) + get(B) + get(Carry);
            set(Zero, !i);
            set(Subtract, 0);
            set(HalfCarry, ((i & 0xF) < (get(A) & 0xF)));
            set(Carry, ((i & 0xFF) < (get(A) & 0xFF)));
            set(A, i);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x89: if (debug) std::cout << std::endl << "ADC A, C" << std::endl;
            i = ((get(A) + get(C) + get(Carry)) >= 0x100);
			set(Subtract, 0);
			set(HalfCarry, (((get(A)&0xF) + (get(C)&0xF) + get(Carry)) >= 0x10));
			set(A, (get(A) + get(C) + get(Carry)));
			set(Carry, i);
			set(Zero, !get(A));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x8A: if (debug) std::cout << std::endl << "ADC A, D" << std::endl;
            i = ((get(A) + get(D) + get(Carry)) >= 0x100);
			set(Subtract, 0);
			set(HalfCarry, (((get(A)&0xF) + (get(D)&0xF) + get(Carry)) >= 0x10));
			set(A, (get(A) + get(D) + get(Carry)));
			set(Carry, i);
			set(Zero, !get(A));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x8B: if (debug) std::cout << std::endl << "ADC A, E" << std::endl;
            i = ((get(A) + get(E) + get(Carry)) >= 0x100);
			set(Subtract, 0);
			set(HalfCarry, (((get(A)&0xF) + (get(E)&0xF) + get(Carry)) >= 0x10));
			set(A, (get(A) + get(E) + get(Carry)));
			set(Carry, i);
			set(Zero, !get(A));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x8C: if (debug) std::cout << std::endl << "ADC A, H" << std::endl;
            i = ((get(A) + get(H) + get(Carry)) >= 0x100);
			set(Subtract, 0);
			set(HalfCarry, (((get(A)&0xF) + (get(H)&0xF) + get(Carry)) >= 0x10));
			set(A, (get(A) + get(H) + get(Carry)));
			set(Carry, i);
			set(Zero, !get(A));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x8D: if (debug) std::cout << std::endl << "ADC A, L" << std::endl;
            i = ((get(A) + get(L) + get(Carry)) >= 0x100);
			set(Subtract, 0);
			set(HalfCarry, (((get(A)&0xF) + (get(L)&0xF) + get(Carry)) >= 0x10));
			set(A, (get(A) + get(L) + get(Carry)));
			set(Carry, i);
			set(Zero, !get(A));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x8E: if (debug) std::cout << std::endl << "ADC A, (HL)" << std::endl;
            s = ram->readByte(get(HL));
            i = ((get(A) + s + get(Carry)) >= 0x100);
			set(Subtract, 0);
			set(HalfCarry, (((get(A)&0xF) + (s&0xF) + get(Carry)) >= 0x10));
			set(A, (get(A) + s + get(Carry)));
			set(Carry, i);
			set(Zero, !get(A));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x8F: if (debug) std::cout << std::endl << "ADC A, A" << std::endl;
            i = ((get(A) + get(A) + get(Carry)) >= 0x100);
			set(Subtract, 0);
			set(HalfCarry, (((get(A)&0xF) + (get(A)&0xF) + get(Carry)) >= 0x10));
			set(A, (get(A) + get(A) + get(Carry)));
			set(Carry, i);
			set(Zero, !get(A));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x90: if (debug) std::cout << std::endl << "SUB A, B" << std::endl;
            i = get(A) - get(B);
            set(Zero, !i);
            set(Subtract, 1);
            set(HalfCarry, ((i & 0xF) > (get(A) & 0xF)));
            set(Carry, ((i & 0xFF) > (get(A) & 0xFF)));
            set(A, i);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x91: if (debug) std::cout << std::endl << "SUB A, C" << std::endl;
            i = get(A) - get(C);
            set(Zero, !i);
            set(Subtract, 1);
            set(HalfCarry, ((i & 0xF) > (get(A) & 0xF)));
            set(Carry, ((i & 0xFF) > (get(A) & 0xFF)));
            set(A, i);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x92: if (debug) std::cout << std::endl << "SUB A, D" << std::endl;
            i = get(A) - get(D);
            set(Zero, !i);
            set(Subtract, 1);
            set(HalfCarry, ((i & 0xF) > (get(A) & 0xF)));
            set(Carry, ((i & 0xFF) > (get(A) & 0xFF)));
            set(A, i);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x93: if (debug) std::cout << std::endl << "SUB A, E" << std::endl;
            i = get(A) - get(E);
            set(Zero, !i);
            set(Subtract, 1);
            set(HalfCarry, ((i & 0xF) > (get(A) & 0xF)));
            set(Carry, ((i & 0xFF) > (get(A) & 0xFF)));
            set(A, i);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x94: if (debug) std::cout << std::endl << "SUB A, H" << std::endl;
            i = get(A) - get(H);
            set(Zero, !i);
            set(Subtract, 1);
            set(HalfCarry, ((i & 0xF) > (get(A) & 0xF)));
            set(Carry, ((i & 0xFF) > (get(A) & 0xFF)));
            set(A, i);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x95: if (debug) std::cout << std::endl << "SUB A, L" << std::endl;
            i = get(A) - get(L);
            set(Zero, !i);
            set(Subtract, 1);
            set(HalfCarry, ((i & 0xF) > (get(A) & 0xF)));
            set(Carry, ((i & 0xFF) > (get(A) & 0xFF)));
            set(A, i);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x96: if (debug) std::cout << std::endl << "SUB A, (HL)" << std::endl;
            i = get(A) - ram->readByte(get(HL));
            set(Zero, !i);
            set(Subtract, 1);
            set(HalfCarry, ((i & 0xF) > (get(A) & 0xF)));
            set(Carry, ((i & 0xFF) > (get(A) & 0xFF)));
            set(A, i);
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x97: if (debug) std::cout << std::endl << "SUB A, A" << std::endl;
            i = get(A) - get(A);
            set(Zero, !i);
            set(Subtract, 1);
            set(HalfCarry, ((i & 0xF) > (get(A) & 0xF)));
            set(Carry, ((i & 0xFF) > (get(A) & 0xFF)));
            set(A, i);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x98: if (debug) std::cout << std::endl << "SBC A, B" << std::endl;
            i = get(Carry) + get(B);
            set(HalfCarry, (((get(A)&0xF) - (get(B)&0xF) - get(Carry)) < 0));
			set(Carry, ((get(A) - get(B) - get(Carry)) < 0));
			set(Subtract, 1);
            decrement(A, i);
            set(Zero, !get(A));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x99: if (debug) std::cout << std::endl << "SBC A, C" << std::endl;
            i = get(Carry) + get(C);
            set(HalfCarry, (((get(A)&0xF) - (get(C)&0xF) - get(Carry)) < 0));
			set(Carry, ((get(A) - get(C) - get(Carry)) < 0));
			set(Subtract, 1);
            decrement(A, i);
            set(Zero, !get(A));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x9A: if (debug) std::cout << std::endl << "SBC A, D" << std::endl;
            i = get(Carry) + get(D);
            set(HalfCarry, (((get(A)&0xF) - (get(D)&0xF) - get(Carry)) < 0));
			set(Carry, ((get(A) - get(D) - get(Carry)) < 0));
			set(Subtract, 1);
            decrement(A, i);
            set(Zero, !get(A));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x9B: if (debug) std::cout << std::endl << "SBC A, E" << std::endl;
            i = get(Carry) + get(E);
            set(HalfCarry, (((get(A)&0xF) - (get(E)&0xF) - get(Carry)) < 0));
			set(Carry, ((get(A) - get(E) - get(Carry)) < 0));
			set(Subtract, 1);
            decrement(A, i);
            set(Zero, !get(A));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x9C: if (debug) std::cout << std::endl << "SBC A, H" << std::endl;
            i = get(Carry) + get(H);
            set(HalfCarry, (((get(A)&0xF) - (get(H)&0xF) - get(Carry)) < 0));
			set(Carry, ((get(A) - get(H) - get(Carry)) < 0));
			set(Subtract, 1);
            decrement(A, i);
            set(Zero, !get(A));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x9D: if (debug) std::cout << std::endl << "SBC A, L" << std::endl;
            i = get(Carry) + get(L);
            set(HalfCarry, (((get(A)&0xF) - (get(L)&0xF) - get(Carry)) < 0));
			set(Carry, ((get(A) - get(L) - get(Carry)) < 0));
			set(Subtract, 1);
            decrement(A, i);
            set(Zero, !get(A));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0x9E: if (debug) std::cout << std::endl << "SBC A, (HL)" << std::endl;
            s = ram->readByte(get(HL));
            i = get(Carry) + s;
            set(HalfCarry, (((get(A)&0xF) - (s&0xF) - get(Carry)) < 0));
			set(Carry, ((get(A) - s - get(Carry)) < 0));
			set(Subtract, 1);
            decrement(A, i);
            set(Zero, !get(A));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0x9F: if (debug) std::cout << std::endl << "SBC A, A" << std::endl;
            i = get(Carry) + get(A);
            set(HalfCarry, (((get(A)&0xF) - (get(A)&0xF) - get(Carry)) < 0));
			set(Carry, ((get(A) - get(A) - get(Carry)) < 0));
			set(Subtract, 1);
            decrement(A, i);
            set(Zero, !get(A));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xA0: if (debug) std::cout << std::endl << "AND A, B" << std::endl;
            set(A, get(A) & get(B));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 1);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xA1: if (debug) std::cout << std::endl << "AND A, C" << std::endl;
            set(A, get(A) & get(C));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 1);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xA2: if (debug) std::cout << std::endl << "AND A, D" << std::endl;
            set(A, get(A) & get(D));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 1);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xA3: if (debug) std::cout << std::endl << "AND A, E" << std::endl;
            set(A, get(A) & get(E));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 1);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xA4: if (debug) std::cout << std::endl << "AND A, H" << std::endl;
            set(A, get(A) & get(H));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 1);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xA5: if (debug) std::cout << std::endl << "AND A, L" << std::endl;
            set(A, get(A) & get(L));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 1);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xA6: if (debug) std::cout << std::endl << "AND A, (HL)" << std::endl;
            set(A, get(A) & ram->readByte(get(HL)));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 1);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0xA7: if (debug) std::cout << std::endl << "AND A, A" << std::endl;
            set(A, get(A) & get(A));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 1);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xA8: if (debug) std::cout << std::endl << "XOR A, B" << std::endl;
            set(A, get(A) ^ get(B));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xA9: if (debug) std::cout << std::endl << "XOR A, C" << std::endl;
            set(A, get(A) ^ get(C));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xAA: if (debug) std::cout << std::endl << "XOR A, D" << std::endl;
            set(A, get(A) ^ get(D));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xAB: if (debug) std::cout << std::endl << "XOR A, E" << std::endl;
            set(A, get(A) ^ get(E));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xAC: if (debug) std::cout << std::endl << "XOR A, H" << std::endl;
            set(A, get(A) ^ get(H));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xAD: if (debug) std::cout << std::endl << "XOR A, L" << std::endl;
            set(A, get(A) ^ get(L));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xAE: if (debug) std::cout << std::endl << "XOR A, (HL)" << std::endl;
            set(A, get(A) ^ ram->readByte(get(HL)));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0xAF: if (debug) std::cout << std::endl << "XOR A, A" << std::endl;
            set(A, get(A) ^ get(A));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xB0: if (debug) std::cout << std::endl << "OR A, B" << std::endl;
            set(A, get(A) | get(B));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xB1: if (debug) std::cout << std::endl << "OR A, C" << std::endl;
            set(A, get(A) | get(C));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xB2: if (debug) std::cout << std::endl << "OR A, D" << std::endl;
            set(A, get(A) | get(D));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xB3: if (debug) std::cout << std::endl << "OR A, E" << std::endl;
            set(A, get(A) | get(E));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xB4: if (debug) std::cout << std::endl << "OR A, H" << std::endl;
            set(A, get(A) | get(H));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xB5: if (debug) std::cout << std::endl << "OR A, L" << std::endl;
            set(A, get(A) | get(L));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xB6: if (debug) std::cout << std::endl << "OR A, (HL)" << std::endl;
            set(A, get(A) | ram->readByte(get(HL)));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0xB7: if (debug) std::cout << std::endl << "OR A, A" << std::endl;
            set(A, get(A) | get(A));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xB8: if (debug) std::cout << std::endl << "CP B" << std::endl;
            set(Zero, (get(A) == get(B)));
            set(Subtract, 1);
            set(HalfCarry, (((get(A) - get(B)) & 0xF) > (get(A) & 0xF)));
            set(Carry, (get(A) < get(B)));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xB9: if (debug) std::cout << std::endl << "CP C" << std::endl;
            set(Zero, (get(A) == get(C)));
            set(Subtract, 1);
            set(HalfCarry, (((get(A) - get(C)) & 0xF) > (get(A) & 0xF)));
            set(Carry, (get(A) < get(C)));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xBA: if (debug) std::cout << std::endl << "CP D" << std::endl;
            set(Zero, (get(A) == get(D)));
            set(Subtract, 1);
            set(HalfCarry, (((get(A) - get(D)) & 0xF) > (get(A) & 0xF)));
            set(Carry, (get(A) < get(D)));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xBB: if (debug) std::cout << std::endl << "CP E" << std::endl;
            set(Zero, (get(A) == get(E)));
            set(Subtract, 1);
            set(HalfCarry, (((get(A) - get(E)) & 0xF) > (get(A) & 0xF)));
            set(Carry, (get(A) < get(E)));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xBC: if (debug) std::cout << std::endl << "CP H" << std::endl;
            set(Zero, (get(A) == get(H)));
            set(Subtract, 1);
            set(HalfCarry, (((get(A) - get(H)) & 0xF) > (get(A) & 0xF)));
            set(Carry, (get(A) < get(H)));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xBD: if (debug) std::cout << std::endl << "CP L" << std::endl;
            set(Zero, (get(A) == get(L)));
            set(Subtract, 1);
            set(HalfCarry, (((get(A) - get(L)) & 0xF) > (get(A) & 0xF)));
            set(Carry, (get(A) < get(L)));
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xBE: if (debug) std::cout << std::endl << "CP (HL)" << std::endl;
            set(Zero, (get(A) == ram->readByte(get(HL))));
            set(Subtract, 1);
            set(HalfCarry, (((get(A) - ram->readByte(get(HL))) & 0xF) > (get(A) & 0xF)));
            set(Carry, (get(A) < ram->readByte(get(HL))));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0xBF: if (debug) std::cout << std::endl << "CP" << std::endl;
            set(Zero, 1);
            set(Subtract, 1);
            set(HalfCarry, 0);
            set(Carry, 0);
            increment(PC, 1);
            *cycles += 1;
            break;
        case 0xC0: if (debug) std::cout << std::endl << "RET NZ" << std::endl;
            if (get(Zero) == 0) {
                set(PC, ram->readWord(get(SP)));
                increment(SP, 2);
                *cycles += 5;
            } else {
                increment(PC, 1);
                *cycles += 2;
            }
            break;
        case 0xC1: if (debug) std::cout << std::endl << "POP BC" << std::endl;
            t = ram->readWord(get(SP));
            set(BC, t);
            increment(SP, 2);
            increment(PC, 1);
            *cycles += 3;
            break;
        case 0xC2: if (debug) std::cout << std::endl << "JP NZ, 0x" << (int) ram->readByte(pc+2) << (int) ram->readByte(pc+1) <<"" << std::endl;
            if (get(Zero) == 0) {
                set(PC, ram->readWord(pc + 1));
                *cycles += 4;
            } else {
                increment(PC, 3);
                *cycles += 3;
            }
            break;
        case 0xC3: if (debug) std::cout << std::endl << "JP " << (int) ram->readByte(pc+2) << (int) ram->readByte(pc+1) <<"" << std::endl;
            set(PC, ram->readWord(pc + 1));
            *cycles += 4;
            break;
        case 0xC4: if (debug) std::cout << std::endl << "CALL NZ, 0x" << (int) ram->readByte(pc+2) << (int) ram->readByte(pc+1) <<"" << std::endl;
            if (get(Zero) == 0) {
                decrement(SP, 2);
                ram->writeWord(get(SP), get(PC) + 3);
                set(PC, ram->readWord(pc + 1));
                *cycles += 6;
            } else {
                increment(PC, 3);
                *cycles += 3;
            }
            break;
        case 0xC5: if (debug) std::cout << std::endl << "PUSH BC" << std::endl;
            decrement(SP, 2);
            ram->writeWord(get(SP), get(BC));
            increment(PC, 1);
            *cycles += 4;
            break;
        case 0xC6: if (debug) std::cout << std::endl << "ADD A, 0x" << (int) ram->readByte(pc+1) <<"" << std::endl;
            i = get(A) + ram->readByte(pc + 1);
            set(Zero, !i);
            set(Subtract, 0);
            set(HalfCarry, ((i & 0xF) < (get(A) & 0xF)));
            set(Carry, ((i & 0xFF) < (get(A) & 0xFF)));
            set(A, i);
            increment(PC, 2);
            *cycles += 2;
            break;
        case 0xC7: if (debug) std::cout << std::endl << "RST 00" << std::endl;
            decrement(SP, 2);
            ram->writeWord(get(SP), get(PC) + 1);
            set(PC, 0x00);
            *cycles += 4;
            break;
        case 0xC8: if (debug) std::cout << std::endl << "RET Z" << std::endl;
            if (get(Zero) == 1) {
                set(PC, ram->readWord(get(SP)));
                increment(SP, 2);
                *cycles += 5;
            } else {
                increment(PC, 1);
                *cycles += 2;
            }
            break;
        case 0xC9: if (debug) std::cout << std::endl << "RET" << std::endl;
            set(PC, ram->readWord(get(SP)));
            increment(SP, 2);
            *cycles += 4;
            break;
        case 0xCA: if (debug) std::cout << std::endl << "JP Z, 0x" << (int) ram->readByte(pc+2) << (int) ram->readByte(pc+1) <<"" << std::endl;
            if (get(Zero) == 1) {
                set(PC, ram->readWord(pc + 1));
                *cycles += 4;
            } else {
                increment(PC, 3);
                *cycles += 3;
            }
            break;
        case 0xCB: if (debug) std::cout << std::endl << "Prefix" << std::endl;
            cbPrefix(ram->readByte(pc + 1));
            increment(PC, 2);
            *cycles += 2;
            break;
        case 0xCC: if (debug) std::cout << std::endl << "CALL Z, 0x" << (int) ram->readByte(pc+2) << (int) ram->readByte(pc+1) <<"" << std::endl;
            if (get(Zero) == 1) {
                decrement(SP, 2);
                ram->writeWord(get(SP), get(PC) + 3);
                set(PC, ram->readWord(pc + 1));
                *cycles += 6;
            } else {
                increment(PC, 3);
                *cycles += 3;
            }
            break;
        case 0xCD: if (debug) std::cout << std::endl << "CALL 0x" << (int) ram->readByte(pc + 1) << std::endl;
            decrement(SP, 2);
            ram->writeWord(get(SP), get(PC) + 3);
            set(PC, ram->readWord(pc + 1));
            *cycles += 6;
            break;
        case 0xCE: if (debug) std::cout << std::endl << "ADC A, 0x" << std::endl;
            s = ram->readByte(pc + 1);
            i = get(A) + s + get(Carry) >= 0x100;
            set(Subtract, 0);
            set(HalfCarry, (((get(A) + s + get(Carry)) & 0xF) < (get(A) & 0xF)));
            set(A, get(A) + s + get(Carry));
            set(Carry, i);
            set(Zero, !get(A));
            increment(PC, 2);
            *cycles += 2;
            break;
        case 0xCF: if (debug) std::cout << std::endl << "RST 08" << std::endl;
            decrement(SP, 2);
            ram->writeWord(get(SP), get(PC) + 1);
            set(PC, 0x08);
            *cycles += 4;
            break;
        case 0xD0: if (debug) std::cout << std::endl << "RET NC" << std::endl;
            if (get(Carry) == 0) {
                set(PC, ram->readWord(get(SP)));
                increment(SP, 2);
                *cycles += 5;
            } else {
                increment(PC, 1);
                *cycles += 2;
            }
            break;
        case 0xD1: if (debug) std::cout << std::endl << "POP DE" << std::endl;
            set(DE, ram->readWord(get(SP)));
            increment(SP, 2);
            increment(PC, 1);
            *cycles += 3;
            break;
        case 0xD2: if (debug) std::cout << std::endl << "JP NC, 0x" << (int) ram->readByte(pc+2) << (int) ram->readByte(pc+1) <<"" << std::endl;
            if (get(Carry) == 0) {
                set(PC, ram->readWord(pc + 1));
                *cycles += 4;
            } else {
                increment(PC, 3);
                *cycles += 3;
            }
            break;
        case 0xD4: if (debug) std::cout << std::endl << "CALL NC, 0x" << (int) ram->readByte(pc+2) << (int) ram->readByte(pc+1) <<"" << std::endl;
            if (get(Carry) == 0) {
                decrement(SP, 2);
                ram->writeWord(get(SP), get(PC) + 3);
                set(PC, ram->readWord(pc + 1));
                *cycles += 6;
            } else {
                increment(PC, 3);
                *cycles += 3;
            }
            break;
        case 0xD5: if (debug) std::cout << std::endl << "PUSH DE" << std::endl;
            decrement(SP, 2);
            ram->writeWord(get(SP), get(DE));
            increment(PC, 1);
            *cycles += 3;
            break;
        case 0xD6: if (debug) std::cout << std::endl << "SUB A, 0x" << (int) ram->readByte(pc+1) <<"" << std::endl;
            i = get(A) - ram->readByte(pc + 1);
            set(Zero, !i);
            set(Subtract, 1);
            set(HalfCarry, ((i & 0xF) > (get(A) & 0xF)));
            set(Carry, ((i & 0xFF) > (get(A) & 0xFF)));
            set(A, i);
            increment(PC, 2);
            *cycles += 4;
            break;
        case 0xD7: if (debug) std::cout << std::endl << "RST 10" << std::endl;
            decrement(SP, 2);
            ram->writeWord(get(SP), get(PC) + 1);
            set(PC, 0x10);
            *cycles += 4;
            break;
        case 0xD8: if (debug) std::cout << std::endl << "RET C" << std::endl;
            if (get(Carry) == 1) {
                set(PC, ram->readWord(get(SP)));
                increment(SP, 2);
                *cycles += 5;
            } else {
                increment(PC, 1);
                *cycles += 2;
            }
            break;
        case 0xD9: if (debug) std::cout << std::endl << "RET NZ" << std::endl;
            set(PC, ram->readWord(get(SP)));
            increment(SP, 2);
            *cycles += 4;
            interrupts->set(Master, 1);
            interrupts->set(Pending, 1);
            break;
        case 0xDA: if (debug) std::cout << std::endl << "JP C, 0x" << (int) ram->readByte(pc+2) << (int) ram->readByte(pc+1) <<"" << std::endl;
            if (get(Carry) == 1) {
                set(PC, ram->readWord(pc + 1));
                *cycles += 4;
            } else {
                increment(PC, 3);
                *cycles += 3;
            }
            break;
        case 0xDC: if (debug) std::cout << std::endl << "CALL C, 0x" << (int) ram->readByte(pc+2) << (int) ram->readByte(pc+1) <<"" << std::endl;
            if (get(Carry) == 1) {
                decrement(SP, 2);
                ram->writeWord(get(SP), get(PC) + 3);
                set(PC, ram->readWord(pc + 1));
                *cycles += 6;
            } else {
                increment(PC, 3);
                *cycles += 3;
            }
            break;
        case 0xDE: if (debug) std::cout << std::endl << "SBC, " << (int) ram->readByte(pc+2) << (int) ram->readByte(pc+1) <<"" << std::endl;
            i = get(A) - (ram->readByte(pc + 1) + get(Carry));
            set(Zero, !i);
            set(Subtract, 1);
            set(HalfCarry, ((i & 0xF) > (get(A) & 0xF)));
            set(Carry, ((i & 0xFF) > (get(A) & 0xFF)));
            set(A, i);
            increment(PC, 2);
            *cycles += 2;
            break;
        case 0xDF: if (debug) std::cout << std::endl << "RST 18" << std::endl;
            decrement(SP, 2);
            ram->writeWord(get(SP), get(PC) + 1);
            set(PC, 0x0018);
            *cycles += 4;
            break;
        case 0xE0: if (debug) std::cout << std::endl << "LD ($FF00+n), A" << std::endl;
            ram->writeByte((0xFF00 + ram->readByte(pc + 1)), get(A));
            increment(PC, 2);
            *cycles += 3;
            break;
        case 0xE1: if (debug) std::cout << std::endl << "POP HL" << std::endl;
            set(HL, ram->readWord(get(SP)));
            increment(SP, 2);
            increment(PC, 1);
            *cycles += 3;
            break;
        case 0xE2: if (debug) std::cout << std::endl << "LD A, ($FF00+C)" << std::endl;
            ram->writeByte((0xFF00 + get(C)), get(A));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0xE5: if (debug) std::cout << std::endl << "PUSH HL" << std::endl;
            decrement(SP, 2);
            ram->writeWord(get(SP), get(HL));
            increment(PC, 1);
            *cycles += 4;
            break;
        case 0xE6: if (debug) std::cout << std::endl << "AND A, 0x" << (int) ram->readByte(pc+1) <<"" << std::endl;
            set(A, get(A) & ram->readByte(pc + 1));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 1);
            set(Carry, 0);
            increment(PC, 2);
            *cycles += 2;
            break;
        case 0xE7: if (debug) std::cout << std::endl << "RST 20" << std::endl;
            decrement(SP, 2);
            ram->writeWord(get(SP), get(PC) + 1);
            set(PC, 0x20);
            *cycles += 4;
            break;
        case 0xE8: if (debug) std::cout << std::endl << "ADD SP, 0x" << (int) ram->readByte(pc+1) <<"" << std::endl;
            t = get(SP);
            increment(SP, (signed char)ram->readByte(pc + 1));
            set(Zero, 0);
            set(Subtract, 0);
            set(HalfCarry, ((get(SP) & 0xF) < (t & 0xF)));
            set(Carry, ((get(SP) & 0xFF) < (t & 0xFF)));
            increment(PC, 2);
            *cycles += 4;
            break;
        case 0xE9: if (debug) std::cout << std::endl << "JP (HL)" << std::endl;
            set(PC, get(HL));
            *cycles += 1;
            break;
        case 0xEA: if (debug) std::cout << std::endl << "LD A, 0x" << (int) ram->readByte(pc+1) <<"" << std::endl;
            ram->writeByte(ram->readWord(pc + 1), get(A));
            increment(PC, 3);
            *cycles += 4;
            break;
        case 0xEE: if (debug) std::cout << std::endl << "XOR A, 0x" << (int) ram->readByte(pc+1) <<"" << std::endl;
            set(A, get(A) ^ ram->readByte(pc + 1));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            increment(PC, 2);
            *cycles += 2;
            break;
        case 0xEF: if (debug) std::cout << std::endl << "RST 28" << std::endl;
            decrement(SP, 2);
            ram->writeWord(get(SP), get(PC) + 1);
            set(PC, 0x28);
            *cycles += 4;
            break;
        case 0xF0: if (debug) std::cout << std::endl << "LD A, ($FF00+n)" << std::endl;
            s = ram->readByte(pc + 1);
            set(A, ram->readByte(0xFF00 + s));
            increment(PC, 2);
            *cycles += 3;
            break;
        case 0xF1: if (debug) std::cout << std::endl << "POP AF" << std::endl;
            set(AF, ram->readWord(get(SP)) & 0xFFF0);
            increment(SP, 2);
            increment(PC, 1);
            *cycles += 3;
            break;
        case 0xF2: if (debug) std::cout << std::endl << "LD A, ($FF00+C)" << std::endl;
            set(A, ram->readByte(get(C) + 0xFF00));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0xF3: if (debug) std::cout << std::endl << "DI" << std::endl;
            increment(PC, 1);
            *cycles += 1;
            interrupts->set(Master, 0);
            break;
        case 0xF5: if (debug) std::cout << std::endl << "PUSH AF" << std::endl;
            decrement(SP, 2);
            ram->writeWord(get(SP), get(AF));
            increment(PC, 1);
            *cycles += 4;
            break;
        case 0xF6: if (debug) std::cout << std::endl << "OR A, 0x" << (int) ram->readByte(pc+1) <<"" << std::endl;
            set(A, get(A) | ram->readByte(pc + 1));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            increment(PC, 2);
            *cycles += 2;
            break;
        case 0xF7: if (debug) std::cout << std::endl << "RST 30" << std::endl;
            decrement(SP, 2);
            ram->writeWord(get(SP), get(PC) + 1);
            set(PC, 0x30);
            *cycles += 4;
            break;
        case 0xF8: if (debug) std::cout << std::endl << "LD HL, SP + 0x" << (int) ram->readByte(pc + 1) << std::endl;
            s = ram->readByte(pc + 1);
            set(HL, get(SP) + (signed char)s);
            set(Subtract, 0);
            set(Zero, 0);
            set(Carry, (((get(SP) + s)&0xFF) < (get(SP)&0xFF))); // a carry will cause a wrap around = making new value smaller
            set(HalfCarry, (((get(SP) + s)&0x0F) < (get(SP)&0x0F))); // add the two, see if it becomes larger
            increment(PC, 2);
            *cycles += 3;
            break;
        case 0xF9: if (debug) std::cout << std::endl << "LD SP, HL" << std::endl;
            set(SP, get(HL));
            increment(PC, 1);
            *cycles += 2;
            break;
        case 0xFA: if (debug) std::cout << std::endl << "LD A, (" << (int) ram->readByte(pc+2) << (int) ram->readByte(pc+1) <<")" << std::endl;
            t = ram->readWord(pc + 1);
            set(A, ram->readByte(t));
            increment(PC, 3);
            *cycles += 4;
            break;
        case 0xFB: if (debug) std::cout << std::endl << "DI" << std::endl;
            increment(PC, 1);
            *cycles += 1;
            interrupts->set(Master, 1);
            interrupts->set(Pending, 1);
            break;
        case 0xFE: if (debug) std::cout << std::endl << "CP 0x" << (int) ram->readByte(pc + 1) << std::endl;
            s = ram->readByte(pc + 1);
            set(Zero, (get(A) == s));
            set(Subtract, 1);
            set(HalfCarry, (((get(A) - s) & 0xF) > (get(A) & 0xF)));
            set(Carry, (get(A) < s));
            increment(PC, 2);
            *cycles += 2;
            break;
        case 0xFF: if (debug) std::cout << std::endl << "RST 38" << std::endl;
            decrement(SP, 2);
            ram->writeWord(get(SP), get(PC) + 1);
            set(PC, 0x0038);
            *cycles += 4;
            break;
        default:
            printf("Instruction: %02X\n", (int)instruction);
            printf("Undefined instruction.\n");
            increment(PC, 1);
            std::cin.ignore();
            break;
    }
}

void CPU::cbPrefix(uint8_t inst)
{
    unsigned char s;
    unsigned char instruction = inst;

    switch (instruction) {
        case 0x00:    // RLC B
            s = (get(B) >> 7);
            set(B, (get(B) << 1) | s);
            set(Zero, !get(B));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            break;
        case 0x01:    // RLC C
            s = (get(C) >> 7);
            set(C, (get(C) << 1) | s);
            set(Zero, !get(C));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            break;
        case 0x02:    // RLC D
            s = (get(D) >> 7);
            set(D, (get(D) << 1) | s);
            set(Zero, !get(D));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            break;
        case 0x03:    // RLC E
            s = (get(E) >> 7);
            set(E, (get(E) << 1) | s);
            set(Zero, !get(E));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            break;
        case 0x04:    // RLC H
            s = (get(H) >> 7);
            set(H, (get(H) << 1) | s);
            set(Zero, !get(H));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            break;
        case 0x05:    // RLC L
            s = (get(L) >> 7);
            set(L, (get(L) << 1) | s);
            set(Zero, !get(L));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            break;
        case 0x06:    // RLC (HL)
            s = (ram->readByte(get(HL)) >> 7);
            ram->writeByte(get(HL), (((ram->readByte(get(HL)) << 1) | s)));
            set(Zero, !get(HL));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            *cycles += 2;
            break;
        case 0x07:    // RLC A
            s = (get(A) >> 7);
            set(A, (get(A) << 1) | s);
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            break;
        case 0x08:    // RRC B
            s = (get(B) & 0x1);
            set(B, (get(B) >> 1) | (s << 7));
            set(Zero, !get(B));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            break;
        case 0x09:    // RRC C
            s = (get(C) & 0x1);
            set(C, (get(C) >> 1) | (s << 7));
            set(Zero, !get(C));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            break;
        case 0x0A:    // RRC D
            s = (get(D) & 0x1);
            set(D, (get(D) >> 1) | (s << 7));
            set(Zero, !get(D));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            break;
        case 0x0B:    // RRC E
            s = (get(E) & 0x1);
            set(E, (get(E) >> 1) | (s << 7));
            set(Zero, !get(E));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            break;
        case 0x0C:    // RRC H
            s = (get(H) & 0x1);
            set(H, (get(H) >> 1) | (s << 7));
            set(Zero, !get(H));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            break;
        case 0x0D:    // RRC L
            s = (get(L) & 0x1);
            set(L, (get(L) >> 1) | (s << 7));
            set(Zero, !get(L));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            break;
        case 0x0E:    // RRC (HL)
            s = (ram->readByte(get(HL)) & 0x1);
            ram->writeByte(get(HL), ((ram->readByte(get(HL)) << 1) | (s)));
            set(Zero, !get(HL));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            *cycles += 2;
            break;
        case 0x0F:    // RRC A
            s = (get(A) & 0x1);
            set(A, (get(A) >> 1) | (s << 7));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            break;
        case 0x10:    // RL B
            s = get(B);
            set(B, (get(B) << 1) | get(Carry));
            set(Carry, s >> 7);
            set(Zero, !get(B));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x11:    // RL C
            s = get(C);
            set(C, (get(C) << 1) | get(Carry));
            set(Carry, s >> 7);
            set(Zero, !get(C));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x12:    // RL D
            s = get(D);
            set(D, (get(D) << 1) | get(Carry));
            set(Carry, s >> 7);
            set(Zero, !get(D));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x13:    // RL E
            s = get(E);
            set(E, (get(E) << 1) | get(Carry));
            set(Carry, s >> 7);
            set(Zero, !get(E));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x14:    // RL H
            s = get(H);
            set(H, (get(H) << 1) | get(Carry));
            set(Carry, s >> 7);
            set(Zero, !get(H));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x15:    // RL L
            s = get(L);
            set(L, (get(L) << 1) | get(Carry));
            set(Carry, s >> 7);
            set(Zero, !get(L));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x16:    // RL (HL)
            s = ram->readByte(get(HL)) >> 7;
            ram->writeByte(get(HL), ((ram->readByte(get(HL)) << 1) | (get(Carry))));
            set(Carry, (s));
            set(Zero, !get(HL));
            set(Subtract, 0);
            set(HalfCarry, 0);
            *cycles += 2;
            break;
        case 0x17:    // RL A
            s = get(A);
            set(A, (get(A) << 1) | get(Carry));
            set(Carry, (s >> 7));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x18:    // RR B
            s = (get(B) & 0x1);
            set(B, (get(B) >> 1) | (get(Carry) << 7));
            set(Carry, s);
            set(Zero, !get(B));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x19:    // RR C
            s = (get(C) & 0x1);
            set(C, (get(C) >> 1) | (get(Carry) << 7));
            set(Carry, s);
            set(Zero, !get(C));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x1A:    // RR D
            s = (get(D) & 0x1);
            set(D, (get(D) >> 1) | (get(Carry) << 7));
            set(Carry, s);
            set(Zero, !get(D));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x1B:    // RR E
            s = (get(E) & 0x1);
            set(E, (get(E) >> 1) | (get(Carry) << 7));
            set(Carry, s);
            set(Zero, !get(E));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x1C:    // RR H
            s = (get(H) & 0x1);
            set(H, (get(H) >> 1) | (get(Carry) << 7));
            set(Carry, s);
            set(Zero, !get(H));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x1D:    // RR L
            s = (get(L) & 0x1);
            set(L, (get(L) >> 1) | (get(Carry) << 7));
            set(Carry, s);
            set(Zero, !get(L));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x1E:    // RR (HL)
            s = (ram->readByte(get(HL)) & 0x1);
            ram->writeByte(get(HL), ((ram->readByte(get(HL)) >> 1) | (get(Carry) << 7)));
            set(Carry, s);
            set(Zero, !get(HL));
            set(Subtract, 0);
            set(HalfCarry, 0);
            *cycles += 2;
            break;
        case 0x1F:    // RR A
            s = (get(A) & 0x1);
            set(A, (get(A) >> 1) | (get(Carry) << 7));
            set(Carry, s);
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x20:    // SLA B
            s = (get(B) >> 7);
            set(B, (get(B) << 1));
            set(Zero, !get(B));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            break;
        case 0x21:    // SLA C
            s = (get(C) >> 7);
            set(C, (get(C) << 1));
            set(Zero, !get(C));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            break;
        case 0x22:    // SLA D
            s = (get(D) >> 7);
            set(D, (get(D) << 1));
            set(Zero, !get(D));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            break;
        case 0x23:    // SLA E
            s = (get(E) >> 7);
            set(E, (get(E) << 1));
            set(Zero, !get(E));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            break;
        case 0x24:    // SLA H
            s = (get(H) >> 7);
            set(H, (get(H) << 1));
            set(Zero, !get(H));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            break;
        case 0x25:    // SLA L
            s = (get(L) >> 7);
            set(L, (get(L) << 1));
            set(Zero, !get(L));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            break;
        case 0x26:    // SLA HL
            s = (ram->readByte(get(HL)) >> 7);
            ram->writeByte(get(HL), ((ram->readByte(get(HL)) << 1)));
            set(Zero, !get(HL));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            *cycles += 2;
            break;
        case 0x27:    // SLA A
            s = (get(A) >> 7);
            set(A, (get(A) << 1));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, s);
            break;
        case 0x28:    // SRA B
            s = get(B);
            set(B, (get(B) >> 1) | (get(B) & 0x80));
            set(Carry, (s & 1));
            set(Zero, !get(B));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x29:    // SRA C
            s = get(C);
            set(C, (get(C) >> 1) | (get(C) & 0x80));
            set(Carry, (s & 1));
            set(Zero, !get(C));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x2A:    // SRA D
            s = get(D);
            set(D, (get(D) >> 1) | (get(D) & 0x80));
            set(Carry, (s & 1));
            set(Zero, !get(D));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x2B:    // SRA E
            s = get(E);
            set(E, (get(E) >> 1) | (get(E) & 0x80));
            set(Carry, (s & 1));
            set(Zero, !get(E));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x2C:    // SRA H
            s = get(H);
            set(H, (get(H) >> 1) | (get(H) & 0x80));
            set(Carry, (s & 1));
            set(Zero, !get(H));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x2D:    // SRA L
            s = get(L);
            set(L, (get(L) >> 1) | (get(L) & 0x80));
            set(Carry, (s & 1));
            set(Zero, !get(L));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x2E:    // SRA (HL)
            s = ram->readByte(get(HL)) & 1;
            ram->writeByte(get(HL), ((ram->readByte(get(HL)) >> 1) | (s)));
            set(Carry, (s));
            set(Zero, !get(HL));
            set(Subtract, 0);
            set(HalfCarry, 0);
            *cycles += 2;
            break;
        case 0x2F:    // SRA A
            s = get(A);
            set(A, (get(A) >> 1) | (get(A) & 0x80));
            set(Carry, (s & 1));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x30:    // SWAP B
            set(B, (((get(B) & 0x0F) << 4) | ((get(B) & 0xF0) >> 4)));
            set(Zero, !get(B));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            break;
        case 0x31:    // SWAP C
            set(C, (((get(C) & 0x0F) << 4) | ((get(C) & 0xF0) >> 4)));
            set(Zero, !get(C));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            break;
        case 0x32:    // SWAP D
            set(D, (((get(D) & 0x0F) << 4) | ((get(D) & 0xF0) >> 4)));
            set(Zero, !get(D));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            break;
        case 0x33:    // SWAP E
            set(E, (((get(E) & 0x0F) << 4) | ((get(E) & 0xF0) >> 4)));
            set(Zero, !get(E));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            break;
        case 0x34:    // SWAP H
            set(H, (((get(H) & 0x0F) << 4) | ((get(H) & 0xF0) >> 4)));
            set(Zero, !get(H));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            break;
        case 0x35:    // SWAP L
            set(L, (((get(L) & 0x0F) << 4) | ((get(L) & 0xF0) >> 4)));
            set(Zero, !get(L));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            break;
        case 0x36:    // SWAP HL
            ram->writeByte(get(HL), (((get(HL) & 0x0F) << 4) | ((get(HL) & 0xF0) >> 4)));
            set(Zero, !get(HL));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            *cycles += 2;
            break;
        case 0x37:    // SWAP A
            set(A, (((get(A) & 0x0F) << 4) | ((get(A) & 0xF0) >> 4)));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            set(Carry, 0);
            break;
        case 0x38:    // SRL B
            s = get(B) & 1;
            set(B, (get(B) >> 1));
            set(Carry, (s));
            set(Zero, !get(B));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x39:    // SRL C
            s = get(C) & 1;
            set(C, (get(C) >> 1));
            set(Carry, (s));
            set(Zero, !get(C));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x3A:    // SRL D
            s = get(D) & 1;
            set(D, (get(D) >> 1));
            set(Carry, (s));
            set(Zero, !get(D));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x3B:    // SRL E
            s = get(E) & 1;
            set(E, (get(E) >> 1));
            set(Carry, (s));
            set(Zero, !get(E));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x3C:    // SRL H
            s = get(H) & 1;
            set(H, (get(H) >> 1));
            set(Carry, (s));
            set(Zero, !get(H));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x3D:    // SRL L
            s = get(L) & 1;
            set(L, (get(L) >> 1));
            set(Carry, (s));
            set(Zero, !get(L));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x3E:    // SRL (HL)
            s = ram->readByte(get(HL)) & 1;
            ram->writeByte(get(HL), ((ram->readByte(get(HL)) >> 1)));
            set(Carry, (s));
            set(Zero, !get(HL));
            set(Subtract, 0);
            set(HalfCarry, 0);
            *cycles += 2;
            break;
        case 0x3F:    // SRL A
            s = get(A) & 1;
            set(A, (get(A) >> 1));
            set(Carry, (s));
            set(Zero, !get(A));
            set(Subtract, 0);
            set(HalfCarry, 0);
            break;
        case 0x40:    // BIT B 0
            set(Zero, !(get(B) & 0x01));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x41:    // BIT C 0
            set(Zero, !(get(C) & 0x01));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x42:    // BIT D 0
            set(Zero, !(get(D) & 0x01));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x43:    // BIT E 0
            set(Zero, !(get(E) & 0x01));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x44:    // BIT H 0
            set(Zero, !(get(H) & 0x01));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x45:    // BIT L 0
            set(Zero, !(get(L) & 0x01));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x46:    // BIT (HL) 0
            set(Zero, !(ram->readByte(get(HL)) & 0x01));
            set(Subtract, 0);
            set(HalfCarry, 1);
            *cycles += 2;
            break;
        case 0x47:    // BIT A 0
            set(Zero, !(get(A) & 0x01));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x48:    // BIT B 1
            set(Zero, !(get(B) & 0x02));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x49:    // BIT C 1
            set(Zero, !(get(C) & 0x02));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x4A:    // BIT D 1
            set(Zero, !(get(D) & 0x02));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x4B:    // BIT E 1
            set(Zero, !(get(E) & 0x02));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x4C:    // BIT H 1
            set(Zero, !(get(H) & 0x02));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x4D:    // BIT L 1
            set(Zero, !(get(L) & 0x02));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x4E:    // BIT (HL) 1
            set(Zero, !(ram->readByte(get(HL)) & 0x02));
            set(Subtract, 0);
            set(HalfCarry, 1);
            *cycles += 2;
            break;
        case 0x4F:    // BIT A 1
            set(Zero, !(get(A) & 0x02));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x50:    // BIT B 2
            set(Zero, !(get(B) & 0x04));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x51:    // BIT C 2
            set(Zero, !(get(C) & 0x04));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x52:    // BIT D 2
            set(Zero, !(get(D) & 0x04));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x53:    // BIT E 2
            set(Zero, !(get(E) & 0x04));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x54:    // BIT H 2
            set(Zero, !(get(H) & 0x04));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x55:    // BIT L 2
            set(Zero, !(get(L) & 0x04));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x56:    // BIT (HL) 2
            set(Zero, !(ram->readByte(get(HL)) & 0x04));
            set(Subtract, 0);
            set(HalfCarry, 1);
            *cycles += 2;
            break;
        case 0x57:    // BIT A 2
            set(Zero, !(get(A) & 0x04));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x58:    // BIT B 3
            set(Zero, !(get(B) & 0x08));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x59:    // BIT C 3
            set(Zero, !(get(C) & 0x08));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x5A:    // BIT D 3
            set(Zero, !(get(D) & 0x08));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x5B:    // BIT E 3
            set(Zero, !(get(E) & 0x08));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x5C:    // BIT H 3
            set(Zero, !(get(H) & 0x08));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x5D:    // BIT L 3
            set(Zero, !(get(L) & 0x08));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x5E:    // BIT (HL) 3
            set(Zero, !(ram->readByte(get(HL)) & 0x08));
            set(Subtract, 0);
            set(HalfCarry, 1);
            *cycles += 2;
            break;
        case 0x5F:    // BIT A 3
            set(Zero, !(get(A) & 0x08));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x60:    // BIT B 4
            set(Zero, !(get(B) & 0x10));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x61:    // BIT C 4
            set(Zero, !(get(C) & 0x10));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x62:    // BIT D 4
            set(Zero, !(get(D) & 0x10));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x63:    // BIT E 4
            set(Zero, !(get(E) & 0x10));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x64:    // BIT H 4
            set(Zero, !(get(H) & 0x10));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x65:    // BIT L 4
            set(Zero, !(get(L) & 0x10));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x66:    // BIT (HL) 4
            set(Zero, !(ram->readByte(get(HL)) & 0x10));
            set(Subtract, 0);
            set(HalfCarry, 1);
            *cycles += 2;
            break;
        case 0x67:    // BIT A 4
            set(Zero, !(get(A) & 0x10));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x68:    // BIT B 5
            set(Zero, !(get(B) & 0x20));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x69:    // BIT C 5
            set(Zero, !(get(C) & 0x20));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x6A:    // BIT D 5
            set(Zero, !(get(D) & 0x20));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x6B:    // BIT E 5
            set(Zero, !(get(E) & 0x20));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x6C:    // BIT H 5
            set(Zero, !(get(H) & 0x20));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x6D:    // BIT L 5
            set(Zero, !(get(L) & 0x20));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x6E:    // BIT (HL) 5
            set(Zero, !(ram->readByte(get(HL)) & 0x20));
            set(Subtract, 0);
            set(HalfCarry, 1);
            *cycles += 2;
            break;
        case 0x6F:    // BIT A 5
            set(Zero, !(get(A) & 0x20));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x70:    // BIT B 6
            set(Zero, !(get(B) & 0x40));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x71:    // BIT C 6
            set(Zero, !(get(C) & 0x40));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x72:    // BIT D 6
            set(Zero, !(get(D) & 0x40));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x73:    // BIT E 6
            set(Zero, !(get(E) & 0x40));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x74:    // BIT H 6
            set(Zero, !(get(H) & 0x40));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x75:    // BIT L 6
            set(Zero, !(get(L) & 0x40));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x76:    // BIT (HL) 6
            set(Zero, !(ram->readByte(get(HL)) & 0x40));
            set(Subtract, 0);
            set(HalfCarry, 1);
            *cycles += 2;
            break;
        case 0x77:    // BIT A 6
            set(Zero, !(get(A) & 0x40));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x78:    // BIT B 7
            set(Zero, !(get(B) & 0x80));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x79:    // BIT C 7
            set(Zero, !(get(C) & 0x80));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x7A:    // BIT D 7
            set(Zero, !(get(D) & 0x80));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x7B:    // BIT E 7
            set(Zero, !(get(E) & 0x80));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x7C:    // BIT H 7
            set(Zero, !(get(H) & 0x80));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x7D:    // BIT L 7
            set(Zero, !(get(L) & 0x80));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x7E:    // BIT (HL) 7
            set(Zero, !(ram->readByte(get(HL)) & 0x80));
            set(Subtract, 0);
            set(HalfCarry, 1);
            *cycles += 2;
            break;
        case 0x7F:    // BIT A 7
            set(Zero, !(get(A) & 0x80));
            set(Subtract, 0);
            set(HalfCarry, 1);
            break;
        case 0x80:    // RES B 0
            set(B, get(B) & 0xFE);;
            break;
        case 0x81:    // RES C 0
            set(C, get(C) & 0xFE);;
            break;
        case 0x82:    // RES D 0
            set(D, get(D) & 0xFE);;
            break;
        case 0x83:    // RES E 0
            set(E, get(E) & 0xFE);;
            break;
        case 0x84:    // RES H 0
            set(H, get(H) & 0xFE);;
            break;
        case 0x85:    // RES L 0
            set(L, get(L) & 0xFE);;
            break;
        case 0x86:    // RES (HL) 0
            ram->writeByte(get(HL), (ram->readByte(get(HL)) & 0xFE));
            *cycles += 2;
            break;
        case 0x87:    // RES A 0
            set(A, get(A) & 0xFE);;
            break;
        case 0x88:    // RES B 1
            set(B, get(B) & 0xFD);;
            break;
        case 0x89:    // RES C 1
            set(C, get(C) & 0xFD);;
            break;
        case 0x8A:    // RES D 1
            set(D, get(D) & 0xFD);;
            break;
        case 0x8B:    // RES E 1
            set(E, get(E) & 0xFD);;
            break;
        case 0x8C:    // RES H 1
            set(H, get(H) & 0xFD);;
            break;
        case 0x8D:    // RES L 1
            set(L, get(L) & 0xFD);;
            break;
        case 0x8E:    // RES (HL) 1
            ram->writeByte(get(HL), (ram->readByte(get(HL)) & 0xFD));
            *cycles += 2;
            break;
        case 0x8F:    // RES A 1
            set(A, get(A) & 0xFD);;
            break;
        case 0x90:    // RES B 2
            set(B, get(B) & 0xFB);;
            break;
        case 0x91:    // RES C 2
            set(C, get(C) & 0xFB);;
            break;
        case 0x92:    // RES D 2
            set(D, get(D) & 0xFB);;
            break;
        case 0x93:    // RES E 2
            set(E, get(E) & 0xFB);;
            break;
        case 0x94:    // RES H 2
            set(H, get(H) & 0xFB);;
            break;
        case 0x95:    // RES L 2
            set(L, get(L) & 0xFB);;
            break;
        case 0x96:    // RES (HL) 2
            ram->writeByte(get(HL), (ram->readByte(get(HL)) & 0xFB));
            *cycles += 2;
            break;
        case 0x97:    // RES A 2
            set(A, get(A) & 0xFB);;
            break;
        case 0x98:    // RES B 3
            set(B, get(B) & 0xF7);;
            break;
        case 0x99:    // RES C 3
            set(C, get(C) & 0xF7);;
            break;
        case 0x9A:    // RES D 3
            set(D, get(D) & 0xF7);;
            break;
        case 0x9B:    // RES E 3
            set(E, get(E) & 0xF7);;
            break;
        case 0x9C:    // RES H 3
            set(H, get(H) & 0xF7);;
            break;
        case 0x9D:    // RES L 3
            set(L, get(L) & 0xF7);;
            break;
        case 0x9E:    // RES (HL) 3
            ram->writeByte(get(HL), (ram->readByte(get(HL)) & 0xF7));
            *cycles += 2;
            break;
        case 0x9F:    // RES A 3
            set(A, get(A) & 0xF7);;
            break;
        case 0xA0:    // RES B 4
            set(B, get(B) & 0xEF);;
            break;
        case 0xA1:    // RES C 4
            set(C, get(C) & 0xEF);;
            break;
        case 0xA2:    // RES D 4
            set(D, get(D) & 0xEF);;
            break;
        case 0xA3:    // RES E 4
            set(E, get(E) & 0xEF);;
            break;
        case 0xA4:    // RES H 4
            set(H, get(H) & 0xEF);;
            break;
        case 0xA5:    // RES L 4
            set(L, get(L) & 0xEF);;
            break;
        case 0xA6:    // RES (HL) 4
            ram->writeByte(get(HL), (ram->readByte(get(HL)) & 0xEF));
            *cycles += 2;
            break;
        case 0xA7:    // RES A 4
            set(A, get(A) & 0xEF);;
            break;
        case 0xA8:    // RES B 5
            set(B, get(B) & 0xDF);;
            break;
        case 0xA9:    // RES C 5
            set(C, get(C) & 0xDF);;
            break;
        case 0xAA:    // RES D 5
            set(D, get(D) & 0xDF);;
            break;
        case 0xAB:    // RES E 5
            set(E, get(E) & 0xDF);;
            break;
        case 0xAC:    // RES H 5
            set(H, get(H) & 0xDF);;
            break;
        case 0xAD:    // RES L 5
            set(L, get(L) & 0xDF);;
            break;
        case 0xAE:    // RES (HL) 5
            ram->writeByte(get(HL), (ram->readByte(get(HL)) & 0xDF));
            *cycles += 2;
            break;
        case 0xAF:    // RES A 5
            set(A, get(A) & 0xDF);;
            break;
        case 0xB0:    // RES B 6
            set(B, get(B) & 0xBF);;
            break;
        case 0xB1:    // RES C 6
            set(C, get(C) & 0xBF);;
            break;
        case 0xB2:    // RES D 6
            set(D, get(D) & 0xBF);;
            break;
        case 0xB3:    // RES E 6
            set(E, get(E) & 0xBF);;
            break;
        case 0xB4:    // RES H 6
            set(H, get(H) & 0xBF);;
            break;
        case 0xB5:    // RES L 6
            set(L, get(L) & 0xBF);;
            break;
        case 0xB6:    // RES (HL) 6
            ram->writeByte(get(HL), (ram->readByte(get(HL)) & 0xBF));
            *cycles += 2;
            break;
        case 0xB7:    // RES A 6
            set(A, get(A) & 0x7F);;
            break;
        case 0xB8:    // RES B 7
            set(B, get(B) & 0x7F);;
            break;
        case 0xB9:    // RES C 7
            set(C, get(C) & 0x7F);;
            break;
        case 0xBA:    // RES D 7
            set(D, get(D) & 0x7F);;
            break;
        case 0xBB:    // RES E 7
            set(E, get(E) & 0x7F);;
            break;
        case 0xBC:    // RES H 7
            set(H, get(H) & 0x7F);;
            break;
        case 0xBD:    // RES L 7
            set(L, get(L) & 0x7F);;
            break;
        case 0xBE:    // RES (HL) 7
            ram->writeByte(get(HL), (ram->readByte(get(HL)) & 0x7F));
            *cycles += 2;
            break;
        case 0xBF:    // RES A 7
            set(A, get(A) & 0x7F);;
            break;
        case 0xC0:    // SET B 0
            set(B, get(B) | 0x01);;
            break;
        case 0xC1:    // SET C 0
            set(C, get(C) | 0x01);;
            break;
        case 0xC2:    // SET D 0
            set(D, get(D) | 0x01);;
            break;
        case 0xC3:    // SET E 0
            set(E, get(E) | 0x01);;
            break;
        case 0xC4:    // SET H 0
            set(H, get(H) | 0x01);;
            break;
        case 0xC5:    // SET L 0
            set(L, get(L) | 0x01);;
            break;
        case 0xC6:    // SET (HL) 0
            ram->writeByte(get(HL), (ram->readByte(get(HL)) | 0x01));
            *cycles += 2;
            break;
        case 0xC7:    // SET A 0
            set(A, get(A) | 0x01);;
            break;
        case 0xC8:    // SET B 1
            set(B, get(B) | 0x02);;
            break;
        case 0xC9:    // SET C 1
            set(C, get(C) | 0x02);;
            break;
        case 0xCA:    // SET D 1
            set(D, get(D) | 0x02);;
            break;
        case 0xCB:    // SET E 1
            set(E, get(E) | 0x02);;
            break;
        case 0xCC:    // SET H 1
            set(H, get(H) | 0x02);;
            break;
        case 0xCD:    // SET L 1
            set(L, get(L) | 0x02);;
            break;
        case 0xCE:    // SET (HL) 1
            ram->writeByte(get(HL), (ram->readByte(get(HL)) | 0x02));
            *cycles += 2;
            break;
        case 0xCF:    // SET A 1
            set(A, get(A) | 0x02);;
            break;
        case 0xD0:    // SET B 2
            set(B, get(B) | 0x04);;
            break;
        case 0xD1:    // SET C 2
            set(C, get(C) | 0x04);;
            break;
        case 0xD2:    // SET D 2
            set(D, get(D) | 0x04);;
            break;
        case 0xD3:    // SET E 2
            set(E, get(E) | 0x04);;
            break;
        case 0xD4:    // SET H 2
            set(H, get(H) | 0x04);;
            break;
        case 0xD5:    // SET L 2
            set(L, get(L) | 0x04);;
            break;
        case 0xD6:    // SET (HL) 2
            ram->writeByte(get(HL), (ram->readByte(get(HL)) | 0x04));
            *cycles += 2;
            break;
        case 0xD7:    // SET A 2
            set(A, get(A) | 0x04);;
            break;
        case 0xD8:    // SET B 3
            set(B, get(B) | 0x08);;
            break;
        case 0xD9:    // SET C 3
            set(C, get(C) | 0x08);;
            break;
        case 0xDA:    // SET D 3
            set(D, get(D) | 0x08);;
            break;
        case 0xDB:    // SET E 3
            set(E, get(E) | 0x08);;
            break;
        case 0xDC:    // SET H 3
            set(H, get(H) | 0x08);;
            break;
        case 0xDD:    // SET L 3
            set(L, get(L) | 0x08);;
            break;
        case 0xDE:    // SET (HL) 3
            ram->writeByte(get(HL), (ram->readByte(get(HL)) | 0x08));
            *cycles += 2;
            break;
        case 0xDF:    // SET A 3
            set(A, get(A) | 0x08);;
            break;
        case 0xE0:    // SET B 4
            set(B, get(B) | 0x10);;
            break;
        case 0xE1:    // SET C 4
            set(C, get(C) | 0x10);;
            break;
        case 0xE2:    // SET D 4
            set(D, get(D) | 0x10);;
            break;
        case 0xE3:    // SET E 4
            set(E, get(E) | 0x10);;
            break;
        case 0xE4:    // SET H 4
            set(H, get(H) | 0x10);;
            break;
        case 0xE5:    // SET L 4
            set(L, get(L) | 0x10);;
            break;
        case 0xE6:    // SET (HL) 4
            ram->writeByte(get(HL), (ram->readByte(get(HL)) | 0x10));
            *cycles += 2;
            break;
        case 0xE7:    // SET A 4
            set(A, get(A) | 0x10);;
            break;
        case 0xE8:    // SET B 5
            set(B, get(B) | 0x20);;
            break;
        case 0xE9:    // SET C 5
            set(C, get(C) | 0x20);;
            break;
        case 0xEA:    // SET D 5
            set(D, get(D) | 0x20);;
            break;
        case 0xEB:    // SET E 5
            set(E, get(E) | 0x20);;
            break;
        case 0xEC:    // SET H 5
            set(H, get(H) | 0x20);;
            break;
        case 0xED:    // SET L 5
            set(L, get(L) | 0x20);;
            break;
        case 0xEE:    // SET (HL) 5
            ram->writeByte(get(HL), (ram->readByte(get(HL)) | 0x20));
            *cycles += 2;
            break;
        case 0xEF:    // SET A 5
            set(A, get(A) | 0x20);;
            break;
        case 0xF0:    // SET B 6
            set(B, get(B) | 0x40);;
            break;
        case 0xF1:    // SET C 6
            set(C, get(C) | 0x40);;
            break;
        case 0xF2:    // SET D 6
            set(D, get(D) | 0x40);;
            break;
        case 0xF3:    // SET E 6
            set(E, get(E) | 0x40);;
            break;
        case 0xF4:    // SET H 6
            set(H, get(H) | 0x40);;
            break;
        case 0xF5:    // SET L 6
            set(L, get(L) | 0x40);;
            break;
        case 0xF6:    // SET (HL) 6
            ram->writeByte(get(HL), (ram->readByte(get(HL)) | 0x40));
            *cycles += 2;
            break;
        case 0xF7:    // SET A 6
            set(A, get(A) | 0x40);;
            break;
        case 0xF8:    // SET B 7
            set(B, get(B) | 0x80);;
            break;
        case 0xF9:    // SET C 7
            set(C, get(C) | 0x80);;
            break;
        case 0xFA:    // SET D 7
            set(D, get(D) | 0x80);;
            break;
        case 0xFB:    // SET E 7
            set(E, get(E) | 0x80);;
            break;
        case 0xFC:    // SET H 7
            set(H, get(H) | 0x80);;
            break;
        case 0xFD:    // SET L 7
            set(L, get(L) | 0x80);;
            break;
        case 0xFE:    // SET (HL) 7
            ram->writeByte(get(HL), (ram->readByte(get(HL)) | 0x80));
            *cycles += 2;
            break;
        case 0xFF:    // SET A 7
            set(A, get(A) | 0x80);;
            break;
    }
}
