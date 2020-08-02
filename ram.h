#ifndef RAM_H
#define RAM_H

#include <iostream>
#include <array>
#include <vector>
#include <string>
#include <fstream>
#include "rom.h"
#include "timer.h"
#include "interrupts.h"
#include "lcd.h"

class RAM {
    private: 
        std::array<uint8_t, 0x4000> rom00; // Fixed ROM
        std::array<uint8_t, 0x4000> rom01; // Switchable ROM
        std::array<uint8_t, 0x2000> vram; // Video RAM
        std::array<uint8_t, 0x2000> extram; // External RAM
        std::array<uint8_t, 0x1000> wram0; // Work RAM bank 0
        std::array<uint8_t, 0x1000> wram1; // Work RAM bank 1 (switchable)
        std::array<uint8_t, 0x100> oam; // Sprite attribute table (OAM) 
        std::array<uint8_t, 0x100> io; // I/O Registers
        std::array<uint8_t, 0x80> hram; // High RAM
        int dPadButtons;
        int dPadDirections;
        uint8_t mask;

        Timer * timers;
        Interrupts * interrupts;
        LCD * lcd;

    public:
        RAM(Timer * timers, Interrupts * interrupts, LCD * lcd);
        uint8_t readByte(uint16_t addr);
        uint16_t readWord(uint16_t addr);
        void writeByte(uint16_t addr, uint8_t value);
        void writeWord(uint16_t addr, uint16_t value);
        void copyToOAM(uint16_t OAM, uint16_t DMA, unsigned int length);
        void changeROMbank(ROM rom, int bank);
        void loadROM(ROM rom);
        void dump(uint16_t start, uint16_t end, int lineLength = 16);
};

#endif