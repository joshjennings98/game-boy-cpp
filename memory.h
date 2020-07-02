#ifndef MEMORY_H
#define MEMORY_H

#include <iostream>
#include <array>

class Memory {
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
    public:
        Memory();
        uint8_t readByte(uint16_t addr);
        uint16_t readShort(uint16_t addr);
        void writeByte(uint16_t addr, uint8_t value);
        void writeShort(uint16_t addr, uint16_t value);
};

#endif