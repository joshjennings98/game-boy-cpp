#ifndef MEMORY_H
#define MEMORY_H

#include <iostream>

class Memory {
    private: 
        uint8_t rom00[0x4000]; // Fixed ROM
        uint8_t rom01[0x4000]; // Switchable ROM
        uint8_t vram[0x2000]; // Video RAM
        uint8_t extram[0x2000]; // External RAM
        uint8_t wram0[0x1000]; // Work RAM bank 0
        uint8_t wram1[0x1000]; // Work RAM bank 1 (switchable)
        uint8_t oam[0x100]; // Sprite attribute table (OAM) 
        uint8_t io[0x100]; // I/O Registers
        uint8_t hram[0x80]; // High RAM
    public:
        Memory();
        uint8_t readByte(uint16_t addr);
        uint16_t readShort(uint16_t addr);
        void writeByte(uint16_t addr, uint8_t value);
        void writeShort(uint16_t addr, uint16_t value);
};

#endif