#ifndef MEMORY_H
#define MEMORY_H

#include <iostream>
#include <array>
#include <vector>
#include <string>
#include <fstream>

enum RomInfo { romType, romSize, ramSize };
enum RamType { rom00, rom01, vram, extram, wram0, wram1, oam, io, hram };

class ROM {
    private:
        std::vector<char> romData;
        char gameTitle[17];
        char typeROM;
        char sizeROM;
        char sizeRAM;
    public:
        static const int romTitleOffset = 0x134;
        static const int romTypeOffset = 0x147;
        static const int romSizeOffset = 0x148;
        static const int romRamOffset = 0x149;
        static const int headerSize = 0x14F;
        
        ROM(std::string filename);
        char get(RomInfo info);
        void set(RomInfo info, char value);
        std::vector<uint8_t> getData(int start = 0, int length = 0);
};

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
    public:
        RAM();
        uint8_t readByte(uint16_t addr);
        uint16_t readShort(uint16_t addr);
        void writeByte(uint16_t addr, uint8_t value);
        void writeShort(uint16_t addr, uint16_t value);
        void copyToOAM(uint16_t OAM, uint16_t DMA, unsigned int length);
        void changeROMbank(ROM rom, int bank);
        void loadROM(ROM rom);
        void dump(RamType ram, int start = 0, int end = -1, int lineLength = 16);
        void dump(uint16_t start, uint16_t end, int lineLength = 16);
};

#endif