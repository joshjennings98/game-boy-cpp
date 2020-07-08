#include <iostream>
#include <array>
#include <vector>
#include <string>
#include <fstream>
#include "ram.h"
#include "rom.h"

RAM::RAM()
{
    // Make sure initisalised to zero
    rom00.fill(0x0);
    rom01.fill(0x0);
    vram.fill(0x0);
    extram.fill(0x0);
    wram0.fill(0x0);
    wram1.fill(0x0);
    oam.fill(0x0);
    io.fill(0x0);
    hram.fill(0x0);

    // Power up RAM initialisation
    writeByte(0xFF10, 0x80);
    writeByte(0xFF11, 0xBF);
    writeByte(0xFF12, 0xF3);
    writeByte(0xFF14, 0xBF);
    writeByte(0xFF16, 0x3F);
    writeByte(0xFF19, 0xBF);
    writeByte(0xFF1A, 0x7F);
    writeByte(0xFF1B, 0xFF);
    writeByte(0xFF1C, 0x9F);
    writeByte(0xFF1E, 0xBF);
    writeByte(0xFF20, 0xFF);
    writeByte(0xFF23, 0xBF);
    writeByte(0xFF24, 0x77);
    writeByte(0xFF25, 0xF3);
    writeByte(0xFF26, 0xF1);
    writeByte(0xFF40, 0x91);
    writeByte(0xFF47, 0xFC);
    writeByte(0xFF48, 0xFF);
    writeByte(0xFF49, 0xFF);
}

uint8_t RAM::readByte(uint16_t addr)
{
    if (0x0000 <= addr && addr < 0x4000) {
        return rom00[addr];
    } else if (0x4000 <= addr && addr < 0x8000) {
        return rom01[addr - 0x4000];
    } else if (0x8000 <= addr && addr < 0xA000) {
        return vram[addr - 0x8000];
    } else if (0xA000 <= addr && addr < 0xC000) {
        return extram[addr - 0xA000];
    } else if (0xC000 <= addr && addr < 0xD000) {
        return wram0[addr - 0xC000];
    } else if (0xD000 <= addr && addr < 0xE000) {
        return wram1[addr - 0xD000];
    } else if (0xE000 <= addr && addr < 0xFE00) {
        // mirror of C000 - DDFF
        return readByte(addr - 0x2000); 
    } else if (0xFE00 <= addr && addr < 0xFEA0) {
        return oam[addr - 0xFE00];
    } else if (0xFEA0 <= addr && addr < 0xFF00) {
        // 0xFEA0 - 0xFF00 is unusable
        return 0x0;
    } else if (addr == 0xFF00) {
        mask = 0;
        if (!dPadButtons) {
            // set mask to getButton
        }
        if (!dPadDirections) {
            // set mask to getDirection
        }
        // return 0xC0 | (0xF ^ mask) | (dPadButtons | dPadDirections);
    } else if (addr == 0xFF04) {
        // return Div
    } else if (addr == 0xFF05) {
        // return Tima
    } else if (addr == 0xFF06) {
        // return Tma
    } else if (addr == 0xFF04) {
        // return Tac
    } else if (addr == 0xFF0F) {
        // return interrupt flags
    } else if (addr == 0xFF40) {
        // return LCDC
    } else if (addr == 0xFF41) {
        // return LCDS
    } else if (addr == 0xFF42) {
        // return scroll y
    } else if (addr == 0xFF43) {
        // return scroll x
    } else if (addr == 0xFF44) {
        // return get line
    } else if (0xFF00 <= addr && addr < 0xFF80) {
        return io[addr - 0xFF00];
    } else if (0xFF80 <= addr && addr < 0xFFFF) {
        return hram[addr - 0xFF80];
    } else if (0xFFFF == addr) {
        // return interrupt register (make in registers)
    }
    return 0x0; // base case
}

uint16_t RAM::readWord(uint16_t addr) {
    return readByte(addr) | readByte(addr + 1) << 8;
}

void RAM::writeByte(uint16_t addr, uint8_t value)
{
    if (0x0000 <= addr && addr < 0x4000) {
        rom00[addr] = value;
    } else if (0x4000 <= addr && addr < 0x8000) {
        rom01[addr - 0x4000] = value;
    } else if (0x8000 <= addr && addr < 0xA000) {
        vram[addr - 0x8000] = value;
    } else if (0xA000 <= addr && addr < 0xC000) {
        extram[addr - 0xA000] = value;
    } else if (0xC000 <= addr && addr < 0xD000) {
        wram0[addr - 0xC000] = value;
    } else if (0xD000 <= addr && addr < 0xE000) {
        wram1[addr - 0xD000] = value;
    } else if (0xE000 <= addr && addr < 0xFE00) {
        // mirror of C000 - DDFF
        writeByte(addr - 0x2000, value); 
    } else if (0xFE00 <= addr && addr < 0xFEA0) {
        oam[addr - 0xFE00] = value;
    } else if (0xFEA0 <= addr && addr < 0xFF00) {
        // 0xFEA0 - 0xFF00 is unusable
    } else if (addr == 0xFF04) { 
        // set Div to value
    } else if (addr == 0xFF05) { 
        // set Tima to value
    } else if (addr == 0xFF06) { 
        // set Tma to value
    } else if (addr == 0xFF04) { 
        // set Tac to value
    } else if (addr == 0xFF40) { 
        // set LCDC to value
    } else if (addr == 0xFF41) { 
        // set LCDS to value
    } else if (addr == 0xFF42) { 
        // set Scroll Y to value
    } else if (addr == 0xFF43) { 
        // set Scroll X to value
    } else if (addr == 0xFF45) { 
        // set Ly Compare to value
    } else if(addr == 0xFF46) { 
        // Direct memory access for OAM
        copyToOAM(0xfe00, value << 8, 160);
    } else if (addr == 0xFF47) { 
        // set BG Palette to value
    } else if (addr == 0xFF48) { 
        // set Sprite Palette 1 to value
    } else if (addr == 0xFF49) { 
        // set Sprite Palette 2 to value
    } else if (addr == 0xFF4A) { 
        // set Window Y to value
    } else if (addr == 0xFF4B) {
        // set Window X to value
    } else if (addr == 0xFF00) {
        dPadDirections = value & 0x10;
        dPadButtons = value & 0x20;
    } else if (addr == 0xFF0F) {
        // set interrupt flags to value
    } else if (0xFF00 <= addr && addr < 0xFF80) {
        io[addr - 0xFF00] = value;
    } else if (0xFF80 <= addr && addr < 0xFFFF) {
        hram[addr - 0xFF80] = value;
    } else if (0xFFFF == addr) {
        // enable interrupt register (make in registers)
    }
}

void RAM::writeWord(uint16_t addr, uint16_t value) {
    writeByte(addr, value & 0x00FF);
    writeByte(addr + 1, (value & 0xFF00) >> 8);
}

void RAM::copyToOAM(uint16_t OAM, uint16_t DMA, unsigned int length) {
	for (unsigned int i = 0; i < length; i++) {
         writeByte(OAM + i, readByte(DMA + i));
    }
}

void RAM::changeROMbank(ROM rom, int bankNum) {
    std::vector<uint8_t> romData = rom.getData(bankNum * 0x4000, 0x4000);
    std::cout << romData.size() << std::endl;
    // std::copy(romData.begin(), romData.end(), rom01);
    for (int i = 0; i < rom01.size(); i++)
        rom01[i] = romData[i];
}

void RAM::loadROM(ROM rom) {
    std::vector<uint8_t> romData = rom.getData(0x0000, 0x8000);
    // std::copy(romData.begin(), romData.begin() + 0x4000, rom00);
    // std::copy(romData.begin() + 0x4000, romData.end(), rom01);
    for (int i = 0; i < 0x4000; i++) {
        rom00[i] = romData[i];
        rom01[i] = romData[i + 0x4000];
    }           
}

void RAM::dump(uint16_t start, uint16_t end, int lineLength) {
    int dataByte;

    for (int j = start + 1; j <= end; j++) {
        dataByte = (int)(readByte(j - 1));
        std::cout << "0x" << std::hex << dataByte << " ";
        if (dataByte < 0x10)
            std::cout << " ";
        if (j % lineLength == 0 && j != 0)
            std::cout << std::endl;
    }

    std::cout << std::endl;
}
