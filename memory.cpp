#include <iostream>
#include <array>
#include "memory.h"

Memory::Memory()
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
}

uint8_t Memory::readByte(uint16_t addr)
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
        return 0;
    } else if (0xFF00 <= addr && addr < 0xFF80) {
        // Sort out weird stuff here
        return io[addr - 0xFF00];
    } else if (0xFF80 <= addr && addr < 0xFFFF) {
        return hram[addr - 0xFF80];
    } else if (0xFFFF == addr) {
        // return interrupt register (make in registers)
    }
    return 0x0; // base case
}

uint16_t Memory::readShort(uint16_t addr) {
    return readByte(addr) | readByte(addr + 1) << 8;
}

void Memory::writeByte(uint16_t addr, uint8_t value)
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
    } else if (0xFF00 <= addr && addr < 0xFF80) {
        io[addr - 0xFF00] = value;
    } else if (0xFF80 <= addr && addr < 0xFFFF) {
        hram[addr - 0xFF80] = value;
    } else if (0xFFFF == addr) {
        // enable interrupt register (make in registers)
    }
}

void Memory::writeShort(uint16_t addr, uint16_t value) {
    writeByte(addr, value & 0x00FF);
    writeByte(addr + 1, (value & 0xFF00) >> 8);
}

void Memory::copyToOAM(uint16_t OAM, uint16_t DMA, unsigned int length) {
	for(unsigned int i = 0; i < length; i++) {
         writeByte(OAM + i, readByte(DMA + i));
    }
}