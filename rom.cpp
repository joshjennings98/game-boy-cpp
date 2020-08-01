#include <iostream>
#include "ram.h"
#include "rom.h"

ROM::ROM() {
    // Does nothing might remove later
}

void ROM::load(std::string filename) {

    std::cout << "Trying to load " << filename << std::endl;
    
    try {
        std::array<char, headerSize> header;
        int fSize;
        std::ifstream romFile;

        romFile.open(filename, std::ios::binary);
        romFile.seekg(0, std::ios::end);
        
        fSize = romFile.tellg();
        
        romData.resize(fSize);
        
        romFile.seekg(0, std::ios::beg);
        romFile.read(&romData[0], fSize);

        romFile.seekg(0, std::ios::beg);
        romFile.read(&header[0], headerSize);

        set(romType, header[romTypeOffset]);

        for (int i = 0; i<16; i++) {
            gameTitle[i] = header[romTitleOffset + i];
        }

        set(romSize, 16 * (2 << (header[romSizeOffset] + 1))); // pow(2,header[ROM_SIZE_OFFSET]+1) * 16;
        set(ramSize, 0.5 * (2 << (2 * header[romRamOffset]))); // rom.ramSize = pow(4, header[ROM_RAM_OFFSET])/2;

        romFile.close();
    }
    catch (const std::bad_alloc& e) {
        std::cout << "Invalid rom file." << std::endl;
        exit(0);
    }

    std::cout << "Successfully loaded " << filename << std::endl;
}

char ROM::get(RomInfo info) {
    switch (info) {
        case(romType):
            return typeROM;
        case(romSize):
            return sizeROM;
        case(ramSize):
            return sizeRAM;
    }
}

void ROM::set(RomInfo info, char value) {
    switch (info) {
        case(romType):
            typeROM = value;
        case(romSize):
            sizeROM = value;
        case(ramSize):
            sizeRAM = value;
    }
}

std::vector<uint8_t> ROM::getData(int start, int length) {
    return std::vector<uint8_t>(romData.begin() + start, romData.begin() + start + length);
}