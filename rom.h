#ifndef ROM_H
#define ROM_H

#include <iostream>
#include <vector>
#include <string>

enum RomInfo { romType, romSize, ramSize };

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

#endif