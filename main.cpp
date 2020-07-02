#include <iostream>
#include "ram.cpp"
#include "rom.cpp"

int main() {
    RAM ram;
    ROM rom("tetris.gb");
    ram.loadROM(rom);

    // test crap
    ram.dump(0x0000, 0x0020);

    return 0;
}