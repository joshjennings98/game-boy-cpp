#include <iostream>
#include "memory.cpp"

int main() {
    RAM ram;
    ROM rom("tetris.gb");
    ram.loadROM(rom);

    // test crap
    ram.dump(0x0000, 0x0020);

    return 0;
}