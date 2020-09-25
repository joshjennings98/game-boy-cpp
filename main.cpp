#include <iostream>
#include <chrono>
#include <thread>
#include "SDL2/SDL.h"

#include "ram.cpp"
#include "rom.cpp"
#include "interrupts.cpp"
#include "timer.cpp"
#include "lcd.cpp"
#include "sdl.cpp"
#include "cpu.cpp"
#include "gameboy.cpp"

int main(int argc, char *argv[]) {

    if(argc != 2) {
		std::cout << "Error, must provide a rom. (" << argv[0] << " <rom>)" << std::endl;
		return 0;
	}

    GameBoy gameboy;

    gameboy.load(argv[1]);

    gameboy.run();

    return 0;
}
