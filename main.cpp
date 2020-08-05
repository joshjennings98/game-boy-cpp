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

int main() {

    GameBoy gameboy(NoDebug);

    gameboy.load("tetris.gb");

    gameboy.run();

    return 0;
}