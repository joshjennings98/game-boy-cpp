#include "gameboy.h"

GameBoy::GameBoy() 
{
    cycles = 0;

    interrupts = new Interrupts();
    display = new Display();
    lcd = new LCD(interrupts, &cycles, display);
    timers = new Timer(interrupts, &cycles);
    ram = new RAM(timers, interrupts, lcd, display);
    cpu = new CPU(interrupts, timers, lcd, ram, &cycles);

}

void GameBoy::load(std::string filename)
{
    rom.load(filename);
    ram->loadROM(rom);
}

void GameBoy::run()
{
    while (lcd->lcdCycle(SDL_GetTicks(), ram)) {
        cpu->cpuCycle();
        interrupts->doCycle(cpu);
        timers->doCycle();
    }
}