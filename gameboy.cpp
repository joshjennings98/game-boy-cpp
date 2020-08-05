#include "gameboy.h"

GameBoy::GameBoy(DebugSetting debug) 
{
    cycles = 0;
    this->debug = (debug == Debug);

    interrupts = new Interrupts();
    display = new Display();
    lcd = new LCD(interrupts, &cycles, display);
    timers = new Timer(interrupts, &cycles);
    ram = new RAM(timers, interrupts, lcd);
    cpu = new CPU(interrupts, timers, lcd, ram, &cycles, debug);

}

void GameBoy::load(std::string filename)
{
    rom.load(filename);
    ram->loadROM(rom);
}

void GameBoy::run()
{
    while (true) {
        unsigned int timeStart = SDL_GetTicks();

        cpu->cpuCycle();
        interrupts->doCycle(cpu);
        timers->doCycle();

        if (!lcd->lcdCycle(timeStart, ram))
            break;
        
        if (debug)
            std::cout << "cycles: " << cycles << std::endl;
    }
}