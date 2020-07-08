#include <iostream>
#include <chrono>
#include <thread>
#include <SDL2/SDL.h>
#include "ram.cpp"
#include "rom.cpp"
#include "interrupts.cpp"
#include "timer.cpp"
#include "io.cpp"

int main() {
    RAM ram;
    ROM rom("tetris.gb");
    ram.loadROM(rom);

    // test crap
    // ram.dump(0x0000, 0x0020);

    Keypad keypad = Keypad();
    Display display = Display();

    int scale = 8;
    int width = scale * 160, height = scale * 144;

    SDL_Event e;

    SDL_Window* window = SDL_CreateWindow ("Gameboy Emulator", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, SDL_WINDOW_SHOWN);;
    SDL_Renderer* renderer = SDL_CreateRenderer( window, -1, SDL_RENDERER_ACCELERATED);
    SDL_SetRenderDrawColor( renderer, 0, 0, 0, 255 );
    SDL_RenderPresent(renderer);

    //while (true) {
    for (int i = 0; i < 500; i++) {
               
        
        // Get rid of
        for (int i = 0; i < 160; i++) {
            for (int j = 0; j < 144; j++) {
                display.set(i, j, (display.get(i, j) + 1) % 255);
            }
        }
        // */

        display.drawDisplay(scale, renderer);
        keypad.handleInput(e);

        // Slow down emulation speed
        std::this_thread::sleep_for(std::chrono::microseconds(1200));

        //std::cout << "cycle " << i << std::endl;
    }

    return 0;
}