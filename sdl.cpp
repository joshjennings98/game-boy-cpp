#include <iostream>
#include "sdl.h"

Display::Display() {
    SDL_Init(SDL_INIT_VIDEO);
    window = SDL_CreateWindow(
        "Josh's Game Boy Emulator", 
        SDL_WINDOWPOS_CENTERED, 
        SDL_WINDOWPOS_CENTERED, 
        160, 144, SDL_WINDOW_OPENGL
    ); 
    screen = SDL_GetWindowSurface(window);
}

void Display::sdlSetFrame() {
    SDL_UpdateWindowSurface(window);
}

int Display::sdlUpdate() {
    SDL_Event event;
    
    while (SDL_PollEvent(&event)) {        
        switch (event.type) {
            case SDL_KEYDOWN:
                switch(event.key.keysym.sym) {
                    case SDLK_LEFT:
                        buttons.left = 1;
                        break;
                    case SDLK_RIGHT:
                        buttons.right = 1;
                        break;
                    case SDLK_UP:
                        buttons.up = 1;
                        break;
                    case SDLK_DOWN:
                        buttons.down = 1;
                        break;
                    case SDLK_z:
                        buttons.a = 1;
                        break;
                    case SDLK_x:
                        buttons.b = 1;
                        break;
                    case SDLK_a:
                        buttons.start = 1;
                        break;
                    case SDLK_s:
                        buttons.select = 1;
                        break;
                    default:
                        break;
                }
                break;
            case SDL_KEYUP:
                switch(event.key.keysym.sym) {
                    case SDLK_ESCAPE:
                    case SDLK_q:
                        exit(0);
                        break;
                    case SDLK_LEFT:
                        buttons.left = 0;
                        break;
                    case SDLK_RIGHT:
                        buttons.right = 0;
                        break;
                    case SDLK_UP:
                        buttons.up = 0;
                        break;
                    case SDLK_DOWN:
                        buttons.down = 0;
                        break;
                    case SDLK_z:
                        buttons.a = 0;
                        break;
                    case SDLK_x:
                        buttons.b = 0;
                        break;
                    case SDLK_a:
                        buttons.start = 0;
                        break;
                    case SDLK_s:
                        buttons.select = 0;
                        break;
                    default:
                        break;
                }
                break;
            default:
                break;
        }
    }
    return 0;
}

unsigned int * Display::sdlFrameBuffer() {
    return static_cast<unsigned int*>(screen->pixels);
}

unsigned int Display::getButton() {
    return ((buttons.start * 8) | (buttons.select * 4) | (buttons.b * 2) | buttons.a);
}

unsigned int Display::getDirection() {
    return ((buttons.down * 8) | (buttons.up * 4) | (buttons.left * 2) | buttons.right);
}