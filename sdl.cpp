#include <iostream>
#include <SDL2/SDL.h>
#include "sdl.h"

struct timeval t1, t2;
SDL_Surface * screenX;

Keypad::Keypad()
{
    Up = SDLK_w;
    Down = SDLK_s;
    Left = SDLK_a;
    Right = SDLK_d;
    A = SDLK_o;
    B = SDLK_p;
    Start = SDLK_u;
    Select = SDLK_i;

    keys = {Up, Down, Left, Right, A, B, Start, Select};

    UpPressed = 0;
    DownPressed = 0;
    LeftPressed = 0;
    RightPressed = 0;
    APressed = 0;
    BPressed = 0;
    StartPressed = 0;
    SelectPressed = 0;
}

uint8_t Keypad::getKey(int i)
{
    return keys[i];
}

Display::Display()
{
    for (int i = 0; i < 160; i++) {
        for (int j = 0; j < 144; j++) {
            display[i][j] = 0;
        }
    }
}

std::array<std::array<int, 144>, 160> Display::getDisplay()
{
    return display;
}

void Display::set(int x, int y, int pixel)
{
    display[x][y] = pixel;
}

int Display::get(int x, int y)
{
    return display[x][y];
}

void Display::drawDisplay(int scale, SDL_Renderer * renderer) 
{
    
    SDL_Rect Message_Rect;
    SDL_Surface* surface;
    SDL_Texture* Message;

    int pixel = 0;
    
    for (int i = 0; i < 160; i++) {
        for (int j = 0; j < 144; j++) {
            SDL_Rect r;
            r.x = i * scale;
            r.y = j * scale;
            r.w = scale;
            r.h = scale;

            pixel = display[i][j];
            
            SDL_SetRenderDrawColor( renderer, pixel, pixel, pixel, 255 );
            SDL_RenderFillRect( renderer, &r );
        }
    }
    SDL_RenderPresent(renderer);
}

void Display::sdlSetFrame()
{
    if (frames == 0) {
        gettimeofday(&t1, NULL);
    }
    frames++;
    if (frames % 1000 == 0) { 
        gettimeofday(&t2, NULL);
        printf("FPS: %i\n", frames/((int)t2.tv_sec - (int)t1.tv_sec));
    }
    SDL_UpdateWindowSurface(screen);
}

void Display::sdlInit() {
    SDL_Init(SDL_INIT_VIDEO);
    this->screen = SDL_CreateWindow("Game Boy Emulator", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 160, 144, SDL_WINDOW_OPENGL); 
    screenX = SDL_GetWindowSurface(this->screen);
    frames = 0;
}

unsigned int * Display::sdlFrameBuffer()
{
    return 0 ; //screen->pixels;
}

void Keypad::setPressed(int key, bool value) // {Up, Down, Left, Right, A, B, Start, Select}
{
    switch(key) {
        case 0:
            UpPressed = value;
            break;
        case 1:
            DownPressed = value;
            break;
        case 2:
            LeftPressed = value;
            break;
        case 3:
            RightPressed = value;
            break;
        case 4:
            APressed = value;
            break;
        case 5:
            BPressed = value;
            break;
        case 6:
            StartPressed = value;
            break;
        case 7:
            SelectPressed = value;
            break;
    }
}

void Keypad::handleInput(SDL_Event e)
{
    while (SDL_PollEvent(&e)) {
        // Handle key events
        switch (e.type) {
            case(SDL_KEYDOWN): 
            {
                if (e.key.keysym.sym == SDLK_ESCAPE) {
                    exit(0);
                }

                for (int i = 0; i < 8; i++) {
                    if (e.key.keysym.sym == getKey(i)) {
                        setPressed(i, 1);
                        std::cout << "Key " << i << " pressed." << std::endl;
                    }
                }
                break;
            }
            case(SDL_KEYUP):
            {
                for (int i = 0; i < 8; i++) {
                    if (e.key.keysym.sym == getKey(i)) {
                        setPressed(i, 0);
                        std::cout << "Key " << i << " released." << std::endl;
                    }
                }
                break;
            }
            default:
                break;
        }
    }
}