#ifndef SDL_H
#define SDL_H

#include <array>
#include <iostream>
#include "SDL2/SDL.h"
#include <sys/time.h>

class Keypad {
    private:
        uint8_t Up;
        uint8_t Down;
        uint8_t Left;
        uint8_t Right;
        uint8_t A;
        uint8_t B;
        uint8_t Start;
        uint8_t Select;
        
        std::array<uint8_t, 8> keys;

        bool UpPressed;
        bool DownPressed;
        bool LeftPressed;
        bool RightPressed;
        bool APressed;
        bool BPressed;
        bool StartPressed;
        bool SelectPressed;

    public:
        Keypad();
        uint8_t getKey(int i);
        void handleInput(SDL_Event e);
        void setPressed(int key, bool value);
};

class Display {
    private:
        std::array<std::array<int, 144>, 160> display;
    public:
        SDL_Window *screen;
        SDL_Renderer *renderer;
        SDL_Texture *texture;
        SDL_Surface *surface;
        unsigned int frames;

        Display();
        void set(int x, int y, int pixel);
        int get(int x, int y);
        void drawDisplay(int scale, SDL_Renderer * renderer);
        std::array<std::array<int, 144>, 160> getDisplay();
        void sdlSetFrame();
        unsigned int * sdlFrameBuffer();
        void sdlInit();
};


#endif