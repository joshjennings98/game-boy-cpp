#ifndef SDL_H
#define SDL_H

#include <SDL2/SDL.h>

struct Buttons {
    
    int start;
    int select;

    int a;
    int b;
    
    int up;
    int down;
    int left;
    int right;

};

class Display {
    private:
        SDL_Window * window;
        SDL_Renderer * renderer;
        SDL_Texture * texture;
        SDL_Surface * screen;

        Buttons buttons;
    public:
        Display();
        void sdlSetFrame();
        int sdlUpdate();
        unsigned int * sdlFrameBuffer();
        unsigned int getButton();
        unsigned int getDirection();

};

#endif