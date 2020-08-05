#ifndef LCD_H
#define LCD_H

#include <iostream>
#include "ram.h"
#include "interrupts.h"
#include "sdl.h"
//#include "cpu.h"
// #include "ram.h"

class RAM;

// LCD Control 
class LCDC {
    public:
    //private:
        int lcdDisplay; // 7
        int windowTileMap; // 6
        int windowDisplay; // 5
        int tileDataSelect; // 4
        int tileMapSelect; // 3
        int spriteSize; // 2
        int spriteDisplay; // 1
        int bgWindowDisplay; // 0
    //public:
        LCDC();
        void setLCDC(uint8_t value);
        uint8_t getLCDC();
        int getSpriteSize();
        int getTileDataSelect();
        int getTileMapSelect();
        int getWindowDisplay();
        int getWindowTileMap();
        int getBgWindowDisplay();
};

// LCD STAT
class LCDS {
    private:
        int lyInterrupt;
        int oamInterrupt;
        int vblankInterrupt;
        int hblankInterrupt;
        int lyFlag;
        int modeFlag;
    public:
        LCDS();
        void setLCDS(uint8_t value);
        uint8_t getLCDS();
        void setModeFlag(int value);
        int getLyInterrupt();
};

class Sprite {
    private:
        int y;
        int x;
        int patternNum;
        int flags;
    public:
        void setX(int val);
        void setY(int val);
        int getX();
        int getY();

        void setFlags(int val);
        void setPatternNum(int val);
        int getFlags();
        int getPatternNum();
};

class LCD {
    private:
        int windowX;
        int windowY;
        int scrollX;
        int scrollY;
        int line;
        int frame;
        int lyCompare;

        Display * display;
        //Keypad * keypad;
        unsigned int * cycles;
    public:
        LCDS lcds;
        LCDC lcdc;

        Interrupts * interrupts;
        
        LCD(Interrupts * interrupts, unsigned int * cycles, Display * display);

        //Keypad * getKeyPad();
        //Display * getDisplay();

        void setScrollX(uint8_t value);
        void setScrollY(uint8_t value);
        void setWindowX(uint8_t value);
        void setWindowY(uint8_t value);
        void setLyCompare(uint8_t value);
        void setBGPalette(uint8_t value);
        void setSpritePalette1(uint8_t value);
        void setSpritePalette2(uint8_t value);

        uint8_t getScrollX();
        uint8_t getScrollY();
        uint8_t getWindowX();
        uint8_t getWindowY();
        int getLine();
        int lcdCycle(unsigned int timeStart, RAM * ram);

        void renderLine(int line, RAM * ram);
        void drawBgWindow(unsigned int *buf, int line, RAM * ram);
        void drawSprites(unsigned int *buf, int line, int blocks, std::vector<Sprite> sprites, RAM * ram);
        std::vector<Sprite> sortSprites(std::vector<Sprite> sprites, int c);
        void draw_stuff();
};

#endif