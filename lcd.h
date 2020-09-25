#ifndef LCD_H
#define LCD_H

#include <iostream>
#include "ram.h"
#include "interrupts.h"
#include "sdl.h"

enum ModeFlag {HBlank, VBlank, OAM, VRAM};

class RAM;

// LCD Control 
class LCDC {
    private:
        int lcdDisplay; 
        int windowTileMap;
        int windowDisplay; 
        int tileDataSelect; 
        int tileMapSelect; 
        int spriteSize; 
        int spriteDisplay; 
        int bgWindowDisplay; 
    public:
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
        void setModeFlag(ModeFlag value);
        int getLyInterrupt();
};

class Sprite {
    private:
        int y;
        int x;
        int patternNum;
        int flags;
    public:
        Sprite(int x, int y, int patternNum, int flags);
        int getX();
        int getY();
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
        unsigned int * cycles;
    public:
        LCDS lcds;
        LCDC lcdc;

        Interrupts * interrupts;
        
        LCD(Interrupts * interrupts, unsigned int * cycles, Display * display);

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
        void drawSprites(unsigned int *buf, int line, std::vector<Sprite> &sprites, RAM * ram);
        void draw_stuff();
};

#endif