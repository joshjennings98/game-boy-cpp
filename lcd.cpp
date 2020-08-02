#include <iostream>
#include <vector>
#include "cpu.h"
#include "lcd.h"
#include "ram.h"
#include "interrupts.h"
#include "sdl.h"
#include "ram.h"

int bgPalette[] = {3,2,1,0};
int spritePalette1[] = {0, 1, 2, 3};
int spritePalette2[] = {0, 1, 2, 3};
unsigned long colours[4] = {0xFFFFFF, 0xC0C0C0, 0x808080, 0x000000};

void Sprite::setX(int val)
{
    x = val;
}

void Sprite::setY(int val)
{
    y = val;
}

int Sprite::getX()
{
    return x;
}

int Sprite::getY()
{
    return y;
}


void Sprite::setFlags(int val)
{
    flags = val;
}

void Sprite::setPatternNum(int val)
{
    patternNum = val;
}

int Sprite::getFlags()
{
    return flags;
}

int Sprite::getPatternNum()
{
    return patternNum;
}


LCD::LCD(/* RAM * ram, */ Interrupts * interrupts)
{
    //lcdc = LCDC();
    //lcds = LCDS();

    //this->cpu = cpu;
    //this->ram = ram;
    this->interrupts = interrupts;
    std::cout << "LCD inited" << std::endl;
}

LCDC::LCDC() {
    lcdDisplay = 0; // 7
    windowTileMap = 0; // 6
    windowDisplay = 0; // 5
    tileDataSelect = 0; // 4
    tileMapSelect = 0; // 3
    spriteSize = 0; // 2
    spriteDisplay = 0; // 1
    bgWindowDisplay = 0; // 0
    std::cout << "LCDC inited" << lcdDisplay << std::endl;
}

LCDS::LCDS() {
    lyInterrupt = 0;
    oamInterrupt = 0;
    vblankInterrupt = 0;
    hblankInterrupt = 0;
    lyFlag = 0;
    modeFlag = 0;
    std::cout << "LCDS inited" << std::endl;
}

void LCDC::setLCDC(uint8_t address) {
    lcdDisplay = (!!(address & 0x80));
    windowTileMap = (!!(address & 0x40));
    windowDisplay = (!!(address & 0x20));
    tileDataSelect = (!!(address & 0x10));
    tileMapSelect = (!!(address & 0x08));
    spriteSize = (!!(address & 0x04));
    spriteDisplay = (!!(address & 0x02));
    bgWindowDisplay = (!!(address & 0x01));
}

uint8_t LCDC::getLCDC(void) {
    return (
        (lcdDisplay << 7) | 
        (windowTileMap << 6) | 
        (windowDisplay << 5) | 
        (tileDataSelect << 4) | 
        (tileMapSelect << 3) | 
        (spriteSize << 2) | 
        (spriteDisplay << 1) | 
        (bgWindowDisplay));
}

void LCDS::setLCDS(uint8_t address) {
    lyInterrupt = (!!(address & 0x40));
    oamInterrupt = ((address & 0x20) >> 5);
    vblankInterrupt = ((address & 0x10) >> 4);
    hblankInterrupt = ((address & 0x08) >> 3);
    lyFlag = ((address & 0x04) >> 2);
    modeFlag = ((address & 0x03));
}

void LCDS::setModeFlag(int value)
{
    modeFlag = value;
}

int LCDS::getLyInterrupt()
{
    return lyInterrupt;
}

uint8_t LCDS::getLCDS(void) {
    return (
        (lyInterrupt << 6) | 
        (oamInterrupt << 5) | 
        (vblankInterrupt << 4) | 
        (hblankInterrupt << 3) | 
        (lyFlag << 2) | 
        (modeFlag));
}

void LCD::setBGPalette(uint8_t address) {
    bgPalette[3] = ((address >> 6) & 0x03);
    bgPalette[2] = ((address >> 4) & 0x03);
    bgPalette[1] = ((address >> 2) & 0x03);
    bgPalette[0] = ((address) & 0x03);
}

void LCD::setSpritePalette1(uint8_t address) {
    spritePalette1[3] = ((address >> 6) & 0x03);
    spritePalette1[2] = ((address >> 4) & 0x03);
    spritePalette1[1] = ((address >> 2) & 0x03);
    spritePalette1[0] = 0;
}

void LCD::setSpritePalette2(uint8_t address) {
    spritePalette2[3] = ((address >> 6) & 0x03);
    spritePalette2[2] = ((address >> 4) & 0x03);
    spritePalette2[1] = ((address >> 2) & 0x03);
    spritePalette2[0] = 0;
}

void LCD::setScrollX(uint8_t address) {
    scrollX = address;
}

uint8_t LCD::getScrollX(void) {
    return scrollX;
}

void LCD::setScrollY(uint8_t address) {
    scrollY = address;
}

uint8_t LCD::getScrollY(void) {
    return scrollY;
}

void LCD::setWindowX(uint8_t address) {
    windowX = address;
}

void LCD::setWindowY(uint8_t address) {
    windowY = address;
}

int LCD::getLine(void) {
    return line;
}

void LCD::setLyCompare(uint8_t address) {
    lyCompare = (line == address);
}

std::vector<Sprite> LCD::sortSprites(std::vector<Sprite> sprites, int c) {
    
    std::vector<Sprite> newSprites(sprites); 
    
    // insertion sort 
    int i, j;
    for (i = 0; i < c; i++) {
        for (j = 0; j < c-1; j++) {
            if (newSprites[j].getX() < newSprites[j+1].getX()) {
                Sprite s;

                s = newSprites[j+1];
                newSprites[j+1] = newSprites[j];
                newSprites[j] = s;    
            }
        }
    }
}

int LCDC::getTileDataSelect()
{
    return tileDataSelect;
}

int LCDC::getTileMapSelect()
{
    return tileMapSelect;
}

int LCDC::getWindowDisplay()
{
    return windowDisplay;
}

int LCDC::getWindowTileMap()
{
    return windowTileMap;
}

int LCDC::getBgWindowDisplay()
{
    return bgWindowDisplay;
}

void LCD::drawBgWindow(unsigned int *buf, int line, RAM * ram) {
	int x;
	for(x = 0; x < 160; x++) // for the x size of the window (160x144)
	{
		unsigned int mapSelect, tileMapOffset, tileNum, tileAddr, currX, currY;
        uint8_t buf1, buf2, mask, colour;

		if(line >= windowY && lcdc.getWindowDisplay() && line - windowY < 144) {
			currX = x;
			currY = line - windowY;
			mapSelect = lcdc.getWindowTileMap();
		} else { 
            
            if (!lcdc.getBgWindowDisplay()) { // background
                buf[line*160 + x] = 0; // if not window or background, make it white
                return;
            }
            
            currX = (x + scrollX) % 256; // mod 256 since if it goes off the screen, it wraps around
			currY = (line + scrollY) % 256;
			mapSelect = lcdc.getTileMapSelect();
		} 

        // map window to 32 rows of 32 bytes
		tileMapOffset = (currY/8)*32 + currX/8;

		tileNum = ram->readByte(0x9800 + mapSelect*0x400 + tileMapOffset);
		if(lcdc.getTileDataSelect())
			tileAddr = 0x8000 + (tileNum*16);
		else
			tileAddr = 0x9000 + (((signed int)tileNum)*16); // pattern 0 lies at 0x9000

		buf1 = ram->readByte(tileAddr + (currY%8)*2); // 2 bytes represent the line
		buf2 = ram->readByte(tileAddr + (currY%8)*2 + 1);
		mask = 128>>(currX%8);
		colour = (!!(buf2&mask)<<1) | !!(buf1&mask);
		buf[line*160 + x] = colours[bgPalette[colour]];
	}
}

int LCDC::getSpriteSize() 
{
    return spriteSize;
}

void LCD::drawSprites(unsigned int *buf, int line, int blocks, std::vector<Sprite> sprites, RAM * ram) {
	
    int i;
	for(i = 0; i < blocks; i++)
	{
		unsigned int buf1, buf2, tileAddr, spriteRow, x;

		// off screen
		if(sprites[i].getX() < -7) {
			continue;
        }

        spriteRow = 
            sprites[i].getFlags() & 
            0x40 ? (lcdc.getSpriteSize() ? 15 : 7) - (line - sprites[i].getY()) : line - sprites[i].getY();

        // similar to background
		tileAddr = 0x8000 + (sprites[i].getPatternNum()*16) + spriteRow*2;

		buf1 = ram->readByte(tileAddr);
		buf2 = ram->readByte(tileAddr+1);

		// draw each pixel
		for(x = 0; x < 8; x++)
		{
            // out of bounds check
            if((sprites[i].getX() + x) >= 160) {
				continue;
            }

			uint8_t mask, colour;
			int *pal;

            if((sprites[i].getX() + x) >= 160)
				continue;

			mask = sprites[i].getFlags() & 0x20 ? 128>>(7-x) : 128>>x;
			colour = ((!!(buf2&mask))<<1) | !!(buf1&mask);

			if(colour == 0) {// no need to draw it
                continue;
            }

			pal = (sprites[i].getFlags() & 0x10) ? spritePalette2 : spritePalette1;

			// only render over colour 0
			if(sprites[i].getFlags() & 0x80)
			{
				unsigned int temp = buf[line*160+(x + sprites[i].getX())];
				if(temp != colours[bgPalette[0]]) {
					continue;
                }
			}
			buf[line*160+(x + sprites[i].getX())] = colours[pal[colour]]; // for testing
		}
	}
}

void LCD::renderLine(int line, RAM * ram) {

    int i = 0; // num of OAM blocks
    int c = 0; // block counter
    std::vector<Sprite> sprites = {}; 
    sprites.resize(10); // max 10 sprites per line

    unsigned int * buf = display->sdlFrameBuffer();

    // OAM is divided into 40 4-byte blocks each - corresponding to a sprite
    for (i = 0; i < 40; i++) {

        int y;
        y = ram->readByte(0xFE00 + (i*4)) - 16;

        if (line < y || line >= y + 8 + (8*lcdc.getSpriteSize())) // out of bounds check
            continue;

        sprites[c].setY(y);
        sprites[c].setX(ram->readByte(0xFE00 + (i*4) + 1) - 8);
        sprites[c].setPatternNum(ram->readByte(0xFE00 + (i*4) + 2));
        sprites[c].setFlags(ram->readByte(0xFE00 + (i*4) + 3));
        c++;
        // max 10 sprites per line
        if (c == 10)
            break;
    }

    if (c)
        sortSprites(sprites, c);

    drawBgWindow(buf, line, ram);
    drawSprites(buf, line, c, sprites, ram);
}

Keypad * LCD::getKeyPad()
{
    return keypad;
}

Display * LCD::getDisplay()
{
    return display;
}

int LCD::lcdCycle(int timeStart, RAM * ram) {
    int cycles = 0; //getCycles();
    static int prevLine;

    int this_frame;
    int end = 0;
    
    this_frame = cycles % (70224/4); // 70224 clks per screen
    line = this_frame / (456/4); // 465 clks per line

    if (this_frame < 204/4)
        lcds.setModeFlag(2);  // OAM
    else if (this_frame < 284/4)
        lcds.setModeFlag(3);  // VRA
    else if (this_frame < 456/4)
        lcds.setModeFlag(0);  // HBlank
    
    // Done all lines
    if (line >= 144)
        lcds.setModeFlag(1);  // VBlank

    if (line != prevLine && line < 144) {
        renderLine(line, ram);
    }

    if (lcds.getLyInterrupt() && line == lyCompare) {
        interrupts->updateFlags(lcdstat);
    }

    if (prevLine == 143 && line == 144) {
        // draw the entire frame
        interrupts->updateFlags(vblank);
        getDisplay()->sdlSetFrame();
        SDL_Event e;
        
        getKeyPad()->handleInput(e);

        float deltaT = (float)1000 / (59.7) - (float)(SDL_GetTicks() - timeStart);
        if (deltaT > 0)
            SDL_Delay(deltaT);
    }

    if (end)   
        return 0;
    
    prevLine = line;
    
    return 1;
}

