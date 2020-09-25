#include <iostream>
#include <vector>
#include <array>
#include "cpu.h"
#include "lcd.h"
#include "ram.h"
#include "interrupts.h"
#include "sdl.h"
#include "ram.h"

std::array<int, 4> bgPalette = {3, 2, 1, 0};
std::array<int, 4> spritePalette1 = {0, 1, 2, 3};
std::array<int, 4> spritePalette2 = {0, 1, 2, 3};
std::array<unsigned long, 4> colours = {0xFFFFFF, 0xC0C0C0, 0x808080, 0x000000};

Sprite::Sprite(int x, int y, int patternNum, int flags)
{
    this->x = x;
    this->y = y;
    this->patternNum = patternNum;
    this->flags = flags;
}

int Sprite::getX()
{
    return x;
}

int Sprite::getY()
{
    return y;
}

int Sprite::getFlags()
{
    return flags;
}

int Sprite::getPatternNum()
{
    return patternNum;
}

LCD::LCD(Interrupts * interrupts, unsigned int * cycles, Display * display)
{
    this->cycles = cycles;
    this->interrupts = interrupts;
    this->display = display;
}

LCDC::LCDC() {
    lcdDisplay = 0;
    windowTileMap = 0;
    windowDisplay = 0;
    tileDataSelect = 0;
    tileMapSelect = 0;
    spriteSize = 0;
    spriteDisplay = 0;
    bgWindowDisplay = 0;
    }

LCDS::LCDS() {
    lyInterrupt = 0;
    oamInterrupt = 0;
    vblankInterrupt = 0;
    hblankInterrupt = 0;
    lyFlag = 0;
    modeFlag = 0;
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

uint8_t LCDC::getLCDC() {
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
    lyInterrupt = !!(address & 0x40);
    oamInterrupt = (address & 0x20) >> 5;
    vblankInterrupt = (address & 0x10) >> 4;
    hblankInterrupt = (address & 0x08) >> 3;
    lyFlag = (address & 0x04) >> 2;
    modeFlag = address & 0x03;
}

void LCDS::setModeFlag(ModeFlag value)
{
    modeFlag = value;
}

int LCDS::getLyInterrupt()
{
    return lyInterrupt;
}

uint8_t LCDS::getLCDS() {
    return (
        (lyInterrupt << 6) | 
        (oamInterrupt << 5) | 
        (vblankInterrupt << 4) | 
        (hblankInterrupt << 3) | 
        (lyFlag << 2) | 
        (modeFlag));
}

void LCD::setBGPalette(uint8_t address) {
    bgPalette[3] = (address >> 6) & 0x03;
    bgPalette[2] = (address >> 4) & 0x03;
    bgPalette[1] = (address >> 2) & 0x03;
    bgPalette[0] = (address) & 0x03;
}

void LCD::setSpritePalette1(uint8_t address) {
    spritePalette1[3] = (address >> 6) & 0x03;
    spritePalette1[2] = (address >> 4) & 0x03;
    spritePalette1[1] = (address >> 2) & 0x03;
    spritePalette1[0] = 0;
}

void LCD::setSpritePalette2(uint8_t address) {
    spritePalette2[3] = (address >> 6) & 0x03;
    spritePalette2[2] = (address >> 4) & 0x03;
    spritePalette2[1] = (address >> 2) & 0x03;
    spritePalette2[0] = 0;
}

void LCD::setScrollX(uint8_t address) {
    scrollX = address;
}

uint8_t LCD::getScrollX() {
    return scrollX;
}

void LCD::setScrollY(uint8_t address) {
    scrollY = address;
}

uint8_t LCD::getScrollY() {
    return scrollY;
}

void LCD::setWindowX(uint8_t address) {
    windowX = address;
}

void LCD::setWindowY(uint8_t address) {
    windowY = address;
}

int LCD::getLine() {
    return line;
}

void LCD::setLyCompare(uint8_t address) {
    lyCompare = (line == address);
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
	for(int x = 0; x < 160; x++) { // for the x size of the window (160x144)
		unsigned int mapSelect, tileMapOffset, tileNum, tileAddr, currX, currY;
        uint8_t buf1, buf2, mask, colour;

		if (line >= windowY && lcdc.getWindowDisplay() && line - windowY < 144) {
			currX = x;
			currY = line - windowY;
			mapSelect = lcdc.getWindowTileMap();
		} else { 
            
            if (!lcdc.getBgWindowDisplay()) {
                buf[line * 160 + x] = 0; // if not window or background, make it white
                return;
            }
            
            currX = (x + scrollX) % 256; // mod 256 since if it goes off the screen, it wraps around
            currY = (line + scrollY) % 256;
            mapSelect = lcdc.getTileMapSelect();
		} 

        // map window to 32 rows of 32 bytes
		tileMapOffset = (currY / 8) * 32 + currX / 8;

		tileNum = ram->readByte(0x9800 + mapSelect * 0x400 + tileMapOffset);
		if (lcdc.getTileDataSelect()) {
			tileAddr = 0x8000 + tileNum * 16;
        }
		else {
			tileAddr = 0x9000 + tileNum * 16; // pattern 0 is at 0x9000
        }

		buf1 = ram->readByte(tileAddr + (currY % 8) * 2); // 2 bytes represent the line
		buf2 = ram->readByte(tileAddr + (currY % 8) * 2 + 1);

		mask = 128 >> (currX % 8);
		colour = (!!(buf2 & mask) << 1) | !!(buf1 & mask);
		buf[line * 160 + x] = colours[bgPalette[colour]];
	}
}

int LCDC::getSpriteSize() 
{
    return spriteSize;
}

void LCD::drawSprites(unsigned int *buf, int line, std::vector<Sprite> &sprites, RAM * ram) {
	
    for(int i = 0; i < sprites.size(); i++) {
		unsigned int buf1, buf2, tileAddr, spriteRow;

		if (sprites[i].getX() >= -7) { // make sure on screen

            spriteRow = 
                sprites[i].getFlags() & 0x40 
                ? (lcdc.getSpriteSize() ? 15 : 7) - (line - sprites[i].getY()) 
                : line - sprites[i].getY();

            tileAddr = 0x8000 + sprites[i].getPatternNum() * 16 + spriteRow * 2;

            buf1 = ram->readByte(tileAddr);
            buf2 = ram->readByte(tileAddr + 1);

            for(int x = 0; x < 8; x++) { // draw each pixel
                if ((sprites[i].getX() + x) < 160) { // out of bounds check
                    uint8_t mask, colour;
                    int * pal;

                    mask = sprites[i].getFlags() & 0x20 ? 128 >> (7 - x) : 128 >> x;
                    colour = ((!!(buf2 & mask)) << 1) | !!(buf1 & mask);

                    if (colour != 0) { // don't need to draw anything if colour is zero
                        pal = (sprites[i].getFlags() & 0x10) ? spritePalette2.data() : spritePalette1.data();
                        buf[line * 160 + x + sprites[i].getX()] = colours[pal[colour]];
                    }
                }
            }
        }
	}
}

void LCD::renderLine(int line, RAM * ram) {

    std::vector<Sprite> sprites;
    unsigned int * buf = display->sdlFrameBuffer();

    // OAM is divided into 40 4-byte blocks each - corresponding to a sprite with a maximum of 10 sprites per line
    for (int i = 0; i < 40 && sprites.size() <= 10; i++) {

        int y = ram->readByte(0xFE00 + (i*4)) - 16;

        if (line >= y && line < y + 8 + 8 * lcdc.getSpriteSize()) { // make sure in bounds
            Sprite s(ram->readByte(0xFE00 + (i * 4) + 1) - 8, y, ram->readByte(0xFE00 + (i * 4) + 2), ram->readByte(0xFE00 + (i * 4) + 3));
            sprites.push_back(s);
        }
    }

    drawBgWindow(buf, line, ram);
    drawSprites(buf, line, sprites, ram);
}

int LCD::lcdCycle(unsigned int timeStart, RAM * ram) {
    static int prevLine;
    int currentFrame;

    bool end = false;
    
    currentFrame = *cycles % (70224 / 4); // 70224 clks per screen
    line = currentFrame / (456 / 4); // 465 clks per line

    if (currentFrame < 204 / 4) {
        lcds.setModeFlag(OAM); 
    }
    else if (currentFrame < 284 / 4) {
        lcds.setModeFlag(VRAM);
    }
    else if (currentFrame < 456 / 4) {
        lcds.setModeFlag(HBlank); 
    }
    
    if (line >= 144) { // Done all lines
        lcds.setModeFlag(VBlank);
    }

    if (line != prevLine && line < 144) {
        renderLine(line, ram);
    }

    if (lcds.getLyInterrupt() && line == lyCompare) {
        interrupts->updateFlags(lcdstat);
    }

    if (prevLine == 143 && line == 144) {
        interrupts->updateFlags(vblank);
        display->sdlSetFrame();

        if (display->sdlUpdate()) {
            end = true;
        }

        float deltaT = 1000.0 / (59.7) - (float)(SDL_GetTicks() - timeStart);
        if (deltaT > 0) {
            SDL_Delay(deltaT);
        }
    }

    if (end == true) {
        return 0;
    }
    else {
        prevLine = line;
        return 1;
    }
}

