CC=g++
LDFLAGS=-lSDL2

all: clean gameboy

gameboy:
	$(CC) main.cpp -o gameboy $(LDFLAGS)

clean:
	rm -f gameboy