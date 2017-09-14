
CC=gcc
CFLAGS = -g -Wall -lm
EXEC = ./pc-terminal

all:
	$(CC) $(CFLAGS) pc_terminal.c ../packet.c packet_pc.c joystick.c -o $(EXEC)

run: all
	$(EXEC)
