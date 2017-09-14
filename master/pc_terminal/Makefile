
CC=gcc
CFLAGS = -g -Wall -lm
EXEC = ./pc-terminal

all:
	$(CC) $(CFLAGS) -I$(abspath ../crc) -I$(abspath ../) pc_terminal.c ../crc/crc.c ../packet.c packet_pc.c joystick.c -o $(EXEC)

run: all
	$(EXEC)
