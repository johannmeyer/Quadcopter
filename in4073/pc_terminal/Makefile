
CC=gcc
CFLAGS = -g -Wall -lm
EXEC = ./pc-terminal

all:
	$(CC) $(CFLAGS) -I$(abspath ../communication) pc_terminal.c ../communication/packet.c packet_pc.c ../communication/crc.c joystick.c -o $(EXEC)

run: all
	$(EXEC)
