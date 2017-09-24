/*------------------------------------------------------------
 * Simple pc terminal in C
 *
 * Arjan J.C. van Gemund (+ mods by Ioannis Protonotarios)
 *
 * read more: http://mirror.datenwolf.net/serial/
 *******************************------------------------------------------------------------
 */

#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>

/*------------------------------------------------------------
 * console I/O
 *******************************------------------------------------------------------------
 */
struct termios savetty;

void term_initio()
{
								struct termios tty;

								tcgetattr(0, &savetty);
								tcgetattr(0, &tty);

								tty.c_lflag &= ~(ECHO|ECHONL|ICANON|IEXTEN);
								tty.c_cc[VTIME] = 0;
								tty.c_cc[VMIN] = 0;

								tcsetattr(0, TCSADRAIN, &tty);
}

void term_exitio()
{
								tcsetattr(0, TCSADRAIN, &savetty);
}

void term_puts(char *s)
{
								fprintf(stderr,"%s",s);
}

void term_putchar(char c)
{
								putc(c,stderr);
}

int term_getchar_nb()
{
								static unsigned char line [2];

								if (read(0,line,1)) // note: destructive read
																return (int) line[0];

								return -1;
}

int term_getchar()
{
								int c;

								while ((c = term_getchar_nb()) == -1) ;
								return c;
}

/*------------------------------------------------------------
 * Serial I/O
 * 8 bits, 1 stopbit, no parity,
 * 115,200 baud
 *******************************------------------------------------------------------------
 */
#include <termios.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <assert.h>
#include <time.h>
#include <stdbool.h>
#include "pc_terminal.h"
#include "packet_pc.h"

int serial_device = 0;
int fd_RS232;

bool isMode(uint8_t c)
{
	if (c	>= '0' && c <= '8')			// check if number is between 0 and 8
	{
		return true;
	}
	else{
		return false;
	}
}

void get_key(char c)
{
	switch (c)
	{
		case 'a':
			lift_key = lift_key + TRIM_VALUE;
			break;
		case 'z':
			lift_key = lift_key - TRIM_VALUE;
			if(lift_key < 0)
			lift_key =0;
			break;
		case 'q':
			yaw_key = yaw_key + TRIM_VALUE;
			break;
		case 'w':
			yaw_key = yaw_key - TRIM_VALUE;
			break;
		case 'u':

			break;
		case 'j':

			break;
		case 'i':

			break;
		case 'k':

			break;
		case 'o':

			break;
		case 'l':

			break;

		case 'A':									// up key
			pitch_key = pitch_key +TRIM_VALUE;
			break;

		case 'B':									// down key
			pitch_key = pitch_key - TRIM_VALUE;
			break;

		case 'C':									// right key
			roll_key = roll_key + TRIM_VALUE;
			break;
		case 'D':									// left key
			roll_key = roll_key - TRIM_VALUE;
			break;

		case 27:
			mode = EXIT_MODE;
			break;

		default:
			if(isMode(c))
			{
				mode = c - 48;            // or (int)c
			}
			else{
					term_puts("\n Invalid character\n");
				}
	}
}

void combine_values()
{
	lift = lift_key - lift_js;
	pitch = pitch_key + pitch_js;
	roll = roll_key + roll_js;
	yaw = yaw_key + yaw_js;
	if(lift < -32767)
	{
		lift = - 32767;
	}
	else if(lift > 32767)
	{
		lift =  32767;
	}
	if(pitch < -32767)
	{
		pitch = - 32767;
	}
	else if(pitch > 32767)
	{
		pitch =  32767;
	}
	if(roll < -32767)
	{
		roll = - 32767;
	}
	else if(roll > 32767)
	{
		roll =  32767;
	}

	if(yaw < -32767)
	{
		yaw = - 32767;
	}
	else if(yaw > 32767)
	{
		yaw =  32767;
	}
	//printf("Roll_key: %x, roll: %x \n",roll_key,roll );

}

void rs232_open(void)
{
								char   *name;
								int result;
								struct termios tty;

								fd_RS232 = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);  // Hardcode your serial port here, or request it as an argument at runtime
								assert(fd_RS232>=0);

								result = isatty(fd_RS232);
								assert(result == 1);

								name = ttyname(fd_RS232);
								assert(name != 0);

								result = tcgetattr(fd_RS232, &tty);
								assert(result == 0);

								tty.c_iflag = IGNBRK; /* ignore break condition */
								tty.c_oflag = 0;
								tty.c_lflag = 0;

								tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; /* 8 bits-per-character */
								tty.c_cflag |= CLOCAL | CREAD; /* Ignore model status + read input */

								cfsetospeed(&tty, B115200);
								cfsetispeed(&tty, B115200);

								tty.c_cc[VMIN]  = 0;
								tty.c_cc[VTIME] = 0; // removed timeout

								tty.c_iflag &= ~(IXON|IXOFF|IXANY);

								result = tcsetattr (fd_RS232, TCSANOW, &tty); /* non-canonical */

								tcflush(fd_RS232, TCIOFLUSH); /* flush I/O buffer */
}


void  rs232_close(void)
{
								int result;

								result = close(fd_RS232);
								assert (result==0);
}


int rs232_getchar_nb()
{
								int result;
								unsigned char c;

								result = read(fd_RS232, &c, 1);

								if (result == 0)
																return -1;

								else
								{
																assert(result == 1);
																return (int) c;
								}
}


int  rs232_getchar()
{
								int c;

								while ((c = rs232_getchar_nb()) == -1) ;
								return c;
}


int  rs232_putpacket(packet *my_packet)
{
								int result;

								do {
																result = (int) write(fd_RS232, my_packet, sizeof(packet));
								} while (result == 0);

								assert(result == sizeof(packet));
								return result;
}


/*----------------------------------------------------------------
 * main -- execute terminal
 *******************************----------------------------------------------------------------
 */
#include "joystick.h"

int main(int argc, char **argv)
{
								char c;
								clock_t start;
								lift_js= 0;  //-32768 if not using joystick
								pitch_js=0;
								roll_js=0;
								yaw_js=0;
								lift_key=0;
								pitch_key=0;
								roll_key=0;
								yaw_key=0;
								lift=0;
								pitch=0;
								roll=0;
								yaw=0;


								term_puts("\nTerminal program - Embedded Real-Time Systems\n");

								term_initio();
								rs232_open();
								joystick_open();
								term_puts("Type ^C to exit\n");

								/*
								   discard any incoming text
								 */
								while ((c = rs232_getchar_nb()) != -1)
																fputc(c,stderr);

								/*
								   send & receive
								 */
								packet my_packet;
								for (;;)
								{
																// Sends data to the Drone
																char readChar = -1;
																if ((readChar = term_getchar_nb()) != -1)
																{
																	do {
																		c = readChar;
																	} while((readChar = term_getchar_nb()) != -1);				// check for Esc key in order to read arrow keys
																	get_key(c);
																}
																int panic = get_joystick_action(&roll_js, &pitch_js, &yaw_js, &lift_js);
																//printf("joystick: %d | %d |%d | %d | %d|\n", roll_js,pitch_js,yaw_js,lift_js,panic);
																// TODO combine the keyboard and joystick data
																combine_values();
																//printf("joystick: %d | %d |%d | %d | \n", roll,pitch,yaw,lift);
																if (panic) encode(&my_packet, PANIC_MODE, PACKET_TYPE_COMMAND);
																else encode(&my_packet, mode, PACKET_TYPE_COMMAND);

																rs232_putpacket(&my_packet);

																// Reads data sent from the Drone
																start = clock();
																while((clock()-start)/(double)CLOCKS_PER_SEC<0.05)
																{
																	if ((c = rs232_getchar_nb()) != -1) term_putchar(c);
																}
								}

								term_exitio();
								joystick_close();
								rs232_close();
								term_puts("\n<exit>\n");

								return 0;
}
