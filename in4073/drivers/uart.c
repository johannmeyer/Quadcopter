/*------------------------------------------------------------------
 *  uart.c -- configures uart
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *
 *  Modified by:
 *  J. Meyer
 *
 *------------------------------------------------------------------
 */

#include "in4073.h"
#include "communication/packet.h"
#include "communication/packet_uav.h"
bool txd_available = true;

void uart_put(uint8_t byte)
{
	NVIC_DisableIRQ(UART0_IRQn);

	if (txd_available) {txd_available = false; NRF_UART0->TXD = byte;}
	else enqueue(&tx_queue, byte);

	NVIC_EnableIRQ(UART0_IRQn);
}

/*
Wrapper function to put packets on the RS232 line
 */
void uart_put_packet(uint8_t *packet_data, size_t packet_size)
{
	for( int i = 0; i < packet_size; i++)
		uart_put(packet_data[i]);
}

// Reroute printf
int _write(int file, const char * p_char, int len)
{
	int i, count = 0;
	packet my_packet;
	for (i = 0; i < len; i++)
	{
		printf_data[count++] = p_char[i];
		if (count == BODY_LENGTH) // Size of packet body
		{
			encode(&my_packet, PACKET_TYPE_PRINTF);
			uart_put_packet((uint8_t *)&my_packet, sizeof(packet));
			count = 0;
		}
		else if (i == len -1) // Packet does not necessarily contain a full packet
		{
			printf_data[count++] = '\0'; // Null terminate to ensure print is consistent
			encode(&my_packet, PACKET_TYPE_PRINTF);
			uart_put_packet((uint8_t *)&my_packet, sizeof(packet));
		}
	}
	return len;
}


void UART0_IRQHandler(void)
{
	if (NRF_UART0->EVENTS_RXDRDY != 0)
	{
		NRF_UART0->EVENTS_RXDRDY  = 0;
		enqueue( &rx_queue, NRF_UART0->RXD);
	}

	if (NRF_UART0->EVENTS_TXDRDY != 0)
	{
		NRF_UART0->EVENTS_TXDRDY = 0;
		if (tx_queue.count) NRF_UART0->TXD = dequeue(&tx_queue);
		else txd_available = true;
	}

	if (NRF_UART0->EVENTS_ERROR != 0)
	{
		NRF_UART0->EVENTS_ERROR = 0;
		printf("uart error: %lu\n", NRF_UART0->ERRORSRC);
	}
}

void uart_init(void)
{
	init_queue(&rx_queue); // Initialize receive queue
	init_queue(&tx_queue); // Initialize transmit queue

	nrf_gpio_cfg_output(TX_PIN_NUMBER);
	nrf_gpio_cfg_input(RX_PIN_NUMBER, NRF_GPIO_PIN_NOPULL);
	NRF_UART0->PSELTXD = TX_PIN_NUMBER;
	NRF_UART0->PSELRXD = RX_PIN_NUMBER;
	NRF_UART0->BAUDRATE        = (UART_BAUDRATE_BAUDRATE_Baud115200 << UART_BAUDRATE_BAUDRATE_Pos);

	NRF_UART0->ENABLE           = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
	NRF_UART0->EVENTS_RXDRDY    = 0;
	NRF_UART0->EVENTS_TXDRDY    = 0;
	NRF_UART0->TASKS_STARTTX    = 1;
	NRF_UART0->TASKS_STARTRX    = 1;

	NRF_UART0->INTENCLR = 0xffffffffUL;
	NRF_UART0->INTENSET = 	(UART_INTENSET_RXDRDY_Set << UART_INTENSET_RXDRDY_Pos) |
                          	(UART_INTENSET_TXDRDY_Set << UART_INTENSET_TXDRDY_Pos) |
                          	(UART_INTENSET_ERROR_Set << UART_INTENSET_ERROR_Pos);

	NVIC_ClearPendingIRQ(UART0_IRQn);
	NVIC_SetPriority(UART0_IRQn, 3); // either 1 or 3, 3 being low. (sd present)
	NVIC_EnableIRQ(UART0_IRQn);
}
