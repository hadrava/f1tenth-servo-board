#include "global.h"
#include <avr/io.h>
#include "uart.h"

#ifndef BAUD_RATE
#define BAUD_RATE	115200L
#endif

#define UBRR_U2X(baud)	(F_CPU / (8 * (baud) - 1))

void uart_init(void) {
	UBRR0 = UBRR_U2X(BAUD_RATE) - 1;
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // Initial value - Asynchronous, No parity, 1 stop bit, 8-bit
	UCSR0A = _BV(U2X0);                 // U2X0 - double speed (better rounding for our baudrate)
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);   // RXEN0 / TXEN0 - Receiver/Transmitter Enable, TODO-interrupts: RXCIE0, TXCIE0, UDRIE0
}

void uart_deinit(void) {
	UCSR0B = 0;                         // Initial value
	UCSR0A = 0;                         // Almost Initial value (UDRE0 is one by hardware)
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // Initial value - Asynchronous, No parity, 1 stop bit, 8-bit
	UBRR0 = 0;                          // Initial value
}
