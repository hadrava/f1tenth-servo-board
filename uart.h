#ifndef _UART_H_
#define _UART_H_


void uart_init(void);
void uart_deinit(void);


#define UART_INPUT_WAIT_FOR_READY() do {} while (!(UCSR0A & (1<<RXC0)))
#define UART_INPUT_READY (UCSR0A & (1<<RXC0))


#define UART_OUTPUT_WAIT_FOR_READY() do {} while (!(UCSR0A & (1<<UDRE0)))
#define UART_OUTPUT_READY (UCSR0A & (1<<UDRE0))

#endif
