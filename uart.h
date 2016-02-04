//uart.h
void uart_init(void);
unsigned char uart_send_data(unsigned char *p, unsigned int datalong);
unsigned char uart_send_single_data(unsigned char send_data);
unsigned char uart_send_string_data(unsigned char *p, unsigned int datalong);


