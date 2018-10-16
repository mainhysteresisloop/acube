/*
 * UARTSI.c
 *
 * Created: 01.05.2016 14:09:24
 * Author : USER
 */ 
#include <avr/io.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <string.h>
#include "UARTSI.h"

#define TX_BUF_MASK (TX_BUF_SIZE-1)
#define RX_BUF_MASK (RX_BUF_SIZE-1)

static volatile char rx_buffer[RX_BUF_SIZE]; 
static volatile uint8_t idxRXIN, idxRXOUT;

static volatile char tx_buffer[TX_BUF_SIZE]; 
static volatile uint8_t idxTXIN, idxTXOUT;

//----------------------------------------------------------------------------------
// INIT
//----------------------------------------------------------------------------------
/*init RX and TX. 9600 bod on 1Mhz freq(!) */
void UARTInitRXTX() {
	
	// FCPU = 1000000 U2X = 1 BAUD = 9600 
	UBRR0H  = 0;
	UBRR0L = 12;
	UCSR0A |= _BV(U2X0);
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);				  // 8-bit data
	
	UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);   // Enable RX and TX and USART Receive Complete interrupt
}
//----------------------------------------------------------------------------------
/*init RX and TX. 4800 bod on 1Mhz freq(!) */
void UARTInitRXTX4800() {
	
	// FCPU = 1000000 U2X = 0 BAUD = 4800 
	UBRR0H  = 0;
	UBRR0L = 12;
//	UCSR0A |= _BV(U2X0);
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);				  // 8-bit data
	
	UCSR0B = _BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0);   // Enable RX and TX and USART Receive Complete interrupt
}
//----------------------------------------------------------------------------------
/*init TX only. 9600 bod on 1Mhz freq(!) */
void UARTInitTX() {
	
	// FCPU = 1000000 BAUD = 9600 U2X  = 1
	UBRR0H  = 0;
	UBRR0L = 12;
	UCSR0A |= _BV(U2X0);
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // 8-bit data
	
	UCSR0B = _BV(TXEN0);   // Enable TX 

}
//----------------------------------------------------------------------------------
void UARTDisableRX() {
	UCSR0B &= ~(_BV(RXEN0) | _BV(RXCIE0));   // Disable RX and TX and USART Receive Complete interrupt	
}
//----------------------------------------------------------------------------------
void UARTEnableRX() {
	UCSR0B |= _BV(RXEN0) | _BV(RXCIE0);      // Enable RX and TX and USART Receive Complete interrupt
}

//----------------------------------------------------------------------------------
//ISR
//----------------------------------------------------------------------------------
ISR( USART_RX_vect ) {
	rx_buffer[idxRXIN++] =  UDR0;
	idxRXIN &= RX_BUF_MASK;
}
//----------------------------------------------------------------------------------
ISR( USART_UDRE_vect ) {
	if( idxTXIN == idxTXOUT ) {
		//���� ������ � fifo ������ ���, �� ��������� ��� ����������
		UCSR0B &= ~_BV(UDRIE0);
	} else {
		UDR0 = tx_buffer[idxTXOUT++];
		idxTXOUT &= TX_BUF_MASK;
	}
}

//----------------------------------------------------------------------------------
// RX
//----------------------------------------------------------------------------------
//reading char from ring buffer without index increment
char UARTRXGetChar() {
	return rx_buffer[idxRXOUT];
}
//----------------------------------------------------------------------------------
// moving to next RX out index if buf is not empty
void UARTRXNext() {
	if(idxRXOUT != idxRXIN ) {
		++idxRXOUT;
		idxTXOUT &= TX_BUF_MASK;
	}
}
//----------------------------------------------------------------------------------
//reading char from ring buffer with index increment
char UARTRXGetCharI() {
	register char ch = UARTRXGetChar();
	UARTRXNext();
	return ch;
}
//----------------------------------------------------------------------------------
uint8_t UARTRXIsEmpty() {
	return (idxRXOUT == idxRXIN);
}

//----------------------------------------------------------------------------------
void UARTRXFlash() {
	idxRXOUT = idxRXIN;
}


//----------------------------------------------------------------------------------
// TX
//----------------------------------------------------------------------------------
void UARTPutChar(char ch) {
	
	if(((idxTXIN + 1) & TX_BUF_MASK ) == idxTXOUT) {				  // check if overrun
		while(idxTXIN != idxTXOUT);								      // if yes, wait till the whole buffer transmitted
	}

	tx_buffer[idxTXIN++] = ch;
	idxTXIN &= TX_BUF_MASK;
	UCSR0B |= _BV(UDRIE0);                                            // enable UDRE interrupt

}
//----------------------------------------------------------------------------------
void UARTPrint( const char *str ) {
	while( *str != '\0' ) {
		UARTPutChar(*(str++));
	}
}
//----------------------------------------------------------------------------------
void UARTPrintln( const char *str ) {
	UARTPrint(str);
	UARTPrint("\r\n");
}
//----------------------------------------------------------------------------------
void UARTPrint_P( const char *str ) {
	char ch;
	while (0 != (ch = pgm_read_byte(str++))) {
		tx_buffer[idxTXIN++] =  ch;
		idxTXIN &= TX_BUF_MASK;
		UCSR0B |= (1<<UDRIE0);  // enabling UDR interrupt, UDR generates when UART is ready to send a byte
	}
}
//----------------------------------------------------------------------------------
void UARTPrintln_P( const char *str ) {
	UARTPrint_P(str);
	UARTPrint("\r\n");
}
//----------------------------------------------------------------------------------
void UARTPrintUint(uint8_t ui, uint8_t raddix) {
	char c_str[5];
	itoa(ui, c_str, raddix);
	UARTPrint(c_str);
}
//----------------------------------------------------------------------------------
void UARTPrintInt16(int16_t int16, uint8_t raddix) {
	char c_str[7];
	itoa(int16, c_str, raddix);
	UARTPrint(c_str);
}
//----------------------------------------------------------------------------------
void UARTPrintUintBits(uint8_t ui) {
	for (uint8_t i = 0; i<8; i++) {
		if ((ui << i) & 0x80){
			UARTPutChar('1');
		} else {
			UARTPutChar('0');
		}
	}
}
//----------------------------------------------------------------------------------
/* Print uint8_t with min characters places*/
void UARTPrintUint_M(uint8_t ui, uint8_t raddix, uint8_t min_cp) {
	char c_str[5];
	itoa(ui, c_str, raddix);
	uint8_t str_len = strlen(c_str);
	if (strlen(c_str) < min_cp) {
		min_cp -= str_len;
		while(min_cp--) {
			UARTPrint("0");
		}
	}
	UARTPrint(c_str);
}
//----------------------------------------------------------------------------------
void UARTPrintBit(uint8_t byte_val, const char *bit_name, uint8_t bit_num ) {
	UARTPrint(bit_name);
	UARTPutChar('=');
	if (byte_val & _BV(bit_num)) {
		UARTPrint("1");
	} else {
		UARTPrint("0");
	}
}








