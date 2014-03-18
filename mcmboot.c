/*
 * mcmboot.c
 *
 * Created: 26.03.2013 07:47:19
 *  Author: grandmaster
 */ 
#define F_CPU 8000000UL

#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <util/delay.h>

#define UART_BAUD 19200L

#define UART_DIV ((F_CPU / (16 * UART_BAUD) -1))
#define UART_HIDIV ( UART_DIV >> 8)
#define UART_LODIV ( UART_DIV & 0xff)

#define TICKSTOWAIT ((F_CPU / 100 ) * 1000/1024 )	// wait 100 ms

#define RBUFSIZE 1024
uint8_t rbuf_data[RBUFSIZE];
uint16_t rbuf_wp;
uint16_t rbuf_rp;
uint16_t rbuf_count;

uint8_t lastChar;
uint8_t lastCharValid;
uint8_t lastCharRead;

#define WBUFSIZE 64	// valid sizes have to be a power of 2 (2^x - eg. 2,4,8,...)
#define WBUFMASK (WBUFSIZE-1)
uint8_t wbuf_data[WBUFSIZE];
uint8_t wbuf_wp;
uint8_t wbuf_rp;
uint8_t wbuf_count;

uint8_t doBoot = 0;


enum { IDLE=0, READ, WRITE, PARSE, CHECK, PROG };

uint8_t blState = 0;
uint8_t nextState = 0;

struct {
	uint8_t		bytecount;
	uint16_t	address;
	uint8_t		recordType;
	uint8_t		checksum;
	uint8_t		data_ptr;
	uint8_t		data[256];
} sHexLine;	

struct S_PAGE{
	uint8_t  buffer[SPM_PAGESIZE];
	uint8_t  count;
	uint8_t  startAddress;
	uint16_t current;
	uint8_t  wp;
} sPage;


void boot_program_page (uint32_t page, uint16_t *buf);

int main(void) __attribute__((noreturn));
int main(void)
{
// configure UART and check communication    

	UBRR0H = UART_HIDIV;
	UBRR0L = UART_LODIV;	// ca 1202 Bit / s

	/* Set frame format: 7data, odd parity, 1stop bit */
	UCSR0C = (1 << UPM01) |
			 (1 << UPM00) |
			 (0 << USBS0) |
			 (1 << UCSZ01)|
			 (0 << UCSZ00);

	/* Enable receiver and transmitter
	 * Disable RX interrupt */
	UCSR0B = (0 << RXCIE0) | (1<<RXEN0)|(1<<TXEN0);
	rbuf_wp = 0;
	rbuf_rp = 0;
	rbuf_count = 0;

	sPage.count = 0;
	sPage.current = 0;
	sPage.wp = 0;
	memset(sPage.buffer, 0xff , sizeof sPage.buffer);
	
	blState = IDLE;
#if F_CPU
	TCCR1B = (( 1 << CS12 ) | (1 << CS10));		// Prescaler 1024
	TCNT1  = 0;
	
	// check HUB/PGM input - use PGM to indicate programmer presence
	while(!doBoot)
    {		
		// if UART Tx Buf is empty and there is something to send
		if((UCSR0A & (1<<UDRE0) && wbuf_count)){
			UDR0 = wbuf_data[wbuf_rp];
			wbuf_rp++;
			wbuf_rp &= WBUFMASK; 
			wbuf_count--;			
		}
		
		// Read char from UART if there is one
		if(UCSR0A & (1<<RXC0)){
			if(rbuf_count < RBUFSIZE)
			{
				rbuf_data[rbuf_wp++] = UDR0;
				rbuf_wp = (rbuf_wp == RBUFSIZE) ? 0 : rbuf_wp;
				rbuf_count++;
			}
			else
			{	// Buffer is full, read data anyway, but discard
				volatile uint8_t dummy = UDR0;
			}
		}
		
		switch(blState){
			case IDLE:
				if (getChar() == "m"){
					blState = WAITFORSYNC;
					TCNT1  = 0;
				}
				break;
			case WAITFORSYNC:
				if (lastCharValid){
					if(getChar() == "c"){
						nextState = HEX1;
						blState = GETBYTE;
						putStrP(PSTR("mcMega Bootloader v14_1.\r\nWaiting 10 seconds for Hex File...\r\n"));
						TCNT1  = -(TICKSTOWAIT*100);
					}
					else{
						blState = IDLE;
					}
				}				
				break;
			case GETCHAR:
				if (lastCharValid){
					TCNT1   = 0;					
					blState = nextState;
				}
				break;
			case GETBYTE:
				if (lastCharValid){
					TCNT1   = 0;
					blState = nextState;
				}
				break;
		}
    }
}


//; Hex-File empfangen und im Speicher ablegen (Intel HEX Format)
//;
//; "ESC" ($1B) - Abbruch
//;
//; Intel Hex Object Format
//;
//; This is the default object file format.
//; This format is line oriented and uses only printable ASCII characters except
//; for the carriage return/line feed at the end of each line. The format is
//; symbolically represented as:
//; :NN AAAA RR HH CC CRLF
//;
//; Where:
//;
//; :    - Record Start Character (colon)
//; NN   - Byte Count (2 hex digits)
//; AAAA - Address of first byte (4 hex digits)
//; RR   - Record Type (00 except for last record which is 01)
//; HH   - Data Bytes (a pair of hex digits for each byte of data in the record)
//; CC   - Check Sum (2 hex digits)
//; CRLF - Line Terminator (CR/LF for DOS, LF for LINUX)
//;
//; The last line of the file will be a record conforming to the above format
//; with a byte count of zero.
//;
//; The checksum is defined as:
//;
//; byte_count+address_hi+address_lo+record_type+(sum of all data bytes)+checksum = 0
//;
//

enum {START, BYTECOUNT, ADDRESS, RECORDTYPE, DATA, CHECKSUM, TERMINATE, CHECKERROR, ERROR, BUFFER, BUFFER2, BUFFER4};

uint8_t iHexParser(uint8_t c){
	static uint8_t count=0;
	static uint8_t cbuf[5];
	static uint8_t HexParserState=START;
	static uint8_t nextHexParserState=START;
	
	switch(HexParserState){
		case START:
			if (c==':'){
				sHexLine.address=0;
				sHexLine.bytecount=0;
				sHexLine.checksum=0;
				sHexLine.recordType=0;

				HexParserState = BUFFER2;		// read 2 chars into buffer
				nextHexParserState = BYTECOUNT;	// then continue with bytecount
			}			
		break;
		case BYTECOUNT:
			sHexLine.bytecount = atoi(cbuf);		// parse hex value from buffer
			sHexLine.checksum = sHexLine.bytecount;	// initialize checksum
			HexParserState = BUFFER4;				// read 4 chars into buffer
			nextHexParserState = ADDRESS;			// then parse it as address
		break;
		case ADDRESS:
			sHexLine.address = atoi(cbuf);			// get address value
			sHexLine.checksum += (sHexLine.bytecount & 0xff);	// add to checksum
			sHexLine.checksum += (sHexLine.bytecount >> 8);
													// check if address is outside our current flash page
			if((sHexLine.address & 0xff00 != sPage.current) || !sPage.count){
				if (sPage.count){					// if it is not the first page to be processed
					programPage(&sPage);			// and it is outside, program page buffer to flash					
				}
				sPage.current = sHexLine.address & 0xff00;	// set address of new page
				sPage.startAddress = (uint8_t) sHexLine.address;	
				sPage.wp = (uint8_t) sHexLine.address;
			}			
			nextHexParserState = RECORDTYPE;
			HexParserState = BUFFER2;
		break;
		case RECORDTYPE:
			sHexLine.recordType = atoi(cbuf);
			sHexLine.checksum += atoi(cbuf);

			if(sHexLine.recordType==0){
				if(sHexLine.bytecount){
					sHexLine.data_ptr = 0;
					nextHexParserState = DATA;
				}
				else{
					nextHexParserState = CHECKSUM;					
				}				
				HexParserState = BUFFER2;
			}
			if(sHexLine.recordType==1){
				nextHexParserState = TERMINATE;
				HexParserState = BUFFER2;
			}
		break;
		case DATA:
			HexParserState = BUFFER2;
			sHexLine.data[sHexLine.data_ptr++] = atoi(cbuf);
			sHexLine.checksum += atoi(cbuf);
			if(sHexLine.data_ptr == sHexLine.bytecount){
				nextHexParserState = CHECKSUM;				
			}						
		break;
		case CHECKSUM:
			sHexLine.checksum += atoi(cbuf);
			if(sHexLine.checksum){
				HexParserState = CHECKERROR;
			}
			else{
				// Checksum ok, copy line data to page buffer
				// if page buffer is full, program page
				uint8_t i;
				for(i=0;i<sHexLine.bytecount;i++){
					sPage.buffer[sPage.wp++] = sHexLine.data[i];
					sPage.count++;
					if(sPage.wp == 0){
						programPage(&sPage);
					}
				}				
				HexParserState = START;
			}
		break;
		case TERMINATE:
		break;
		case BUFFER2:
			count = 2;
			memset(cbuf,0,sizeof cbuf);
			HexParserState = BUFFER;
			break;
		case BUFFER4:
			count = 4;
			memset(cbuf,0,sizeof cbuf);
			HexParserState = BUFFER;
			break;
		case BUFFER:
			if(count--){
				cbuf[3]=cbuf[2];
				cbuf[2]=cbuf[1];
				cbuf[1]=cbuf[0];
				cbuf[0]=c;
				if(count==0){
					HexParserState=nextHexParserState;
				}
			}
			else
				HexParserState=ERROR;
		break;
		case CHECKERROR:
		
		break;
		case ERROR:		
		break;
	}		
}



uint8_t getChar( ){
	uint8_t c = 0;
	if(rbuf_count)
	{
		c = rbuf_data[rbuf_rp++];
		rbuf_count--;
		rbuf_rp = (rbuf_rp == RBUFSIZE) ? 0 : rbuf_rp;
	}
	return c;
}	


void putChar(uint8_t c){
	if(wbuf_count < WBUFSIZE){
		wbuf_data[wbuf_wp++] = c;
		wbuf_wp &= WBUFMASK;
		wbuf_count++;
	}	
}


void putStrP(uint8_t * c ){
	while(*c){
		putChar(pgm_read_byte(*c++));
	}
	return 0;
}



void programPage (struct S_PAGE * p)
{
	uint8_t i;
	uint16_t * buf;

	buf = (uint16_t *) p->buffer;
	boot_page_erase (p->current);
	boot_spm_busy_wait ();      // Wait until the memory is erased.

	for (i=0; i<SPM_PAGESIZE; i+=2){
		boot_page_fill (p->current + i, *buf++);
	}

	boot_page_write (p->current);	// Store buffer in flash page.
	boot_spm_busy_wait();			// Wait until the memory is written.

	// Reenable RWW-section again. We need this if we want to jump back
	// to the application after bootloading.
	boot_rww_enable ();
	p->count = 0;
	memset(p->buffer,0xff,sizeof p->buffer);
}
