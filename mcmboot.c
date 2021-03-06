/*

Copyright (c) 2016, Felix Erckenbrecht
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */ 
#ifndef F_CPU
#define F_CPU 8000000UL
#warning "F_CPU not defined, setting to 8 MHz"
#endif

#define BOOTLOADER_BYTE_ADDRESS 0xF000U

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

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

uint8_t doVerify = 0;

#define WBUFSIZE 128	// valid sizes have to be a power of 2 (2^x - eg. 2,4,8,...)
#define WBUFMASK (WBUFSIZE-1)
uint8_t wbuf_data[WBUFSIZE];
uint8_t wbuf_wp;
uint8_t wbuf_rp;
uint8_t wbuf_count;

uint16_t errorAtAddress = 0;

uint8_t doBoot = 0;


enum { BL_IDLE=0, BL_WAITPROMPT, BL_PROMPT, BL_PROGRAM, BL_VERIFY, BL_READ, BL_READ_DATA, BL_SEND_DATA,
		BL_PROCESSHEXDATA, BL_ERASE_PAGE, BL_PROGRAM_PAGE, BL_VERIFY_PAGE,
		BL_PROG_DONE, BL_ERASE_PMEM, BL_DO_ERASE_PMEM, BL_BOOT, BL_IGNORELINE };
	
enum { OK=0, PROCESSING, CHECKSUMERROR, FLASHERROR, VERIFYERROR, PROG_PAGE, EOF };
		
enum { IHEX_DATA_RECORD=0, IHEX_EOF_RECORD};

uint8_t mode = BL_PROGRAM;
uint8_t eof = 0;
uint8_t blState = BL_PROMPT;
uint8_t wait = 0;
int16_t noprompt = 0;

struct {
	uint8_t		bytecount;
	uint8_t		offset;
	uint16_t	address;
	uint8_t		recordType;
	uint8_t		checksum;
	uint8_t		data_ptr;
	uint8_t		data[256];
} sHexLine;	

struct S_PAGE{
	uint8_t  buffer[SPM_PAGESIZE];
	uint8_t  count;
//	uint8_t  startAddress;
	uint32_t current;
	uint8_t  wp;
} sPage;
/*

	PGM Pin checken, wenn nicht aktiv, dann booten
	sonst:
	Prompt ausgeben (p/e/v)
	Auf HEX File warten

*/

void programPageBegin(struct S_PAGE * p);
uint8_t programPageVerify (struct S_PAGE * p);
void erasePage (struct S_PAGE * p);

void eraseProgramMemory (void);

uint8_t iHexParser(uint8_t c);
char getChar( );
void putChar(char c);
void putStrP(const char * c );
void putHex(uint8_t u);
void resetWatchdog();

int main(void) __attribute__((OS_main));
int main(void)
{
	/* Pointer to main program reset vector at 0x0000 */
	void (*start)( void ) = 0;
	char cBuf[8];

// configure io Pins
	/*  Port A:
	 *  Bit 0-2 : PL Encode
	 *  Bit 3	: F1 / #F2  (Input from Control Panel, parallel to SCI RX)
	 *  Bit 4-5 : n.c.
	 *  Bit 6	: HUB
	 *  Bit 7	: Test
	 */
	DDRA  = (0 << 3) | (1 << 2) | (1 << 1) | (1 << 0);
	PORTA = (1 << 2);

	/* Port B:
	 * Bit 0	: CSQ/UNSQ (Input from Control Panel)
	 * Bit 1-3  : SPI (SCK,MOSI,MISO)
	 * Bit 4	: Alert Tone (OC0) (default = 0 -> used by HW as programmable Pull-Up for /NMI (INT4) input)
	 * Bit 5	: Data Inhibit (clamp Signal Decode Input to GND)
	 * Bit 6	: Tx/Busy (Control Head Reset)
	 * Bit 7	: Syn_Latch
	 */
	DDRB  = (1 << 4) | ( 1 << 5 ) | ( 0 << 6) | ( 1 << 7 );
	PORTB = (0 << 4);

	/* Port C:
	 * Bit 0 - 3 : SEL_ENC (4-Bit DAC / Sel Call)
	 * Bit 4 - 7 : n.c.
	 */
	DDRC  = 0x0f;
	PORTC = 3;

	/*
	 * Port D:
	 * Bit 0	: SWB+
	 * Bit 1	: PTT (input)
	 * Bit 2-3	: UART1
	 * Bit 4	: Signaling Decode (IC1)
	 * Bit 5	: Clock
	 * Bit 6	: Data out
	 * Bit 7	: DPTT
	 */
	// Configure Data, Clock and DPTT as outputs
	// 
	DDRD  = ( 1 << 5) | ( 1 << 6 ) | ( 1 << 7 );
	PORTD = ( 0 << 5) | ( 0 << 6);

	/*
	 * Port E:
	 * Bit 0-1	: UART0 (SCI to Control Head)
	 * Bit 2	: Data In
	 * Bit 3	: Call LED
	 * Bit 4	: #NMI  (INT4)
	 * Bit 5	: #STBY (INT5)
	 * Bit 6	: Lock Detect (INT6)    ( 1 = Lock )
	 * Bit 7	: Squelch Detect (INT7) ( 1 = Signal)
	 */

	DDRE  = ( 1 << 3 ) | ( 1 << 0);
	PORTE = ( 1 << 0);

	/*
	 * Port G:
	 * Bit 0	: Call SW
	 * Bit 1	: Emergency
	 * Bit 2	: n.c.
	 * Bit 3	: SR Latch
	 * Bit 4	: #Mem enable
	 */

	PORTG = (1 << 4);
	DDRG  = (1 << 3) | (1 << 4);

	resetWatchdog();
	
// configure UART and check communication    

	UBRR0H = UART_HIDIV;
	UBRR0L = UART_LODIV;	// ca 1202 Bit / s

	/* Set frame format: 8 data, no parity, 1 stop bit */
	UCSR0C = (0 << UPM01) |
			 (0 << UPM00) |
			 (0 << USBS0) |
			 (1 << UCSZ01)|
			 (1 << UCSZ00);

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
	
	blState = BL_PROMPT;
//#if F_CPU
	TCCR1B = (( 1 << CS12 ) | (1 << CS10));		// Prescaler 1024
	TCNT1  = 0;
	
	// check HUB/PGM input - use PGM to indicate programmer presence
	// HUB/PGM = GND -> indicate HUB
	// HUB/PGM = +5V -> indicate Programming request
	// "/NMI" is used with original CPU to indicate programming request
	// "/NMI" is mapped to Port E4 (INT 4)
	
	doBoot = PINE & (1 << PINE4);	// read /PGM input
	while(!doBoot)
    {		
		doBoot = PINE & (1 << PINE4);	// read /PGM input
	
		resetWatchdog();

		// if UART Tx Buf is empty and there is something to send
		if((UCSR0A & (1<<UDRE0)) && wbuf_count){
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
				// mark dummy as unused to avoid issuing a compiler warning
				volatile uint8_t dummy __attribute__((unused));
				dummy = UDR0;
			}
		}
		
		switch(blState){
			case BL_WAITPROMPT:
			{
				uint8_t i;
				if(wbuf_count)	// wait until write buffer is empty
					break;
				for(i=0;i<50;i++){
					_delay_ms(10);
					resetWatchdog();
				}
				noprompt=0;
			}			
			//no break;
			case BL_PROMPT:
				blState = BL_IDLE;
				// try to avoid multiple consecutive prompts which could overflow the TX ringbuffer
				if(!noprompt)
					putStrP(PSTR("\r\n: mcMega Bootloader v14_1 (dg1yfe / 2014).\r\n: X - Erase\r\n: p - Program\r\n: v - Verify\r\n: r - Read\r\n: b - Boot\r\n"));
				noprompt = -1;
				break;				
			case BL_IDLE:{
				char c;
				if(noprompt)
					noprompt--;
				if((c = getChar())){
					if( c == ':'){
						blState = BL_IGNORELINE;
						break;
					}
					if( c == '\r'){
						blState = BL_PROMPT;
						break;
					}
					
					if( c == 'p'){
						blState = BL_PROGRAM;
					}
					else
					if( c == 'v'){
						blState = BL_VERIFY;
					}
					else
					if( c == 'X'){
						blState = BL_ERASE_PMEM;
					}
					else
					if (c == 'r'){
						blState = BL_READ;
					}
					else
					if (c == 'b'){
						blState = BL_BOOT;
					}
					
					if( (c == 'p') || (c == 'v') ){
						eof = 0;
						putStrP(PSTR("Send Hex File... (ESC to abort)\r\n"));
					}						
				}
				break;
			}			
			case BL_PROGRAM:
				mode = BL_PROGRAM;
				blState=BL_PROCESSHEXDATA;
				break;
			case BL_VERIFY:
				mode = BL_VERIFY;
				blState=BL_PROCESSHEXDATA;
				break;
			case BL_PROCESSHEXDATA:
			{
				char c;
				if((c = getChar())){
					uint8_t err;
					// check for ESCAPE
					if(c==0x1b){
						putStrP(PSTR("ESC\r\n"));
						eof=2;
						// abort
						blState = BL_PROG_DONE;
						// no break here, Hex Parser needs to abort as well
					}
					// send char to iHex Parser
					err = iHexParser(c);
					
					// evaluate return state
					if(err==EOF || err == CHECKSUMERROR){
						// end of file received (or line checksum error)
						eof = 1;
						// treat line checksum error and EOF the same way
						if(err == CHECKSUMERROR){
							eof++;
							// give error message on checksum error
							putStrP(PSTR("Line Checksum Error near address: 0x"));
							putHex(sHexLine.address >> 8);
							putHex(sHexLine.address & 0xff);						
						}
						// program last page
						if (sPage.wp){
							blState = BL_ERASE_PAGE;
						}
						else{
							blState = BL_PROG_DONE;
						}
					}
					
					if(err == PROG_PAGE){
						if(mode==BL_VERIFY)
							blState = BL_VERIFY_PAGE;
						else
							blState = BL_ERASE_PAGE;
					}
				}
				break;
			}
			case BL_ERASE_PAGE:
				if (boot_spm_busy())
					break;
				erasePage(&sPage);
				blState = BL_PROGRAM_PAGE;
				break;
			case BL_PROGRAM_PAGE:
				if (boot_spm_busy())
					break;
					
				programPageBegin(&sPage);
				
				blState = BL_VERIFY_PAGE;
				break;
			case BL_VERIFY_PAGE:
				if (boot_spm_busy())
					break;
				putStrP(PSTR("\r\n0x"));
				putHex((uint8_t) (sPage.current >> 8));
				putHex( (uint8_t) sPage.current);
				
				if(programPageVerify(&sPage) != OK){
					memset(cBuf,0,sizeof cBuf);
					putStrP(PSTR(" - Verify error."));
					blState = BL_WAITPROMPT;
					break;
				}
				else
				{					
					memset(cBuf,0,sizeof cBuf);
					putStrP(PSTR(" - Ok"));
				}
				
				if(eof)
				{
					blState = BL_PROG_DONE;
				}
				else{
					blState = BL_PROCESSHEXDATA;
				}
				break;
			case BL_PROG_DONE:
				putStrP(PSTR("\r\nProg / Verify "));
				// check for abort
				if(eof == 2)
					putStrP(PSTR("aborted.\r\n"));
				else
					putStrP(PSTR("ok.\r\n"));
				blState = BL_WAITPROMPT;
				break;
			case BL_READ:
				sHexLine.address=0x0000;
				sHexLine.recordType = IHEX_DATA_RECORD;
				blState = BL_READ_DATA;
				break;
			case BL_READ_DATA:
#define HEXLINEBYTECOUNT 16
#define READ_ADDR_MAX 0xf000
				if(getChar() == 0x1b){
					putStrP(PSTR("ESC\r\n"));
					// abort
					blState = BL_WAITPROMPT;
				}				

				if(wbuf_count)	// wait until write buffer is empty
					break;
					
				if(sHexLine.address < READ_ADDR_MAX )
				{
					uint8_t i;
					sHexLine.checksum = 0;
					sHexLine.checksum-= (uint8_t) (sHexLine.address >> 8);
					sHexLine.checksum-= (uint8_t) sHexLine.address & 0xff;
					sHexLine.bytecount = 0;
					for(i=0;i<HEXLINEBYTECOUNT;i++){
						if(sHexLine.address + i >= READ_ADDR_MAX)
							break;
						sHexLine.data[i] = pgm_read_byte(sHexLine.address + i);
						sHexLine.checksum -= sHexLine.data[i];
						sHexLine.bytecount++;
					}
					sHexLine.checksum -= sHexLine.bytecount;
					putChar(':');	// start of line
					putHex(sHexLine.bytecount);	// followed by bytecount (number of bytes in data part of line)
					putHex((uint8_t) (sHexLine.address >> 8));	// followed by 2 Byte address
					putHex((uint8_t) sHexLine.address);

					sHexLine.address += i;

					putChar('0');	// Followed by record type - "00" = This is a data record
					putChar('0');
					for(i=0;i < sHexLine.bytecount;i++){	// write actual data
						putHex(sHexLine.data[i]);
					}
					putHex(sHexLine.checksum);	// last item is the line checksum
					putChar('\r');
					putChar('\n');
				}
				else{
					putStrP(PSTR(":00000001FF"));	// End Of File record
					blState = BL_WAITPROMPT;
				}
				break;
			case BL_SEND_DATA:
				break;
			case BL_ERASE_PMEM:
				putStrP(PSTR("Erasing program memory...\r\n"));
				blState = BL_DO_ERASE_PMEM;
				break;
			case BL_DO_ERASE_PMEM:
				if(wbuf_count)	// wait until write buffer is empty
					break;
				eraseProgramMemory();
				putStrP(PSTR("...done.\r\n"));
				blState=BL_WAITPROMPT;
				break;
			case BL_BOOT:
				doBoot = 1;
				break;
			case BL_IGNORELINE:{
				uint8_t c;
				// ignore all chars until LF or CR is received
				if((c = getChar()) && ( c == '\n' || c == '\r')){
					// abort
					blState = BL_IDLE;
				}
				break;
			}						
		}
    }
	start();
	for(;;);
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

enum {START, BYTECOUNT, ADDRESS, RECORDTYPE, DATA, DATA_TO_PAGE, CHECKSUM, TERMINATE, ERROR, BUFFER, BUFFER2, BUFFER4};

uint8_t iHexParser(uint8_t c){
	static uint8_t count=0;
	static uint8_t cbuf[6];
	static uint8_t HexParserState=START;
	static uint8_t nextHexParserState=START;
	uint8_t rval = PROCESSING;
	uint8_t i;
		
	if(c == 0x1b)
	{
		HexParserState=START;
		return PROCESSING;
	}
	
	switch(HexParserState){
		case BUFFER2:
		case BUFFER4:
			if (HexParserState == BUFFER2)
				count = 2;
			else
				count = 4;
				memset(cbuf,'0',sizeof cbuf);
				HexParserState = BUFFER;
				// no break
		case BUFFER:
			if(count){
				cbuf[sizeof cbuf - count]=c;
				if(--count==0){
					HexParserState=nextHexParserState;
				}
			}
			else
				HexParserState=ERROR;
			break;
		default:
			break;		
	}
	
	switch(HexParserState){
		case START:
			if (c==':'){
				sHexLine.address=0;
				sHexLine.bytecount=0;
				sHexLine.offset=0;
				sHexLine.checksum=0;
				sHexLine.recordType=0;

				HexParserState = BUFFER2;		// read 2 chars into buffer
				nextHexParserState = BYTECOUNT;	// then continue with bytecount
			}			
		break;
		case BYTECOUNT:
			sHexLine.bytecount = (uint8_t) strtoul((const char *)cbuf,0,16);		// parse hex value from buffer
			sHexLine.checksum = sHexLine.bytecount;	// initialize checksum
			HexParserState = BUFFER4;				// read 4 chars into buffer
			nextHexParserState = ADDRESS;			// then parse it as address
		break;
		case ADDRESS:
			sHexLine.address = (uint16_t) strtoul((const char *)cbuf,0,16);			// get address value
			sHexLine.checksum += (sHexLine.address & 0xff);	// add to checksum
			sHexLine.checksum += (sHexLine.address >> 8);
													// check if address is outside our current flash page
			if(((sHexLine.address & 0xff00) != sPage.current) || !sPage.count){
				if (sPage.count){					// if it is not the first page to be processed
													// and it is outside, program page buffer to flash
					rval = PROG_PAGE;
				}
			}			
			nextHexParserState = RECORDTYPE;
			HexParserState = BUFFER2;
		break;
		case RECORDTYPE:
			sHexLine.recordType = (uint8_t) strtoul((const char *)cbuf,0,16);
			sHexLine.checksum += sHexLine.recordType;

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
			sHexLine.data[sHexLine.data_ptr] = (uint8_t) strtoul((const char *)cbuf,0,16);
			sHexLine.checksum += sHexLine.data[sHexLine.data_ptr++];
			if(sHexLine.data_ptr == sHexLine.bytecount){
				nextHexParserState = CHECKSUM;				
			}						
			break;
		case CHECKSUM:
			sHexLine.checksum += (uint8_t) strtoul((const char *)cbuf,0,16);
			if(sHexLine.checksum){
				HexParserState = START;
				rval = CHECKSUMERROR;
				break;
			}
			else{
				// Checksum ok, copy line data to page buffer
				// if page buffer is full, program page
				sPage.current = sHexLine.address & 0xff00;	// set address of new page
				sPage.wp = (uint8_t) sHexLine.address;
				// continue in state DATA_TO_PAGE
			}
			// no break
		case DATA_TO_PAGE:
			i=sHexLine.offset;
			while(i<sHexLine.bytecount && ((sPage.wp) || (!sPage.count))){
				sPage.buffer[sPage.wp++] = sHexLine.data[i];
				i++;
			}
			if(!sPage.wp){
				sPage.count++;
				sHexLine.offset = i;
				rval = PROG_PAGE;				// program page
			}
			if(i<sHexLine.bytecount){
				HexParserState = DATA_TO_PAGE;	// return to this state to copy remaining bytes
			}
			else{
				HexParserState = START;
			}
			break;
		case TERMINATE:
			sHexLine.checksum += (uint8_t) strtoul((const char *)cbuf,0,16);
			if(sHexLine.checksum){
				rval = CHECKSUMERROR;
			}
			else{
				rval = EOF;
			}
			HexParserState = START;
			break;
		case ERROR:		
			break;
		default:
			break;
	}
	return rval;
}



char getChar( ){
	uint8_t c = 0;
	if(rbuf_count)
	{
		c = rbuf_data[rbuf_rp++];
		rbuf_count--;
		rbuf_rp = (rbuf_rp == RBUFSIZE) ? 0 : rbuf_rp;
	}
	return c;
}	


void putChar(char c){
	if(wbuf_count < WBUFSIZE){
		wbuf_data[wbuf_wp++] = c;
		wbuf_wp &= WBUFMASK;
		wbuf_count++;
	}	
}


void putStrP(const char * s ){
	char c;
	while((c = pgm_read_byte(s++))){
		putChar(c);
	}
}

void putHex(uint8_t u){
	char c;
	if(wbuf_count >= WBUFSIZE-1)
		return;

	c = (char) ((u >> 4) > 9 ? (u >> 4) + 'a' - 10 : (u >> 4)+'0');
	wbuf_data[wbuf_wp++] = c;
	wbuf_wp &= WBUFMASK;
	wbuf_count++;
	u &= 0xf;
	c = (char) ( u > 9 ? u + 'a' - 10 : u+'0');
	wbuf_data[wbuf_wp++] = c;
	wbuf_wp &= WBUFMASK;
	wbuf_count++;
}


void programPageBegin(struct S_PAGE * p)
{
	uint16_t i;
	uint16_t * buf;

	buf = (uint16_t *) p->buffer;
//	boot_page_erase (p->current);
//	resetWatchdog();
	for (i=0; i<SPM_PAGESIZE; i+=2){
		boot_page_fill (p->current + i, *buf++);
	}
	resetWatchdog();
	boot_page_write (p->current);	// Store buffer in flash page.
	resetWatchdog();
	
}


uint8_t programPageVerify(struct S_PAGE * p)
{
	uint16_t i;
	uint16_t * buf;
	// Reenable RWW-section again. 
	// Verify would fail otherwise
	boot_rww_enable ();
	p->count = 0;

	// Verify memory content
	buf = (uint16_t *) p->buffer;	
	for (i=0; i<SPM_PAGESIZE; i+=2){
		uint16_t b;
		b = pgm_read_word(p->current + i);
		if(b != *buf++)
			break;
	}

	if(i<SPM_PAGESIZE)
		return VERIFYERROR;
	else
		return OK;
}


void erasePage (struct S_PAGE * p)
{
	resetWatchdog();
	boot_page_erase (p->current);
	resetWatchdog();
}

void eraseProgramMemory ()
{
	uint32_t page;
	for (page=0; page < BOOTLOADER_BYTE_ADDRESS; page+=SPM_PAGESIZE){
// This is	"boot_page_erase_safe(page);" but includes Watchdog reset
		do
		{
			resetWatchdog();
		}while(boot_spm_busy() || (!eeprom_is_ready()));
		boot_page_erase (page);		
	}
	
	do
	{
		resetWatchdog();
	}while(boot_spm_busy());
	boot_rww_enable ();
}
	

void resetWatchdog()
{
	// toggle PortD, Bit 6
	PORTD ^= (1 << 6);	
}
