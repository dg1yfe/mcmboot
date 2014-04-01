/*
 * mcmboot.c
 *
 * Created: 26.03.2013 07:47:19
 *  Author: grandmaster
 */ 
#ifndef F_CPU
#define F_CPU 8000000UL
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


enum { IDLE=0, WAITPROMPT, PROMPT, PROGRAM, VERIFY, READ, READ_DATA, SEND_DATA,
		PROCESSHEXDATA, PROGRAM_PAGE_EOF, VERIFY_PAGE_EOF, PROGRAM_PAGE, VERIFY_PAGE,
		PROG_DONE, VERIFY_DONE, ERASE_PMEM, BOOT };
	
enum { OK=0, PROCESSING, CHECKSUMERROR, FLASHERROR, VERIFYERROR, PROG_PAGE, EOF };
		
enum { IHEX_DATA_RECORD=0, IHEX_EOF_RECORD};

uint8_t mode = PROGRAM;
uint8_t blState = PROMPT;
uint8_t wait = 0;

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

void eraseProgramMemory (void);


uint8_t iHexParser(uint8_t c);
char getChar( );
void putChar(char c);
void putStrP(const char * c );
void putHex(uint8_t u);

void (*start)( void ) = 0x0000;        /* Funktionspointer auf 0x0000 */

int main(void) __attribute__((noreturn));
int main(void)
{
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
	 * Bit 4	: Signalling Decode (IC1)
	 * Bit 5	: Clock
	 * Bit 6	: Data out
	 * Bit 7	: DPTT
	 */
	// leave Bit 6 low & input
	// only use PORTE2 as data i/o
	DDRD  = ( 1 << 5) | ( 0 << 6 ) | ( 1 << 7 );
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
	
	blState = PROMPT;
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
				volatile uint8_t dummy = UDR0;
			}
		}
		
		switch(blState){
			case WAITPROMPT:
				if(--wait)
				{
					_delay_ms(1);
					break;
				}				
			case PROMPT:
				blState = IDLE;
				putStrP(PSTR("\r\nmcMega Bootloader v14_1 (dg1yfe / 2014).\r\nR - Erase\r\np - Program\r\nv - Verify\r\nb - Boot\r\n"));
				break;				
			case IDLE:{
				char c;
				if((c = getChar())){
					if( c == 'p'){
						blState = PROGRAM;						
					}
					else
					if( c == 'v'){
						blState = VERIFY;
					}
					else
					if( c == 'R'){
						putStrP(PSTR("\r\nErasing Program Memory...\r\n"));
						blState = ERASE_PMEM;
					}
					else
					if (c == 'r'){
						blState = READ;
					}
					if( (c == 'p') || (c == 'v') ){
						putStrP(PSTR("Send Hex File... (ESC to abort)\r\n"));
					}						
				}
				break;
			}			
			case PROGRAM:
				mode = PROGRAM;
				blState=PROCESSHEXDATA;
				break;
			case VERIFY:
				mode = VERIFY;
				blState=PROCESSHEXDATA;
				break;
			case PROCESSHEXDATA:
			{
				char c;
				if((c = getChar())){
					uint8_t err;
					if(c==0x1b){
						putStrP(PSTR("ESC\r\n"));
						blState = PROMPT;
					}
					err = iHexParser(c);
					
					if(err == PROG_PAGE){
						if(mode==VERIFY)						
							blState = VERIFY_PAGE;
						else
							blState = PROGRAM_PAGE;
					}
					else
					if(err==EOF){
						if (sPage.wp){
							if(mode==VERIFY)
								blState = VERIFY_PAGE_EOF;
							else
								blState = PROGRAM_PAGE_EOF;
						}
						else{
							blState = PROG_DONE;						
						}
					}
				}
				break;
			}
			case PROGRAM_PAGE_EOF:			
			case PROGRAM_PAGE:
				if (boot_spm_busy())
					break;
					
				programPageBegin(&sPage);
				
				if(blState == PROGRAM_PAGE)
					blState = VERIFY_PAGE;
				else
					blState = VERIFY_PAGE_EOF;
				break;
			case VERIFY_PAGE_EOF:
			case VERIFY_PAGE:
				if (boot_spm_busy())
					break;
				errorAtAddress = 0;
				
				if(programPageVerify(&sPage) != OK){
					putStrP(PSTR("\r\nVerify error at address 0x"));
					memset(cBuf,0,sizeof cBuf);
					putHex((uint8_t) (sPage.current >> 8));
					putHex( (uint8_t) sPage.current);
					putStrP(PSTR(".\r\n"));
					wait=100;
					blState = WAITPROMPT;
					break;
				}
				else
				{					
					putStrP(PSTR("0x"));
					memset(cBuf,0,sizeof cBuf);
					putHex((uint8_t) (sPage.current >> 8));
					putHex( (uint8_t) sPage.current);
					putStrP(PSTR(" Ok\r\n"));
				}
				
				if(blState==VERIFY_PAGE_EOF)
				{
					blState = PROG_DONE;
				}
				else{
					blState = PROCESSHEXDATA;					
				}
				break;
			case PROG_DONE:
				putStrP(PSTR("\r\nProg & Verify ok.\r\n"));
				wait=100;
				blState = WAITPROMPT;
				break;
			case VERIFY_DONE:
				putStrP(PSTR("\r\nData successfully verified.\r\n"));
				wait=100;
				blState = WAITPROMPT;
				break;
			case READ:
				sHexLine.address=0;
				sHexLine.recordType = IHEX_DATA_RECORD;
				break;
			case READ_DATA:
#define HEXLINEBYTECOUNT 16
#define READ_ADDR_MAX 0xf000
				if(wbuf_count)	// wait until write buffer is empty
					break;
				if(sHexLine.address <= READ_ADDR_MAX )
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
					putChar(':');

					putHex(sHexLine.bytecount);
					putHex((uint8_t) (sHexLine.address >> 8));
					putHex((uint8_t) sHexLine.address);
					putChar('0');
					putChar('0');
					for(i=0;i < sHexLine.bytecount;i++){
						putHex(sHexLine.data[i]);
					}
					putHex(sHexLine.checksum);
					putChar('\r');
					putChar('\n');
				}
				else{
					wait=100;
					blState = WAITPROMPT;
				}
				break;
			case SEND_DATA:
				break;
			case ERASE_PMEM:
				eraseProgramMemory();
				putStrP(PSTR("...done\r\n"));
				wait=50;
				blState=WAITPROMPT;
				break;
			case BOOT:
				doBoot = 1;
				break;
		}
    }
	start();
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
	c = (char) (u > 9 ? u + 'a' - 10 : u+'0');
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
	for (i=0; i<SPM_PAGESIZE; i+=2){
		boot_page_fill (p->current + i, *buf++);
	}
	boot_page_write (p->current);	// Store buffer in flash page.
	
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
	boot_page_erase_safe (p->current);	
}

void eraseProgramMemory ()
{
	uint32_t page;
	for (page=0; page < BOOTLOADER_BYTE_ADDRESS; page+=SPM_PAGESIZE){
		boot_page_erase_safe(page);
	}
	boot_spm_busy_wait();
	boot_rww_enable ();
}
	
