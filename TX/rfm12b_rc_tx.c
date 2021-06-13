/********************************************************************/ 
/* 433 MHz Radio Control for battery powered large scale model loco */
/*                    TRX: RFM12B                                   */
/*                    Display OLED SSD1306 64x32                    */
/*                    MCU: ATMEL AVR ATmega328p, 8 MHz              */
/*                                                                  */
/*  Compiler:         GCC (GNU AVR C-Compiler)                      */
/*  Author:           Peter Rachow (DK7IH)                          */
/*  Last change:      MAY 2021                                      */
/********************************************************************/
//                       TRANSMITTER module                         //

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/twi.h>

//CPU
#ifdef F_CPU
#undef F_CPU 
#endif
#define F_CPU 8000000

//Data transfer via RFM12B
#define TWAIT 100 //Wait Microceonds for SPI

//Output lines
//TRX port & lines (PORTD)
//Output lines
//TRX port & lines (PORTD)
#define NSEL (1 << PD2)  //YELLOW
#define SCK  (1 << PD3)  //BLUE
#define SDI  (1 << PD4)  //VIOLET
#define SDO  (1 << PD5)
#define FSK  (1 << PD6) //BROWN

#define UARTDELAY 5 //Delay in ms for transmission of single char

//Input line
//ADC 0 for speet potentiometer

//////////////////////////////////////
//   L   C   D   
//////////////////////////////////////

// Font 5x8 for OLED
const char fontchar[485] = {
0x00,0x00,0x00,0x00,0x00, // 20 space ASCII table for NOKIA LCD: 96 rows * 5 bytes= 480 bytes
0x00,0x00,0x5f,0x00,0x00, // 21 ! Note that this is the same set of codes for character you
0x00,0x07,0x00,0x07,0x00, // 22 " would find on a HD44780 based character LCD.
0x14,0x7f,0x14,0x7f,0x14, // 23 # Also, given the size of the LCD (84 pixels by 48 pixels),
0x24,0x2a,0x7f,0x2a,0x12, // 24 $ the maximum number of characters per row is only 14.
0x23,0x13,0x08,0x64,0x62, // 25 %
0x36,0x49,0x55,0x22,0x50, // 26 &
0x00,0x05,0x03,0x00,0x00, // 27 '
0x00,0x1c,0x22,0x41,0x00, // 28 (
0x00,0x41,0x22,0x1c,0x00, // 29 )
0x14,0x08,0x3e,0x08,0x14, // 2a *
0x08,0x08,0x3e,0x08,0x08, // 2b +
0x00,0x50,0x30,0x00,0x00, // 2c ,
0x08,0x08,0x08,0x08,0x08, // 2d -
0x00,0x60,0x60,0x00,0x00, // 2e .
0x20,0x10,0x08,0x04,0x02, // 2f /
0x3e,0x51,0x49,0x45,0x3e, // 30 0
0x00,0x42,0x7f,0x40,0x00, // 31 1
0x42,0x61,0x51,0x49,0x46, // 32 2
0x21,0x41,0x45,0x4b,0x31, // 33 3
0x18,0x14,0x12,0x7f,0x10, // 34 4
0x27,0x45,0x45,0x45,0x39, // 35 5
0x3c,0x4a,0x49,0x49,0x30, // 36 6
0x01,0x71,0x09,0x05,0x03, // 37 7
0x36,0x49,0x49,0x49,0x36, // 38 8
0x06,0x49,0x49,0x29,0x1e, // 39 9
0x00,0x36,0x36,0x00,0x00, // 3a :
0x00,0x56,0x36,0x00,0x00, // 3b ;
0x08,0x14,0x22,0x41,0x00, // 3c <
0x14,0x14,0x14,0x14,0x14, // 3d =
0x00,0x41,0x22,0x14,0x08, // 3e >
0x02,0x01,0x51,0x09,0x06, // 3f ?
0x32,0x49,0x79,0x41,0x3e, // 40 @
0x7e,0x11,0x11,0x11,0x7e, // 41 A
0x7f,0x49,0x49,0x49,0x36, // 42 B
0x3e,0x41,0x41,0x41,0x22, // 43 C
0x7f,0x41,0x41,0x22,0x1c, // 44 D
0x7f,0x49,0x49,0x49,0x41, // 45 E
0x7f,0x09,0x09,0x09,0x01, // 46 F
0x3e,0x41,0x49,0x49,0x7a, // 47 G
0x7f,0x08,0x08,0x08,0x7f, // 48 H
0x00,0x41,0x7f,0x41,0x00, // 49 I
0x20,0x40,0x41,0x3f,0x01, // 4a J
0x7f,0x08,0x14,0x22,0x41, // 4b K
0x7f,0x40,0x40,0x40,0x40, // 4c L
0x7f,0x02,0x0c,0x02,0x7f, // 4d M
0x7f,0x04,0x08,0x10,0x7f, // 4e N
0x3e,0x41,0x41,0x41,0x3e, // 4f O
0x7f,0x09,0x09,0x09,0x06, // 50 P
0x3e,0x41,0x51,0x21,0x5e, // 51 Q
0x7f,0x09,0x19,0x29,0x46, // 52 R
0x46,0x49,0x49,0x49,0x31, // 53 S
0x01,0x01,0x7f,0x01,0x01, // 54 T
0x3f,0x40,0x40,0x40,0x3f, // 55 U
0x1f,0x20,0x40,0x20,0x1f, // 56 V
0x3f,0x40,0x38,0x40,0x3f, // 57 W
0x63,0x14,0x08,0x14,0x63, // 58 X
0x07,0x08,0x70,0x08,0x07, // 59 Y
0x61,0x51,0x49,0x45,0x43, // 5a Z
0x00,0x7f,0x41,0x41,0x00, // 5b [
0x02,0x04,0x08,0x10,0x20, // 5c Yen Currency Sign
0x00,0x41,0x41,0x7f,0x00, // 5d ]
0x04,0x02,0x01,0x02,0x04, // 5e ^
0x40,0x40,0x40,0x40,0x40, // 5f _
0x00,0x01,0x02,0x04,0x00, // 60 `
0x20,0x54,0x54,0x54,0x78, // 61 a
0x7f,0x48,0x44,0x44,0x38, // 62 b
0x38,0x44,0x44,0x44,0x20, // 63 c
0x38,0x44,0x44,0x48,0x7f, // 64 d
0x38,0x54,0x54,0x54,0x18, // 65 e
0x08,0x7e,0x09,0x01,0x02, // 66 f
0x0c,0x52,0x52,0x52,0x3e, // 67 g
0x7f,0x08,0x04,0x04,0x78, // 68 h
0x00,0x44,0x7d,0x40,0x00, // 69 i
0x20,0x40,0x44,0x3d,0x00, // 6a j
0x7f,0x10,0x28,0x44,0x00, // 6b k
0x00,0x41,0x7f,0x40,0x00, // 6c l
0x7c,0x04,0x18,0x04,0x78, // 6d m
0x7c,0x08,0x04,0x04,0x78, // 6e n
0x38,0x44,0x44,0x44,0x38, // 6f o
0x7c,0x14,0x14,0x14,0x08, // 70 p
0x08,0x14,0x14,0x18,0x7c, // 71 q
0x7c,0x08,0x04,0x04,0x08, // 72 r
0x48,0x54,0x54,0x54,0x20, // 73 s
0x04,0x3f,0x44,0x40,0x20, // 74 t
0x3c,0x40,0x40,0x20,0x7c, // 75 u
0x1c,0x20,0x40,0x20,0x1c, // 76 v
0x3c,0x40,0x30,0x40,0x3c, // 77 w
0x44,0x28,0x10,0x28,0x44, // 78 x
0x0c,0x50,0x50,0x50,0x3c, // 79 y
0x44,0x64,0x54,0x4c,0x44, // 7a z
0x00,0x08,0x36,0x41,0x00, // 7b <
0x00,0x00,0x7f,0x00,0x00, // 7c |
0x00,0x41,0x36,0x08,0x00, // 7d >
0x10,0x08,0x08,0x10,0x08, // 7e Right Arrow ->
0x78,0x46,0x41,0x46,0x78, // 7f Left Arrow <-
0x00,0x06,0x09,0x09,0x06};  // 80 °

///////////////////////////
//     DECLARATIONS
///////////////////////////
//
//I²C
void TWIInit(void);
void TWIStart(void);
void TWIStop(void);
uint8_t TWIReadACK(void);
uint8_t TWIReadNACK(void);
uint8_t TWIGetStatus(void);

//OLED
void oled_command(int value);
void oled_init(void);
void oled_write_byte(int col, int page, int val);
void oled_cls(void);
void oled_putchar1(int, int, unsigned char, int);
void oled_putstring(int, int, char *, int);
void oled_putnumber(int, int, long, int,  int);
void oled_clear_section(int, int, int);
int xp2(int xp);

//String
void int2asc(int, char*);

//RFM12B
int spi_send_word(unsigned short);
void rfm12b_init(void);
void rfm12b_setbandwidth(unsigned char, unsigned char, unsigned char);
void rfm12b_setfreq(int);
void rfm12b_setbaud(int);
void rfm12b_setpower(unsigned char, unsigned char);

void rfm12b_txchar(unsigned char);
unsigned char rfm12b_rxchar(void);

void send_speed(char);

//UART
void uart_init(void);
void uart_tx_ch(char);
void uart_tx_str(char*);
char calc_checksum(char*);

//MISC
int main(void);
int get_adc(int);

unsigned long ms = 0;
///////////////////////////
//
//         TWI
//
///////////////////////////

void twi_init(void)
{
    //set SCL to 400kHz
    TWSR = 0x00;
    TWBR = 0x0C;
	
    //enable TWI
    TWCR = (1<<TWEN);
}

//Send start signal
void twi_start(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}

//send stop signal
void twi_stop(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

void twi_write(uint8_t u8data)
{
    TWDR = u8data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}

uint8_t TWIGetStatus(void)
{
    uint8_t status;
    //mask status
    status = TWSR & 0xF8;
    return status;
}

////////////////////////////////
//
// OLED commands
//
///////////////////////////////

//Send comand to OLED
void oled_command(int value)
{
   twi_start();
   twi_write(0x78); //Device address
   twi_write(0x00);
   twi_write(value);
   twi_stop();
} 

//Initialize OLED
void oled_init(void)
{
    oled_command(0xae);
	
    oled_command(0xa8); //Multiplex ratio
    oled_command(0x2F);
	
    oled_command(0xd3); //Set display offset
    oled_command(0x00);
    
    oled_command(0x40 + 0x00); //Set display start line
    
    oled_command(0xa1);
    	
	oled_command(0x20); //Adressing mode
    oled_command(0x00); //HOR
	
    oled_command(0xc0);
    oled_command(0xc8);
    
    oled_command(0xda);
    oled_command(0x12);
    
    oled_command(0x81);
    oled_command(0xfF);
    oled_command(0xa4); //Display ON with RAM content
    oled_command(0xa6); //Normal display (Invert display = A7)
    oled_command(0xd5);
    oled_command(0x80);
    oled_command(0x8d);
    oled_command(0x14);
	//oled_command(0x40); //Set display start line
    oled_command(0xAF); //Display ON
   
} 

//Print character in normal size
//Write 8 vertical bits to given col (0..127) and page (0..7)
void oled_write_byte(int col, int page, int val)
{
    int t1;
	
	//Col and line adr for OLED 68x32
	oled_command(0x21); //COL (+32)
	oled_command(col + 32 ); 
	oled_command(col + 32); 
	
	oled_command(0x22); //PAGE (+2)
	oled_command(page + 2); 
	oled_command(page + 2); 
		
    twi_start();
	twi_write(0x78);
	twi_write(0x40); //Data
    for(t1 = 0; t1 < 7; t1++)
    {
        twi_write(val);
    }
	twi_stop();
}

//Clear screen
void oled_cls(void)
{

    int x, y;
	for(x = 0; x < 128; x++)
	{
	    for(y = 0; y < 8; y++)
		{
		    oled_write_byte(x, y, 0);
		}
    }	

}

//Print one character in normal size to OLED
void oled_putchar1(int col, int row, unsigned char ch1, int inv)
{ 
    int p, t1;
    char ch2;
	int c = col;
	    
    p = (5 * ch1) - 160;
    for(t1 = 0; t1 < 5; t1++)
    { 
	    if(!inv)
		{
	        ch2 = fontchar[p + t1];
		}
		else
		{
	        ch2 = ~fontchar[p + t1];
		}
		
        oled_write_byte(c++, row, ch2);
    }
	
	if(!inv)
	{
	    oled_write_byte(c, row, 0x00);
	}
	else
	{
	    oled_write_byte(c, row, 0xFF);
	}
    
}

//2^x
int xp2(int xp)
{
    int t1, r = 1;
    for(t1 = 0; t1 < xp; t1++)
    {
        r <<= 1;
    }
    return r;

}


//Print string in given size
//lsize=0 => normal height, lsize=1 => double height
void oled_putstring(int col, int row, char *s, int inv)
{
    int c = col * 6;
	
	while(*s)
	{
	    oled_putchar1(c, row, *s++, inv);
		c += 6;
	}
}

//Print an integer/long to OLED
void oled_putnumber(int col, int row, long num, int dec, int inv)
{
    char *s = malloc(16);
	if(s != NULL)
	{
	    int2asc(num, s);
	    oled_putstring(col, row, s, inv);
	    free(s);
	}	
}

void oled_clear_section(int x1, int x2, int row)
{
    int t1;
	for(t1 = x1; t1 < x2; t1++)
	{
	    oled_write_byte(t1, row, 0);	
	}

}

/////////////////////////////////
//
// STRING FUNCTIONS
//
////////////////////////////////
//INT 2 ASC
void int2asc(int num, char *buf)
{
    int i, n, d = 100;

    n = num;

    while(d)
    {
	    i = n / d;
	    n = n - i * d;
	    *(buf++) = i + 48;
	    *(buf + 1) = 0;
	    d /= 10;
    }
}

/////////////////////////////
 // Data Display Functions  // 
/////////////////////////////


///////////////
//    SPI    //
///////////////
//Homemade SPI function
int spi_send_word(unsigned short txdata)
{
	int t1 = 0;
	int r = 0;
	    
	PORTD &= ~(NSEL);
	_delay_us(TWAIT);   	
	
	//Send ID byte to define register
	for(t1 = 15; t1 >= 0; t1--)
    {
        PORTD &= ~(SCK);   
        _delay_us(TWAIT);  

	    if((1 << t1) & txdata)
        {
            PORTD |= SDI;  
        }
        else
        {
            PORTD &= ~(SDI);
        }
        _delay_us(TWAIT);   
        
        PORTD |= SCK;  
		_delay_us(TWAIT);  
        
        if(PIND & SDO)
        { 
			r += (1 << t1);
		}	
		
        PORTD &= ~(SCK);   
        _delay_us(TWAIT);  
    }
    
    PORTD |= NSEL; 
    
    return r;
}

void rfm12b_setbandwidth(unsigned char bandwidth, unsigned char gain, unsigned char drssi)
{
	spi_send_word(0x9400 | ((bandwidth&7) << 5) | ((gain & 3) <<3 ) | (drssi & 7));
}

void rfm12b_setfreq(int freq)
{	
	if (freq < 96)				// 430,2400MHz
	{
		freq = 96;
	}	
	else if (freq > 3903)			// 439,7575MHz
	{
		freq = 3903;
	}	
	spi_send_word(0xA000 | freq);
}

void rfm12b_setbaud(int baud)
{
	if (baud < 663)
	{
		spi_send_word(0xC680|((43104/5400)-1));;
	}
		
	if (baud < 5400)					// Baudrate= 344827,58621/(R+1)/(1+CS*7)
	{
		spi_send_word(0xC680|((43104/baud)-1));
	}
	else
	{
		spi_send_word(0xC600|((344828UL/baud)-1));
	}	
}

void rfm12b_setpower(unsigned char power, unsigned char mod)
{	
	spi_send_word(0x9800|(power&7)|((mod&15)<<4));
}

//Hand over ADC value from potentiometer to TX
void send_speed(char loco_id)
{	
	int t1;
	char locospeed;
	char *s0; 
	char *s1;
	
	s0 = malloc(16);
	s1 = malloc(16);
		
	for(t1 = 0; t1 < 16; t1++)
	{
		s0[t1] = 0;
	}	
		
	while(1)
	{
		locospeed = get_adc(0) >> 2;
	    int2asc(locospeed, s0);
	    
	    for(t1 = 0; t1 < 16; t1++)
	    {
		    s1[t1] = 0;
	    }	
	    s1[0] = loco_id;
	    s1[1] = ' ';
	    s1[2] = 'S';
	    s1[3] = '=';
	    strcat(s1, s0);
	    uart_tx_str(s1);
		oled_putstring(0, 0, s1, 0);
        _delay_ms(100);
    }
    
    free(s0);
    free(s1);
    
}

//////////////////////
//
//   A   D   C   
//
/////////////////////
//Read ADC value
int get_adc(int adc_channel)
{
	
	int adc_val = 0;
	
	ADMUX = (1<<REFS0) + adc_channel;     // Kanal adcmode aktivieren
    _delay_ms(3);
	
    ADCSRA |= (1<<ADSC);
    //while(ADCSRA & (1<<ADSC));
	_delay_ms(3);
	
	adc_val = ADCL;
    adc_val += ADCH * 256;   
	
	return adc_val;
}	

ISR(TIMER2_COMPA_vect)
{
    ms++; 
}

void uart_init(void)
{
	// Set Baud Rate
	UBRR0H = 0;
	UBRR0L = 103; //4800 Bd.
	
	// Set Frame Format
	UCSR0C = (0<<UMSEL00) | (3 << UCSZ00) | (0<<USBS0) | (0<<UPM00); //asynchronous, 8 data bits, 1 stop bit, no parity
	
	// Enable  Transmitter
	UCSR0B = (1<<TXEN0);
}

void uart_tx_ch(char data)
{
	while (!( UCSR0A & (1<<UDRE0))); /* Wait for empty transmit buffer       */
			
	UDR0 = data;					 /* Put data into buffer, sends the data */
}	

void uart_tx_str(char *data)
{
	int t1 = 0;
	int cs = 0;
	
	uart_tx_ch(2); //Mark begin of message string
	_delay_ms(UARTDELAY);
	while(data[t1]) //Transmit data
	{
	    uart_tx_ch(data[t1++]);
	    _delay_ms(UARTDELAY);
	}   
		
	cs = calc_checksum(data);
	
	uart_tx_ch(cs);
	_delay_ms(UARTDELAY);
	
	uart_tx_ch(3);
}	

char calc_checksum(char *s)
{
	int t1 = 0;
	int sum = 0;
	
	while(s[t1])
	{
		sum += s[t1++];
	}
	if(sum < 32) //Avoid characters coded <32!
	{
	    sum += 32;
	}    
	
	sum &= 0xFF;
	
	return sum;
}	

int main(void)
{   		
	// P O R T S
    //OUTPUT 		
    DDRD = NSEL | SCK | SDI ;
                
	//INPUT
    PORTC = 0x30;//PC0: Pull-up for key switches with various resistors against GND 
	             //I²C-Bus lines: PC4=SDA, PC5=SCL
			
    //Timer 2 as counter for 1 millisecond fclock = 8MHz
    OCR2A = 62;
    TCCR2A |= (1 << WGM21); // Set to CTC Mode
    TIMSK2 |= (1 << OCIE2A); //Set interrupt on compare match
    TCCR2B |= (1 << CS21)|(1 << CS22);  // set prescaler to 256 and start PWM
       
    //ADC config and ADC init
    ADCSRA = (1<<ADPS0) | (1<<ADPS1) | (1<<ADEN); //Prescaler 64 and ADC on
	get_adc(0); //One dummy conversion
	
	//Set timer 1 for PWM	
	TCCR1A = (1<<WGM10)|(1<<COM1A1);  // Set up the two Control registers of Timer1.
                                      // Wave Form Generation is Fast PWM 8 Bit,
    TCCR1B = (1<<WGM12)|(1<<CS12)     // OC1A and OC1B are cleared on compare match
            |(1<<CS10);               // and set at BOTTOM. Clock Prescaler is 1024.

    //UART
    uart_init();	
    
	//TWI
	twi_init();
	
	//OLED
	oled_init();
	oled_cls();	
    
    _delay_ms(500);                         //Wait for proper power up of module 
        
    //Init Si4421
    spi_send_word(0x8017);      //NO FIFO => !EL,!EF,433band,12.0pF   (1. Configuration Setting Command)
    spi_send_word(0x8239);      // !er,!ebb,ET,ES,EX,!eb,!ew,DC, FIFO (2. Power Management Command)
    spi_send_word(0xA640);      //434MHz freq. definition (n=1600)    (3. Frequency Setting Command)
    spi_send_word(0xC647);      //4.8kbps                             (4. Data Rate Command)
    spi_send_word(0x94A0);      //VDI,FAST,134kHz,0dBm,-103dBm        (5. Receiver Control Command)
    spi_send_word(0xC2AC);      //AL,!ml,DIG,DQD4                     (6. Data Filter Command)
    spi_send_word(0xC483);      //@PWR,NO RSTRIC,!st,!fi,OE,EN        (10. AFC Command) 
    spi_send_word(0x9820);      // !mp,45kHz,MAX OUT*/                (11. TX Configuration Control Command)
    spi_send_word(0xCC77);      //OB1,OB0, LPX,!ddy,DDIT,BW0          (12. PLL Setting Command)
    spi_send_word(0xE000);      //NOT USED                            (14. Wake-Up Timer Command)
    spi_send_word(0xC800);      //NOT USED                            (15. Low Duty-Cycle Command)
    spi_send_word(0xC040);      //1.66MHz,2.2V                        (16. Low Battery Detector and Microcontroller Clock Divider Command)  
 
    _delay_ms(500);
    spi_send_word(0x8208);			        // Turn on crystal
	rfm12b_setfreq((433.92-430.0)/0.0025);
	rfm12b_setbandwidth(4, 1, 4);			// 200kHz band width, -6dB gain, DRSSI threshold: -79dBm 
	rfm12b_setbaud(19200);					// 19200 baud
	rfm12b_setpower(4, 6);			  	    // 1mW output power, 120kHz frequency shift
    	
	spi_send_word(0x8238);  //2. Power Management Command TX on
	
	DDRD |= FSK; //FSK out
	 
	sei();
		
    for(;;)
    {
        //Transmit value from manual controller
		/* TEST only!
		uart_tx_ch(127);
       _delay_ms(5);		
		*/		
		send_speed('A');
    }
	return 0;
}
