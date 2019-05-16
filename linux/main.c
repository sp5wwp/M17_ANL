#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>
#include "codec2.h"

#define		FRAME_BYTES					16

#define		CONTENT_VOICE				1
#define		CONTENT_DATA				2
#define		CONTENT_VOICE_DATA		3

#define		ENC_NONE						0
#define		ENC_STATIC_KEY				1
#define		ENC_DYNAMIC_KEY			2

#define		GPS_NONE						0

#define		KEY_NONE						0

#define		TALKGROUP_NONE				0

//FIELDS - GENERAL STRUCT
struct FIELDS {
	volatile uint8_t	content_type;		//CONTENT_TYPE		2bit
	volatile uint8_t	enc_type;			//ENC_TYPE			2bit
	volatile uint16_t	frame;				//FRAME_NUMBER		12bit
	volatile uint8_t	key_id;				//KEY_ID				8bit
	volatile uint32_t	sender_id;			//SENDER_ID			24bit
	volatile uint32_t	recipient_id;		//RECIPIENT_ID		24bit
	volatile uint16_t	crc;					//CRC					16bit
};

//RCV FIELDS - EXTRACTED FROM RECEIVED DATA
struct FIELDS rcv={0, 0, 0, 0, 0, 0, 0};

uint8_t speech_buff[16000];

uint8_t TempChar; //Temporary character used for reading
uint8_t SerialBuffer[1024];//Buffer for storing Rxed Data
uint16_t NoBytesRead;
uint16_t cnt=0;				//frame byte conuter
uint8_t frame_rcv=0;		//are we receiving a frame?

uint8_t bits[FRAME_BYTES];
int16_t samples[320];
uint16_t frame=0;

struct CODEC2 *cod;

int set_interface_attribs(int fd, int speed, int parity)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		printf("error %d from tcgetattr", errno);
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);
	
	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
	                                // no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
	
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
	
	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                // enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		printf("Error %d from tcsetattr", errno);
		return -1;
	}
	return 0;
}

void set_blocking(int fd, int should_block)
{
	struct termios tty;
   memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		printf("Error %d from tggetattr", errno);
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	printf("Error %d setting term attributes", errno);
}

int main(int argc, char *argv[])
{
	char *portname = "/dev/ttyUSB0";

	cod = codec2_create(CODEC2_MODE_3200);
	
	int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
		printf("Error %d opening %s: %s", errno, portname, strerror(errno));
		return 1;
	}

	set_interface_attribs(fd, B230400, 0);  // set speed to 230,400 bps, 8n1 (no parity)
	set_blocking (fd, 0);                // set no blocking

	printf("Opening serial port successful, listening...\t\t\t Payload:\n");

	while(1)
	{
		NoBytesRead=read(fd, &TempChar, 1);

		if(NoBytesRead)
		{
			if(!frame_rcv)
        	{
        		for(uint16_t j=0; j<4; j++)
        			SerialBuffer[j]=SerialBuffer[j+1];
        	
	        	SerialBuffer[4] = TempChar;// Store Tempchar into buffer
	        }
	        else
	        {
	        	cnt++;
	        	SerialBuffer[4+cnt] = TempChar;
	        	
	        	if(cnt==97)	//got full frame
	        	{
	        		//extract data
	        		rcv.content_type=(SerialBuffer[5+0]>>6)&3;
					rcv.enc_type=(SerialBuffer[5+0]>>4)&3;
					rcv.frame=((SerialBuffer[5+0]&0x0F)<<8) | SerialBuffer[5+1];

					rcv.sender_id=((uint32_t)SerialBuffer[5+2]<<16) | ((uint32_t)SerialBuffer[5+3]<<8) | SerialBuffer[5+4];
					rcv.recipient_id=((uint32_t)SerialBuffer[5+5]<<16) | ((uint32_t)SerialBuffer[5+6]<<8) | SerialBuffer[5+7];
					
					if(rcv.content_type==CONTENT_VOICE) //play audio
					{
						for(uint8_t i=13, j=0; i<13+32-1; i+=2, j++)
						{
							bits[j]=SerialBuffer[i];
						}

						codec2_decode(cod, &samples[0], &bits[0]);
						codec2_decode(cod, &samples[160], &bits[8]);

						for(uint8_t i=0; i<160; i++)
						{
							samples[i]*=atof(argv[1]);
							printf("%c", (uint8_t)(samples[i]>>8));
							printf("%c", (uint8_t)(samples[i]&0xFF));
						}
						for(uint16_t i=160; i<320; i++)
						{
							samples[i]*=atof(argv[1]);
							printf("%c", (uint8_t)(samples[i]>>8));
							printf("%c", (uint8_t)(samples[i]&0xFF));
						}
	        		}
	        		
	        		for(uint16_t j=0; j<5; j++)
        				SerialBuffer[j]=0;
					frame_rcv=0;
	        		cnt=0;
	        	}
	        }
	             
	        if(SerialBuffer[0]=='F' && SerialBuffer[1]=='R' && SerialBuffer[2]=='A' && SerialBuffer[3]=='M' && SerialBuffer[4]=='E')
	        {
	        	frame_rcv=1;
	    	}
	    }
  	}	

	set_interface_attribs(fd, B0, 0);
	
	return 0;
}
