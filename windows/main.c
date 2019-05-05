#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <windows.h>

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
	volatile uint8_t	key_id;				//KEY_ID			8bit
	volatile uint32_t	sender_id;			//SENDER_ID			24bit
	volatile uint32_t	recipient_id;		//RECIPIENT_ID		24bit
	volatile uint16_t	crc;				//CRC				16bit
};

//RCV FIELDS - EXTRACTED FROM RECEIVED DATA
struct FIELDS rcv={0, 0, 0, 0, 0, 0, 0};

int main(void)
{
	HANDLE hComm;

	hComm = CreateFile("\\\\.\\COM4",                //port name
                    GENERIC_READ | GENERIC_WRITE, //Read/Write
                	0,                            // No Sharing
                    NULL,                         // No Security
                    OPEN_EXISTING,				// Open existing port only
                    0,            				// Non Overlapped I/O
                    NULL);       					 // Null for Comm Devices

	if (hComm == INVALID_HANDLE_VALUE)
	{
		printf("Error in opening serial port\n");
		return 0;
	}
 	else
 		printf("Opening serial port successful, listening...\t\t\t Payload:\n\n");
 		
 	DCB dcbSerialParams = { 0 }; // Initializing DCB structure
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	
	dcbSerialParams.BaudRate = CBR_256000;  // Setting BaudRate = 9600
	dcbSerialParams.ByteSize = 8;         // Setting ByteSize = 8
	dcbSerialParams.StopBits = ONESTOPBIT;// Setting StopBits = 1
	dcbSerialParams.Parity   = NOPARITY;  // Setting Parity = None
	
	SetCommState(hComm, &dcbSerialParams);
	
	COMMTIMEOUTS timeouts = { 0 };
	timeouts.ReadIntervalTimeout         = 30; // in milliseconds
	timeouts.ReadTotalTimeoutConstant    = 30; // in milliseconds
	timeouts.ReadTotalTimeoutMultiplier  = 10; // in milliseconds
	timeouts.WriteTotalTimeoutConstant   = 50; // in milliseconds
	timeouts.WriteTotalTimeoutMultiplier = 10; // in milliseconds
	
	uint8_t TempChar; //Temporary character used for reading
	uint8_t SerialBuffer[1024];//Buffer for storing Rxed Data
	uint16_t NoBytesRead;
	uint16_t cnt=0;				//frame byte conuter
	uint8_t frame_rcv=0;		//are we receiving a frame?

	while(1)
 	{
   		ReadFile(hComm,       //Handle of the Serial port
             &TempChar,       //Temporary character
             sizeof(TempChar),//Size of TempChar
             &NoBytesRead,    //Number of bytes read
             NULL);
        
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
					
					//print out data
	        		printf("FRAME %03d", rcv.frame);
	        		printf("\t%07d->%07d", rcv.sender_id, rcv.recipient_id);
	        		if(rcv.content_type==CONTENT_VOICE)
	        			printf("\tVOICE");
	        		else if(rcv.content_type==CONTENT_DATA)
	        			printf("\tDATA");
	        		else if(rcv.content_type==CONTENT_VOICE_DATA)
	        			printf("\tVOICE+DATA");
	        		
	        		if(rcv.enc_type==ENC_NONE)
	        			printf("\tENCR_NONE");
	        		else if(rcv.enc_type==ENC_STATIC_KEY)
	        			printf("\tENCR_STATIC");
	        		else if(rcv.enc_type==ENC_DYNAMIC_KEY)
	        			printf("\tENCR_DYNAMIC");
	        		
	        		printf("\t");
	        		for(uint8_t i=13; i<13+16-1; i++)
	        			printf(" %02X", SerialBuffer[i]);
	        		
	        		printf("\n");
	        		
	        		
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

	CloseHandle(hComm);	//Closing the Serial Port
	
	return 0;
}

