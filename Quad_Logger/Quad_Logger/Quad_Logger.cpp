// Quad_Logger.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdio.h>
#include <tchar.h>
#include <conio.h>
#include <winsock2.h>
#include <windows.h>
#include<time.h> 
#include <math.h>
#pragma comment(lib, "winmm.lib")
#pragma warning( disable : 4996 )

bool bLogData = false;

char bytes_to_read[23]="";//contains array of bytes to be read
char bytes_to_log[23]="";

// Serial port for the communication 
HANDLE hSerial1; 

DCB dcbSerialParams = {0};
COMMTIMEOUTS timeouts = {0};
INT16 log_data[8] = {0,0,0,0,0,0,0,0};

void Comms()
{
	// Send specified text trough USB (remaining command line arguments)
	DWORD bytes_written;
	DWORD bytes_header;
	char header_bytes[3] = {0,0,0};

	char checksum=0;

	//==================================================================
	if(!ReadFile(hSerial1, header_bytes, 3, &bytes_header, NULL))
	{
		printf("Error \n");
		return;
	}
	else
	{
		if (header_bytes[0] == '$' && header_bytes[1] == 'M' && header_bytes[2] == '>')
		{
			header_bytes[0] = 0;
			header_bytes[1] = 0;
			header_bytes[2] = 0;
			//printf("\nReceived: ");
			if(!ReadFile(hSerial1, bytes_to_read, 13, &bytes_written, NULL))
			{
				printf("Error \n");
				return;
			}
			else
			{
				if (true)
				{
					for (int u = 0; u < 12; u++)
					{
						checksum ^= bytes_to_read[u];
					}

					if (checksum == bytes_to_read[12])
					{
						//printf("CHECKSUM PASSED!\n");
						bLogData = true;
					}
					else
					{
						//printf("FAIL!\n");
					}

					for (int u = 0; u < 10; u++)
					{
						bytes_to_log[u] = bytes_to_read[u+2];
						bytes_to_read[u+2] = 0;
					}
				}
			}
		}
	}
}

int _tmain(int argc, _TCHAR* argv[])
{
	FILE* Record = fopen("Record.csv","w"); // File for recording data
	/////////////////////////////////////////////////////////////////////////////////////    
	//						Prepare the USB port to send data						   //
	/////////////////////////////////////////////////////////////////////////////////////
	// Open the available serial port number 
	fprintf(stderr, "Opening serial port..."); 
	// Check the number of serial port
	hSerial1 = CreateFile(_T("\\\\.\\COM3"), GENERIC_READ, 0, NULL,OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL /*| FILE_FLAG_OVERLAPPED*/, NULL ); 

	if (hSerial1 == INVALID_HANDLE_VALUE)
	{ 
		fprintf(stderr, "Error\n"); 
		return 1; 
	} 
	else fprintf(stderr, "OK\n");


	// Set device parameters (38400 baud, 1 start bit,
	// 1 stop bit, no parity) 
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	if (GetCommState(hSerial1, &dcbSerialParams) == 0)
	{
		fprintf(stderr, "Error getting device state\n");
		CloseHandle(hSerial1);
		return 1;
	}

	dcbSerialParams.BaudRate = CBR_115200;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;

	if(SetCommState(hSerial1, &dcbSerialParams) == 0)
	{ 
		fprintf(stderr, "Error setting device parameters\n");
		CloseHandle(hSerial1); 
		return 1; 
	} 

	// Set COM port timeout settings 
	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutConstant = 50; 
	timeouts.ReadTotalTimeoutMultiplier = 10; 
	timeouts.WriteTotalTimeoutConstant = 50; 
	timeouts.WriteTotalTimeoutMultiplier = 10; 

	if(SetCommTimeouts(hSerial1, &timeouts) == 0) 
	{
		fprintf(stderr, "Error setting timeout settings\n"); 
		CloseHandle(hSerial1); 
		return 1; 
	}

	while (true)
	{
		Comms();

		for (int u = 0; u < 10; u++)
		{
			((UINT8*)(log_data))[u] = (UINT8)(bytes_to_log[u]);
		}

		fprintf(Record, "%d,%d,%d,%d,%d\n", log_data[0], log_data[1], log_data[2], log_data[3], log_data[4]);/*, log_data[5], log_data[6], log_data[7], log_data[8], log_data[9]);*/
		printf("\n");
		printf("%d \n",log_data[0]);
		printf("%d \n",log_data[1]);
		printf("%d \n",log_data[2]);
		printf("%d \n",log_data[3]);
		printf("%d \n",log_data[4]);
		bLogData = false;
	}

}

