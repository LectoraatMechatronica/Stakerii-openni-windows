#include <Windows.h>
#include <iostream>
#include <Serial.h>

HANDLE hSerial;

void test(){
	serialInit();
	DWORD result = readSerial();
	printf("Serial terminal result: %d", result);
	close();
}


void serialInit(){
	hSerial = CreateFile(L"COM1", GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL,0);

	if(hSerial==INVALID_HANDLE_VALUE){
		if(GetLastError()==ERROR_FILE_NOT_FOUND){
			//serial port does not exit
			printf("Error: (S1) serial port does not exist.\n");
		}else{
			//some other error
			printf("Error: (S2) cannot connect to serial port.\n");
		}
		return;
	}
	DCB dcbSerialParams = {0};
	//dcbSerial.DCBlength=sizeof(dcbSerialParams); //??

	if(!GetCommState(hSerial, &dcbSerialParams)){
		//error getting state
		printf("Error: (S3) Error getting port state.\n");
	}
	dcbSerialParams.BaudRate=CBR_115200;
	dcbSerialParams.ByteSize=8;
	dcbSerialParams.StopBits=ONESTOPBIT;
	dcbSerialParams.Parity=NOPARITY;
	
	if(!GetCommState(hSerial, &dcbSerialParams)){
		//error getting state
		printf("Error: (S4) Error setting port state.\n");
	}

	COMMTIMEOUTS timeouts={0};

	timeouts.ReadIntervalTimeout=50;
	timeouts.ReadTotalTimeoutConstant=50;
	timeouts.ReadTotalTimeoutMultiplier=10;
	timeouts.WriteTotalTimeoutConstant=50;
	timeouts.WriteTotalTimeoutMultiplier=10;

	if(!SetCommTimeouts(hSerial,&timeouts)){
		//error occureed
		printf("Error: (S5) Error setting timeout state.\n");
	}	
}

DWORD readSerial(){
	char szBuff[5] = {0}; 
	DWORD dwBytesRead = 0;
	if(!ReadFile(hSerial, szBuff, 4,&dwBytesRead,NULL)){
		printf("Error: (S6) Cannot read serial terminal.\n");
	}
	return dwBytesRead;
}

void close(){
	CloseHandle(hSerial);
}