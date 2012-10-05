#ifndef SERIAL_F
#define SERIAL_F


#include <windows.h>
#include <tchar.h>
#include <stdio.h>
#include <atlstr.h>

class PSerial
{
	//PSerial();
	

  public:
	  int serial_send(CString data);
	  PSerial();
	  ~PSerial();
	  int Connect(TCHAR * commport,long baudrate,BYTE bytesize,BYTE parity,BYTE stopbits);
	  int Close();

private:
	BOOL fSuccess;
	CString filetext;
	DCB dcb;
	HANDLE hCom;
};

#endif