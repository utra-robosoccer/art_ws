#include <termios.h> // terminal io (serial port) interface
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
#include <assert.h>

#include <iostream>
#include <stdio.h> // added by liang
#include <stdlib.h>
#include <string.h> // added by liang
#include <unistd.h>

#include "com_port.h"

using std::cout;
using std::cerr;
using std::endl;

//Function to combine two bytes and make a signed short
short MakeShort(Byte msb, Byte lsb)
{
  //short must be a 2 byte integer
  assert(sizeof(short) == 2);
  short s = 0;
  //map the short to a byte array
  Byte* tmp = (Byte*)&s;
  tmp[1] = msb;
  tmp[0] = lsb;
  return s;
}

Byte* ShortToBytes(short s) {
  Byte* result = (Byte*)malloc(2);
  Byte* tmp = (Byte*)&s;/*<----------------Why is this here?*/

  Byte msb = (s >> 8) & 0xff;
  Byte lsb = s & 0xff;
  result[0] = msb;
  result[1] = lsb;
  return result;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// utility functions for working with a com port in Linux
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

//=======================================================================================
// Purge
//---------------------------------------------------------------------------------------
// Clears the com port's read and write buffers
//=======================================================================================

bool Purge(ComPortHandle comPortHandle)
{
  if (tcflush(comPortHandle,TCIOFLUSH)==-1)
    {
      cerr << "flush failed" << endl;
      return false;
    }
  return true;
}

//=======================================================================================
// OpenComPort
//---------------------------------------------------------------------------------------
// Opens a com port with the correct settings for communicating with a MicroStrain
// 3DM-GX1 sensor
//=======================================================================================

ComPortHandle OpenComPort(const char* comPortPath, bool silent)
{
  ComPortHandle comPort = open(comPortPath, O_RDWR | O_NOCTTY | O_NDELAY);

  if (comPort== -1) //Opening of port failed
    {
      if (errno != 2) {
	cerr << "Unable to open com port: \"" << comPortPath << "\"\n"
	  "Error:(" << errno << ") " << strerror(errno) << endl;
      }
      return -1;
    }
  fcntl(comPort, F_SETFL, 0);
  //Get the current options for the port...
  termios options;
  tcgetattr(comPort, &options);

  /* 8 bits, no parity, no stop bits */
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  /* Canonical mode */
  //options.c_lflag |= ICANON;
  
  options.c_cflag &= ~CRTSCTS;
  /* enable receiver, ignore status lines */
  options.c_cflag |= CREAD | CLOCAL;
  /* disable input/output flow control, disable restart chars */
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  /* disable canonical input, disable echo,
     disable visually erase chars,
	 disable terminal-generated signals */
  options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  /* disable output processing */
  options.c_oflag &= ~OPOST;
   
  options.c_cc[VMIN] = 0;
  /* no minimum time to wait before read returns */
  options.c_cc[VTIME] = 10;

  //options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  //options.c_oflag &= ~OPOST;


  //options.c_cflag |= (CLOCAL | CREAD);

  //options.c_cflag &= ~PARENB;
  //options.c_cflag &= ~CSTOPB;
  //options.c_cflag &= ~CSIZE;
  //options.c_cflag |= CS8;

  //options.c_cflag |= (CLOCAL | CREAD);
  //options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  //options.c_oflag &= ~OPOST;

  // Setup termios for 8N1
  //options.c_cflag &= ~PARENB;
  //options.c_cflag &= ~CSTOPB;
  //options.c_cflag &= ~CSIZE;
  //options.c_cflag |= CS8;

  // Reccomended settings
  //options.c_cflag &= ~CRTSCTS;   // no flow control
  //options.c_cflag |= CREAD | CLOCAL;  // turn on read & ignore ctrl lines
  //options.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
  //options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw 
  //options.c_lflag |= (ICANON | ECHO | ECHOE);
  //options.c_iflag |= (INPCK | ISTRIP);
									
  //options.c_oflag &= ~OPOST; // make raw

  //set the baud rate to 115200
  //int baudRate = 115200;
  //int baudRate = 38400;
  //int baudRate = 9600;
  cfsetospeed(&options, B115200);
  cfsetispeed(&options, B115200);
  
  //Time-Outs -- won't work with NDELAY option in the call to open
  //options.c_cc[VMIN]  = 0;   // block reading until RX x characers. If x = 0, it is non-blocking.
  //options.c_cc[VTIME] = 10;   // Inter-Character Timer -- i.e. timeout= x*.1 s

  //Purge serial port buffers
  //Purge(comPort);

  //Set the new options for the port...
  int status=tcsetattr(comPort, TCSANOW, &options);
  if (status != 0) //For error message
    {
      //if (!silent) {
		cout << "Configuring comport failed" << endl;
      //}
      return status;
    }
   tcflush(comPort, TCIFLUSH);
  //Purge serial port buffers
  //Purge(comPort);
  return comPort;
}

//=======================================================================================
// CloseComPort
//---------------------------------------------------------------------------------------
// Closes a port that was previously opened with OpenComPort
//=======================================================================================

void CloseComPort(ComPortHandle comPort)
{
  close(comPort);
}

//=======================================================================================
// readComPortQuick
//---------------------------------------------------------------------------------------
// read the specified number of bytes from the com port
//=======================================================================================

int readComPortQuick(ComPortHandle comPort, Byte* bytes, int bytesToRead)
{
  int bytesRead = read(comPort, bytes, bytesToRead);
  cout << bytesRead << endl;
  return bytesRead; 
}

//=======================================================================================
// readComPort
//---------------------------------------------------------------------------------------
// read the specified number of bytes from the com port
//=======================================================================================

int readComPort(ComPortHandle comPort, Byte* bytes, int bytesToRead)
{
  Byte* buffer = (Byte*)malloc(bytesToRead);
  int totalBytesRead = 0;
  while (totalBytesRead != bytesToRead) {
    int bytesRead = read(comPort, buffer, bytesToRead - totalBytesRead);
    for (int i = 0; i < bytesRead; i++) {
      bytes[totalBytesRead + i] = buffer[i];
    }
    totalBytesRead += bytesRead;
  }
  //int bytesRead = read(comPort, bytes, bytesToRead);
  return totalBytesRead;
  //int bytesRead = read(comPort, bytes, bytesToRead);
  //return bytesRead; 
}

Byte readByte(ComPortHandle comPort) {
	Byte buffer;
	
	int bytesRead = read(comPort, &buffer, 1);
	if (bytesRead > 0) {
		return buffer;
	} else {
		return 0;
	}
}

//=======================================================================================
// writeComPort
//---------------------------------------------------------------------------------------
// send bytes to the com port
//======================================================================================

int writeComPort(ComPortHandle comPort, const Byte* bytesToWrite, int size)
{
  return write(comPort, bytesToWrite, size);
}
