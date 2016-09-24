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

//=======================================================================================
// OpenComPort
//---------------------------------------------------------------------------------------
// Opens a com port with the correct settings for communicating with GPS
//=======================================================================================

ComPortHandle OpenComPort(const char* comPortPath, bool silent)
{
  ComPortHandle comPort = open(comPortPath, O_RDWR | O_NOCTTY | O_NDELAY);

  if (comPort== -1) //Opening of port failed
    {
      if (errno != 2) {
          std::cout << "Unable to open com port: \"" << comPortPath << "\"\n"
              "Error:(" << errno << ") " << strerror(errno) << std::endl;
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

  //set the baud rate to 115200
  // int baudRate = 4800;  //CHANGE BAUDRATE
  //int baudRate = 38400;
  //int baudRate = 9600;
  cfsetospeed(&options, B4800);
  cfsetispeed(&options, B4800);
  
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
        std::cout << "Configuring comport failed" << std::endl;
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

Byte readByte(ComPortHandle comPort) {
	Byte buffer;
	
	int bytesRead = read(comPort, &buffer, 1);
	if (bytesRead > 0) {
		return buffer;
	} else {
		return 0;
	}
}

