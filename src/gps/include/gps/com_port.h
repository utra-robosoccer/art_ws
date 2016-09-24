#ifndef COM_PORT_H
#define COM_PORT_H

typedef int ComPortHandle;
typedef unsigned char Byte;

ComPortHandle OpenComPort(const char* comPortPath, bool silent);
void CloseComPort(ComPortHandle comPort);

Byte readByte(ComPortHandle comPort);

#endif
