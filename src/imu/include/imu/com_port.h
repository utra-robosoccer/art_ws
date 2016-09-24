typedef int ComPortHandle;
typedef unsigned char Byte;

short MakeShort(Byte msb, Byte lsb);
Byte* ShortToBytes(short s);
 
bool Purge(ComPortHandle comPortHandle);
ComPortHandle OpenComPort(const char* comPortPath, bool silent);
void CloseComPort(ComPortHandle comPort);

int readComPortQuick(ComPortHandle comPort, Byte* bytes, int bytesToRead);
int readComPort(ComPortHandle comPort, Byte* bytes, int bytesToRead);
int writeComPort(ComPortHandle comPort, const Byte* bytesToWrite, int size);
Byte readByte(ComPortHandle comPort);
