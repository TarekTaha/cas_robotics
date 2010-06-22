#ifndef SERIALCOM_H_
#define SERIALCOM_H_

#include <termios.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

class SerialCom
{
public:
    SerialCom(){}
    SerialCom(char * port, int rate);
    ~SerialCom();
    int ReadByte(unsigned int *buf);
    int Read(unsigned int *buf,int numChars);
    void WriteByte(char buf);
    void Write(char buf[], int numChars);
    int SendCommand(char cmd[]);
    void Flush(void);
	private:
    int fd;
    struct termios oldtio;
    pthread_mutex_t lock;
};

#endif /*SERIALCOM_H_*/
