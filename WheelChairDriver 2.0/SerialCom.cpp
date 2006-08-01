#include "SerialCom.h"

SerialCom::SerialCom(char * port, int rate) 
{
	fd = -1;
	
	// open the serial port
	fd = open(port, O_RDWR | O_NOCTTY);// | O_NDELAY | O_NONBLOCK);
	if ( fd<0 )
	{
		fprintf(stderr, "Could not open serial device %s\n",port);
		return;
	}
	
	// save the current io settings
	tcgetattr(fd, &oldtio);

	// rtv - CBAUD is pre-POSIX and doesn't exist on OS X
	// should replace this with ispeed and ospeed instead.
	
	// set up new settings
	struct termios newtio;
	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag = CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	newtio.c_cc[VTIME] = 10;
	newtio.c_cc[VMIN] = 0;

	// activate new settings
	tcflush(fd, TCIFLUSH);
	if (cfsetispeed(&newtio, rate) < 0 || cfsetospeed(&newtio, rate) < 0) {
		fprintf(stderr,"Failed to set serial baud rate: %d\n", rate);
		tcsetattr(fd, TCSANOW, &oldtio);	
		close(fd);
		fd = -1;				
		return;
	}
	tcsetattr(fd, TCSANOW, &newtio);
	tcflush(fd, TCIOFLUSH);
	
	// clear the input buffer in case junk data is on the port
	usleep(10000);
	tcflush(fd, TCIFLUSH);
}

SerialCom::~SerialCom() 
{
	// restore old port settings
	if (fd > 0) {
		tcsetattr(fd, TCSANOW, &oldtio);
		close(fd);	
	}
}

void SerialCom::Flush(void) 
{
	tcflush(fd, TCIOFLUSH);
}

int SerialCom::ReadByte(unsigned int *buf) 
{
	char temp;
	temp = read(fd,buf,1);
	return temp;
}

void SerialCom::WriteByte(char buf) 
{
	write(fd,&buf,1);
}

void SerialCom::Write(char buf[], int numChars) 
{
	write(fd,buf,numChars);
}

// Send a command to the IO board and clear the two returned bytes.
// This is not generic but it is convienient to have located here.
int SerialCom::SendCommand(char cmd[]) 
{
	char cmd2[7];
	unsigned int retbyte, retbyte2;
		sprintf (cmd2, "%s\r", cmd);
		Write(cmd2, 6);
		ReadByte(&retbyte);
		ReadByte(&retbyte2);
	if ((char)retbyte == 'O' || (char)retbyte == 'L') 
	{
		return 0;
	} else 
	{
		return -1;
	}
}
