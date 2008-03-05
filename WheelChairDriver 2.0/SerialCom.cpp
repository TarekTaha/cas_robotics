#include "SerialCom.h"

SerialCom::SerialCom(char * port, int rate) 
{
	// open the serial port
	//printf("\n HERE begin");  fflush(stdout);
	fd = open(port, O_RDWR | O_NOCTTY );//| O_NONBLOCK| O_NDELAY);
	if ( fd<0 )
	{
		fprintf(stderr, "Could not open serial device %s\n",port);
		return;
	}

	int flags = fcntl(fd, F_GETFL, 0) ;
    	if( -1 == fcntl(fd, F_SETFL, flags | O_NONBLOCK ) ) 
	{
//        	return -1 ;
	}
/*
    	//
    	// Flush out any garbage left behind in the buffers associated
    	// with the port from any previous operations. 
    	//
    	if( -1 == tcflush(fd, TCIOFLUSH) ) 
	{
//	       	return -1 ;
    	}
    	//
    	// Set up the default configuration for the serial port. 
    	//
	struct termios tio;
	tio.c_iflag = IGNBRK;
	tio.c_oflag = 0;
	tio.c_cflag = B19200 | CS8 | CLOCAL | CREAD;
	tio.c_lflag = 0;
	//
	// :TRICKY:
	// termios.c_line is not a standard element of the termios structure (as 
	// per the Single Unix Specification 2. This is only present under Linux.
	//
	#ifdef __linux__
	tio.c_line = '\0';
	#endif
	bzero( &tio.c_cc, sizeof(tio.c_cc) );
	tio.c_cc[VTIME] = 0;
	tio.c_cc[VMIN]  = 0;
    	if ( -1 == tcsetattr(fd,TCSANOW,&tio) ) 
	{
//		return -1 ;
	}
	cfsetispeed(&tio, rate);
	cfsetospeed(&tio, rate);
      	if( -1 == tcsetattr(fd, TCSANOW, &tio) ) 
	{
//		return -1 ;
        }
	// 8 Char size
        tio.c_iflag &= ~ISTRIP ; // clear the ISTRIP flag.
        tio.c_cflag &= ~CSIZE ;     // clear all the CSIZE bits.
       	tio.c_cflag |= 8 ;  // set the character size. 
       	//
       	// Set the new settings for the serial port. 
       	//
       	if( -1 == tcsetattr(fd, TCSANOW, &tio) ) 
	{
//		return -1;
       	} 
	// 1 Stop Bit
        tio.c_cflag &= ~CSTOPB ;
	//
	// Set the new settings for the serial port. 
	//
	if( -1 == tcsetattr(fd, TCSANOW, &tio) ) 
	{
//		return -1 ;
	}
	// No Parity
       	tio.c_cflag &= ~PARENB ;
    	//
	// Write the settings back to the serial port. 
	//
	if( -1 == tcsetattr(fd, TCSANOW, &tio) ) 
	{
//		return -1 ;
	} 
	// Set the flow control. Hardware flow control uses the RTS (Ready
	// To Send) and CTS (clear to Send) lines. Software flow control
	// uses IXON|IXOFF
	// Hardware
	tio.c_iflag &= ~ (IXON|IXOFF);
        tio.c_cflag |= CRTSCTS;
       	tio.c_cc[VSTART] = _POSIX_VDISABLE;
       	tio.c_cc[VSTOP] = _POSIX_VDISABLE;
	if (-1 ==tcsetattr(fd, TCSANOW, &tio)) 
	{
//        	return -1;
    	}
    	//
    	// Allow all further communications to happen in blocking 
    	// mode. 
    	//
    	flags = fcntl(fd, F_GETFL, 0) ;
    	if( -1 == fcntl(fd, F_SETFL,flags & ~O_NONBLOCK ) ) 
	{
//        	return -1 ;
    	}
*/
	// save the current io settings
	tcgetattr(fd, &oldtio);

	// rtv - CBAUD is pre-POSIX and doesn't exist on OS X
	// should replace this with ispeed and ospeed instead.


	// set up new settings
	struct termios newtio;
	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag = CS8 | CLOCAL | CREAD;
	
	// No Parity
	newtio.c_cflag &= ~PARENB;
	// One Stop Bit	
	newtio.c_cflag &= ~CSTOPB;
	// for 8-bit char, this prevents setting the MSB to zero
	newtio.c_iflag &= ~ISTRIP;
	
	// For soft Flow
        newtio.c_iflag |= IXON|IXOFF;
        newtio.c_cflag &= ~ CRTSCTS;
	// Start and stop characters
        newtio.c_cc[VSTART] = 0x11; // ^q
        newtio.c_cc[VSTOP]  = 0x13; // ^s
	
	// Ignore Parity errors
	newtio.c_iflag = IGNPAR;

	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	newtio.c_cc[VTIME] = 1;
	newtio.c_cc[VMIN]  = 1;
	// activate new settings
	tcflush(fd, TCIFLUSH);
	if (cfsetispeed(&newtio, rate) < 0 || cfsetospeed(&newtio, rate) < 0) 
	{
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
    	flags = fcntl(fd, F_GETFL, 0) ;
    	if( -1 == fcntl(fd, F_SETFL,flags & ~O_NONBLOCK ) ) 
	{
//        	return -1 ;
    	}
	//printf("\n HERE begin");  fflush(stdout);
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

int SerialCom::Read(unsigned int *buf,int size) 
{
	char temp;
	for(int i=0;i<size;i++)
	{
		if((temp = read(fd,&buf[i],1))==-1)
			return -1;
	}
	return size;
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
	unsigned int retbyte[2];
	//printf("\nSending Chars:");
	for(unsigned int i=0; i< 5;i++)
	{
		WriteByte(cmd[i]);
	//	printf("%c",cmd[i]);
	}
	WriteByte(13);
	//usleep(1000);
	fflush(stdout);
	//printf("\nRecieved Chars:");
	if(Read(retbyte,2) !=2)
		return -1;
	//while(ReadByte(&retbyte[count]))
	//for(int i=0;i<2;i++)
	//{ 
	//	printf("%c",retbyte[i]); 
	//	count++; 
	//	if(count>19) 
	//		break;
	//}
	//fflush(stdout);
	if ((char)retbyte[0] == 'O' || (char)retbyte[0] == 'L') 
	{
		return 0;
	}
	else 
	{
		return -1;
	}
}
