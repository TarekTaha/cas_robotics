#include "simpleSerialComms.h"

/************************ Constructors/Destructor *****************************/

/***************************************************
* Contstructors/Destructor
*	These will create a connection to a serial port
*	either on the default /dev/ttyUSB0 @ 38400bps
*	or on the supplied port and baudrate.
***************************************************/
// Open serial port on the default port & baudrate
simpleSerialComms::simpleSerialComms()
{
	initalised	= false;
	owner		= 255;
	//initalise(DEFAULT_PORT, DEFAULT_BAUD);
}
// Open serial port on a given port & baudrate
simpleSerialComms::simpleSerialComms(char* port, speed_t baud)
{
	initalised	= false;
	owner		= 255;
	initalise(port,baud);
}
// Close ports and free any used memory
simpleSerialComms::~simpleSerialComms()
{
	if (initalised)
	{
		close(fd);
	}
}


/*************************** PUBLIC  FUNCTIONS ********************************/

/***************************************************
* Public Functions: initalise
*	initialises the given serial port at the given
*	baud rate.
***************************************************/
// Initalise
bool simpleSerialComms::initalise(char* port, speed_t baudrate)
{
	// Set flags & other inits
	initalised	= false;
	this->port 	= port;
	
	// Setup serial port for IO
	printf("Initalising Serial Port %s ... ",port);
	fd 			= open(port, O_RDWR|O_NOCTTY|O_NDELAY);
	initalised	= true;
	if (!fd) 
	{
		printf("failed to open serial port\n");
		return(false);
	}
	if(tcflush(fd,TCIOFLUSH)<0) 
	{
		printf("failed to flush the port\n");
		return(false);
	}
	if (tcgetattr(fd,&attr)<0)
	{
		printf("failed get the ports attributes\n");
		return(false);
	}
	cfmakeraw(&attr);
  	cfsetispeed(&attr,baudrate);
  	cfsetospeed(&attr,baudrate);
	if (tcsetattr(fd,TCSAFLUSH,&attr)<0)
	{
		printf("failed to set the ports attributes\n");
		return(false);
	}
	this->baudrate	= baudrate;
	printf("success\n");
	
	return(true);
}



/***************************************************
* Public Functions: Set baudrate
*	Sets the baudrate of the serial port
***************************************************/
// Set the baudrate of the port
bool simpleSerialComms::setBaudrate(speed_t baudrate)
{
//	if (owner != ID)
//		return(false);
	
	if (tcgetattr(fd,&attr)<0)
	{
		printf("failed get the ports attributes\n");
		return(false);
	}
	cfmakeraw(&attr);
  	cfsetispeed(&attr,baudrate);
  	cfsetospeed(&attr,baudrate);
	if (tcsetattr(fd,TCSAFLUSH,&attr)<0)
	{
		printf("failed to set the ports attributes\n");
		return(false);
	}
	return(true);
}


/***************************************************
* Public Functions: reserve/free
*	This method allows multiple clients to handle
*	the serial port. Ie you first wait until you
*	can reserve the port then do your writing and
*	reading, then free up the port for others
***************************************************/
// Reserve the port
bool simpleSerialComms::reserve(unsigned char ID)
{
	// If already owned refuse reservation
	if (owner != 255)
		return(false);
	
	owner	= ID;
	return(true);
}
// free the port
bool simpleSerialComms::free(unsigned char ID)
{
	if (ID != owner)
		return(false);
	
	owner 	= 255;
	return(true);
}

void simpleSerialComms::closePort()
{
	if (initalised)
	{
		close(fd);
	}
}


/***************************************************
* Public Functions: Single byte functions
* 	These methods allow the user to put a single 
*	unsigned char onto the serial port and read one 
*	back.
***************************************************/
// Put a unsigned character onto the serial port
bool simpleSerialComms::putByte(unsigned char data, unsigned char ID)
{
	unsigned char sendData[1];
	if (owner != ID)
		return(false);

	sendData[0] = data;
	write(fd, sendData, 1);
	return(true);
}
// Read a unsigned character from the serial port
bool simpleSerialComms::getByte(unsigned char* byte, float timeout, unsigned char ID)
{
	struct timeval		currentTime, startTime;
	float				deltaTime;
	
	if (owner != ID)
		return(false);
	
	deltaTime = 0;
	gettimeofday(&startTime,NULL);
	// Wait for a character to come
	while(read(fd,serial_Byte,1)<0)
	{
		// Find delta time
		gettimeofday(&currentTime,NULL);
		deltaTime = (currentTime.tv_sec + currentTime.tv_usec/1e6)-
             		(startTime.tv_sec   + startTime.tv_usec/1e6);
		// If the reading has timed out exit
		if (deltaTime >= timeout)
		{
			printf("timeout\n");
			return(false);
		}
	}
	
	// Now copy the result into byte
	*byte = serial_Byte[0];
	
	return(true);
}
// Get byte with default timeout
bool simpleSerialComms::getByte(unsigned char* byte, unsigned char ID)
{
	return(getByte(byte, DEFAULT_TIMEOUT, ID));
}


/***************************************************
* Public Functions: Multi byte functions
*	This function lets the user send from 1 to 255
*	sets of bytes in an arrar
***************************************************/
// Multi byte send
bool simpleSerialComms::putBytes(unsigned char* data, unsigned char size, unsigned char ID)
{
	for (unsigned char count = 0; count < size; count++)
	{
		if (!putByte(data[count], ID))
			return(false);
	}
	return(true);
}


/***************************************************
* Public Functions: Get port/ Get baudrate
*	Simply returns the serial port name used by this
*	object or the baudrate of the port
***************************************************/
// get port
char* simpleSerialComms::getPort()
{
	return(port);
}
// get baud
speed_t simpleSerialComms::getBaudrate()
{
	return(baudrate);
}

/***************************************************
* Public Functions: baud too string
*	This function returns a baudrate as a string
***************************************************/
// get port
char* simpleSerialComms::baudrate2str(speed_t baudrate)
{
	if (baudrate == B2400)
		return("2400bps");
	else if (baudrate == B4800)
		return("4800bps");
	else if (baudrate == B9600)
		return("9600bps");
	else if (baudrate == B19200)
		return("19200bps");
	else if (baudrate == B38400)
		return("38400bps");
	else if (baudrate == B57600)
		return("57600bps");
	else if (baudrate == B115200)
		return("115200bps");
	//else if (baudrate == B153600) // Doesnt seem to be supported in C++
	//	return("153600bps");
	else if (baudrate == B230400)
		return("230400bps");
	else if (baudrate == B460800)
		return("460800bps");
	else
		return("unknown baudrate");
}
