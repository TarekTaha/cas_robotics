#ifndef SIMPLESERIALCOMMS_H
#define SIMPLESERIALCOMMS_H

/***************************************************
* Title			: Simple Serial Coms
* Project 		: RoboRescue Victum Identification
* Revision		: 0.1
* Author		: Oliver Thane (01058902)
* Date			: 12/01/2006
* Description	:
*	This code provides a method to access and share
*	a single serial port.
*	
***************************************************/



/***************************************************
* Include files required
***************************************************/
#include <stdio.h>		// For general IO
#include <iostream>		// For serial port coms
#include <fcntl.h>		// For serial port coms
#include <termios.h>	// For serial port coms
#include <sys/time.h>	// For checking timeout on read


/***************************************************
* Definitions
***************************************************/
#define	DEFAULT_PORT		"/dev/ttyUSB0"
#define DEFAULT_BAUD		B57600
#define DEFAULT_TIMEOUT		500e-3



/***************************************************
* Prototypes
***************************************************/
class simpleSerialComms
{
	public:
		// Constructors/Destructor
		simpleSerialComms();
		simpleSerialComms(char* port, speed_t baud);
		~simpleSerialComms();
		// Initalisation etc
		bool 			initalise(char* port, speed_t baudrate);
		bool			setBaudrate(speed_t baud);
		void closePort();
		// Reserve/Free methods
		bool			reserve(unsigned char ID);
		bool			free(unsigned char ID);
		// Put/Get methods
		bool			putByte(unsigned char input, unsigned char ID);
		bool			putBytes(unsigned char* input, unsigned char size, unsigned char ID);
		bool			getByte(unsigned char* byte, float timeout, unsigned char ID);
		bool			getByte(unsigned char* byte, unsigned char ID);
		char* 			getPort();
		speed_t 		getBaudrate();
		char* 			baudrate2str(speed_t baudrate);
	private:
		// Variables
		bool			initalised; 	// Has the object been initalised (setup serial port)
		unsigned char	owner;			// ID of AI currently controlling the serial port (0 is no-one)
		int32_t			fd;				// Serial port
		struct termios 	attr;			// Serial ports details or attributes ie baudrate
		char*			port;			// This serial port
		speed_t			baudrate;		// This serial ports baudrate
		unsigned char	serial_Byte[1];	// unsigned char read of the serial port
};

#endif

