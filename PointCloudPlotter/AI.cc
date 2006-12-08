#include "AI.h"


/************************ Constructors/Destructor *****************************/

/***************************************************
* Contstructors/Destructor
*	These constructors take a device ID (ie the ID
*	for the AI motor from 0 to 30) and a pointer to
*	an initalised serial port object from the header
*	simpleSerialComms.h
***************************************************/
// Nothing constructor because C++ is crap
AI::AI()
{
}
// Simple test constructor
AI::AI(unsigned char ID,simpleSerialComms* serialPort)
{
	this->ID			= ID;
	this->serialPort	= serialPort;
}
// This is a relativly advanced automatic constructor 
// that sets almost every aspect of the servo
AI::AI(unsigned char ID, simpleSerialComms* serialPort, unsigned char res, unsigned char minPos, unsigned char maxPos, unsigned char Pgain, unsigned char Dgain)
{
	initalise(ID, serialPort, res, minPos, maxPos, Pgain, Dgain);
}
// Destructor, returns to home
AI::~AI()
{
	// Go back to the initial position
	//setPos(127,DEFAULT_SPEED);
}



/*************************** PUBLIC  FUNCTIONS ********************************/

/***************************************************
* Public Functions: initialise
*	Initialise all the properties of the AI servo.
***************************************************/
// Advanced initialisation
bool AI::initalise(unsigned char ID, simpleSerialComms* serialPort, unsigned char res, unsigned char minPos, unsigned char maxPos, unsigned char Pgain, unsigned char Dgain)
{
	speed_t		backupBaudrate;
	
	this->ID			= ID;
	this->serialPort	= serialPort;
	// Backup the required baudrate setting first
	backupBaudrate	= serialPort->getBaudrate();
	// Now try to lock onto the current baudrate setting of the servo
	// so we can talk to it
	speed_t		dummyBaudrate;
	if (!getBaudrate(&dummyBaudrate))
	{
		printf("Cannot find baud rate\n");
		return(false);
	}
	// Now set the baudrate to that required
	if (!setBaudrate(backupBaudrate))
	{
		printf("Cannot set required baud rate\n");
		return(false);
	}
	// Now configure the servos
	// ------------------------
	// set res
	if (!setRes(res))
	{
		printf("Cannot set resolution\n");
		return(false);
	}
	// Now configure the servos min/max pos
	if (!setBound(minPos,maxPos))
	{
		printf("Cannot set boundaries\n");
		return(false);
	}
	// Set the PD gains
	if (!setPDgain(Pgain,Dgain))
	{
		printf("Cannot set controller gains\n");
		return(false);
	}
	// Now initialise the position of the servo
	// setPos(127,DEFAULT_SPEED);
	// Show AI setup
	unsigned char ret_res,ret_minPos,ret_maxPos,ret_Pgain,ret_Dgain,ret_position;
	double ret_current;
	if (!getRes(&ret_res))
	{
		printf("Cannot get resolution\n");
		return(false);
	}
	if (!getBound(&ret_minPos,&ret_maxPos))
	{
		printf("Cannot get boundaries\n");
		return(false);
	}
	if (!getPDgain(&ret_Pgain,&ret_Dgain))
	{
		printf("Cannot get controller gains\n");
		return(false);
	}
	if (!update(&ret_position, &ret_current))
	{
		printf("Cannot get position and current\n");
		return(false);
	}
	printf("New AI servo created\n");
	printf("\tID: %d\n",ID);
	printf("\tBaudrate: %s\n",serialPort->baudrate2str(serialPort->getBaudrate()));
	printf("\tMin Pos: %d\n\tMax Pos: %d\n", ret_minPos, ret_maxPos);
	printf("\tP gain: %d\n\tD gain: %d\n", ret_Pgain, ret_Dgain);
	printf("\tCurrent Position: %d\n", ret_position);
	printf("\tCurrent: %3.3f\n\n", ret_current);
	fflush(stdout);
	return(true);
}



/***************************************************
* Public Functions: setPos
*	Set the position of the AI servo to the position
*	given (0 to 255). The speed at which the servo 
*	will move is given by mode or can be left out in
*	which case the servo will move at the slowest 
*	speed
***************************************************/
// Set posistion with a speed mode
bool AI::setPos(unsigned char position,unsigned char mode)
{
	unsigned char	currentByte,position_now;
	double			current;
	unsigned char 	data[4];
	
	// Parse the input
	if (position > 254)
	{
		printf("Invalid position\n");
		return(false);
	}
	// Reserve the serial port (TODO: add timeout)
	while (!serialPort->reserve(ID))
	{
	}
	// Send commands	
	data[0]		= HEADER;
	data[1]		= ((mode<<5)&0xe0) | ID;
	data[2]		= position;
	data[3]		= (data[1]^data[2])&0x7f;
	if (!serialPort->putBytes(data, 4, ID))
		return(false);
	// Read back the current & position but dont really care
	if (!serialPort->getByte(&currentByte,ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	if (!serialPort->getByte(&position_now,ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	current	 	= (double)(currentByte*18.4);
	// Free the serial port of other servos
	serialPort->free(ID);
	
	return(true);
}
// Set position using the default speed mode
bool AI::setPos(unsigned char pos)
{
	return(setPos(pos,DEFAULT_SPEED));
}



/***************************************************
* Public Functions: getPos
*	Returns the servos position between 0 & 255
*	this is unscaled as the configuration of the 
*	servo will affect this meaning
***************************************************/
// Return position
bool AI::getPos(unsigned char* position)
{
	double temp;
	return(update(position, &temp));
}



/***************************************************
* Public Functions: getCurrent
*	Returns the servo current in mA; This is related
*	to torque as well and can be throught of as the
*	stress level of the motor.
***************************************************/
// Return torque
bool AI::getCurrent(double* current)
{
	unsigned char temp;
	return(update(&temp, current));
}



/***************************************************
* Public Functions: update
*	This function reads the position and current 
*	from the servo.
***************************************************/
// Update the position and current of the servo
bool AI::update(unsigned char* position, double* current)
{
	unsigned char 	data[4];
	unsigned char 	currentByte;
	
	// Reserve the serial port (TODO: add timeout)
	while (!serialPort->reserve(ID))
	{
	}
	// Send commands
	data[0]		= HEADER;
	data[1]		= ((MOT_UPDATE<<5)&0xe0) | ID;
	data[2]		= 0;
	data[3]		= (data[1]^data[2])&0x7f;
	if (!serialPort->putBytes(data, 4, ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	// Read back the current & position
	if (!serialPort->getByte(&currentByte,ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	if (!serialPort->getByte(position,ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	*current	 = (currentByte)*18.4;			// in mA
	// Free the serial port for other servos
	serialPort->free(ID);
	
	return(true);
}



/***************************************************
* Public Functions: actDown
*	This function will remove the torque from the 
*	servos so they can be moved freely 
***************************************************/
// Remove the torque from the AI
bool AI::actDown()
{
	unsigned char	data[4];	
	
	// Reserve the serial port (TODO: add timeout)
	while (!serialPort->reserve(ID))
	{
	}
	// Send commands
	data[0]		= HEADER;
	data[1]		= ((MOT_ACTDOWN<<5)&0xe0) | ID;
	data[2]		= (1<<4)&0xf0;
	data[3]		= (data[1]^data[2])&0x7f;
	if (!serialPort->putBytes(data, 4, ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	// Clear the port by reading returned bytes (they are irrelivant)
	if (!serialPort->getByte(NULL,ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	if (!serialPort->getByte(NULL,ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	// Free the serial port for other servos
	serialPort->free(ID);
	
	return(true);
}



/***************************************************
* Public Functions: Power down
*	This function will stop __ALL__ AI motors until
*	a new position command is sent
***************************************************/
// Update to position and torque
bool AI::powerDown()
{
	unsigned char	data[4];	
	
	// Reserve the serial port (TODO: add timeout)
	while (!serialPort->reserve(ID))
	{
	}
	// Send commands
	data[0]		= HEADER;
	data[1]		= ((MOT_POWERDOWN<<5)&0xe0) | 31;
	data[2]		= (2<<4)&0xf0;
	data[3]		= (data[1]^data[2])&0x7f;
	if (!serialPort->putBytes(data, 4, ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	// Read bytes, but dont care about them
	if (!serialPort->getByte(NULL,ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	if (!serialPort->getByte(NULL,ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	// Free the serial port for other servos
	serialPort->free(ID);
	
	return(true);
}



/***************************************************
* Public Functions: Rotate 360 degrees
*	This function allows you to rotate the servo 
*	360 degrees at a speed between 0(stoped) to 15
*	(fastest)
***************************************************/
// Update to position and torque
bool AI::rotate(unsigned char speed, bool direction)
{
	unsigned char data[4];	
	unsigned char rotations, position;
	
	// Parse input
	if (speed > 15)
		return(false);						// Speed to high
	if (direction != CW | direction != CCW)
		return(false);						// Unkown direction
	// Reserve the serial port (TODO: add timeout)
	while (!serialPort->reserve(ID))
	{
	}
	// Send commands
	data[0]		= HEADER;
	data[1]		= ((MOT_ROTATE<<5)&0xe0) | ID;
	data[2]		= ((direction<<4)&0xf0) | (speed&0x0f);
	data[3]		= (data[1]^data[2])&0x7f;
	if (!serialPort->putBytes(data, 4, ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	// Read rotations and position (maybe used later)
	if (!serialPort->getByte(&rotations,ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	if (!serialPort->getByte(&position,ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	// Free the serial port for other servos
	serialPort->free(ID);
	
	return(true);
}


/***************************************************
* Public Functions: Set baudrate
* 	This function allows the user to set the baud
*	rate of the AI servo then sets the serial port 
*	to the requested baudrate and returns. NOTE:
*	only this servos baud will change so if you
*	update this servo you will need to change
*	the serial port back manually to update the
*	others.
***************************************************/
// Update to position and torque
bool AI::setBaudrate(speed_t baudrate)
{
	// First request a baud rate
	unsigned char data[6];
	unsigned char baudrateCode,readback_baudrateCode;
	
	// Parse the baudrate
	if (baudrate == B2400)
		baudrateCode	= 191;
	else if (baudrate == B4800)
		baudrateCode	= 95;
	else if (baudrate == B9600)
		baudrateCode	= 47;
	else if (baudrate == B19200)
		baudrateCode	= 23;
	else if (baudrate == B38400)
		baudrateCode	= 11;
	else if (baudrate == B57600)
		baudrateCode	= 7;
	else if (baudrate == B115200)
		baudrateCode	= 3;
	//else if (baudrate == B153600) // Doesnt seem to be supported in C++
	//	baudrateCode	= 2;
	else if (baudrate == B230400)
		baudrateCode	= 1;
	else if (baudrate == B460800)
		baudrateCode	= 0;
	else
	{
		printf("Unkown baudrate \n");
		return(false);
	};
	
	// Reserve the serial port (TODO: add timeout)
	while (!serialPort->reserve(ID))
	{
	}
	// Send commands
	data[0]		= HEADER;
	data[1]		= ((MOT_SETBAUD<<5)&0xe0) | ID;
	data[2]		= 0x08;
	data[3]		= baudrateCode;
	data[4]		= baudrateCode;
	data[5]		= (data[1]^data[2]^data[3]^data[4])&0x7f;
	if (!serialPort->putBytes(data, 6, ID))
	{
		serialPort->free(ID);
		return(false);
	}
	// Reconfigure the serial port at the new baud rate
	serialPort->setBaudrate(baudrate);
	// Read back the baudrate
	if (!serialPort->getByte(&readback_baudrateCode,ID))
	{
		serialPort->free(ID);
		return(false);
	}
	if (readback_baudrateCode != baudrateCode)
	{
		serialPort->free(ID);
		return(false);
	}
	if (!serialPort->getByte(&readback_baudrateCode,ID))
	{
		serialPort->free(ID);
		return(false);
	}
	if (readback_baudrateCode != baudrateCode)
	{
		serialPort->free(ID);
		return(false);
	}
	// Free the serial port for other servos
	serialPort->free(ID);
	
	return(true);
}




/***************************************************
* Public Functions: Set Baudrates
* 	This function trys to search for the correct
*	baudrate setting for the servo. It does this
*	by trying to call set baudrate with all the
*	possible baudrates until set baudrate is
*	successfull.
***************************************************/
// Set the PD gains
bool AI::getBaudrate(speed_t* baudrate)
{
	speed_t	currentBaudrate[] = BAUDRATE_TRYLIST;
	for (unsigned int k = 0; k<sizeof(currentBaudrate)/4; k++)
	{
		if (!serialPort->setBaudrate(currentBaudrate[k]))
		{
			printf("Unable to reset the baudrate of the serial port\n");
			return(false);
		}
		if (setBaudrate(currentBaudrate[k]))
		{
			*baudrate	= currentBaudrate[k];
			return(true);
		}
	}
	printf("Warning no baudrate found\n");
	return(false);
}



/***************************************************
* Public Functions: Set PD gains
*	This function will allow the user to set the PD
*	feedback gains on the servos. Note the default
*	P gain is 10 and the default D gain is 35. The 
*	manufacture suggests setting the gains between
*	1~50 for the P and 0~100 for the D however the
*	gains can be set outside this point if desired.
***************************************************/
// Set the PD gains
bool AI::setPDgain(unsigned char P, unsigned char D)
{
	unsigned char	data[6];
	unsigned char	newP, newD;	

	// Parse input
	if (P < 1 | P > 50)
		printf("Warning servo #%d's P gain set outside manufacturers recomendations\n Recomended values are between 1 and 50\n", ID);
	if (P > 100)
		printf("Warning servo #%d's D gain set outside manufacturers recomendations\n Recomended values are between 0 and 100\n", ID);
	
	// Reserve the serial port (TODO: add timeout)
	while (!serialPort->reserve(ID))
	{
	}
	// Send commands
	data[0]		= HEADER;
	data[1]		= ((MOT_SETGAINS<<5)&0xe0) | ID;
	data[2]		= 0x09;
	data[3]		= P;
	data[4]		= D;
	data[5]		= (data[1]^data[2]^data[3]^data[4])&0x7f;
	if (!serialPort->putBytes(data, 6, ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	// Read back gains
	if (!serialPort->getByte(&newP,ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	if (newP != P)
	{
		serialPort->free(ID);
		return(false);
	}		
	if (!serialPort->getByte(&newD,ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	if (newD != D)
	{
		serialPort->free(ID);
		return(false);
	}		

	// Free the serial port for other servos
	serialPort->free(ID);
	
	return(true);	
}



/***************************************************
* Public Functions: get the PD gains off the servos
*	This function is passed a pointer to two uchars 
*	used to store the P and D values read out of the
*	servos
***************************************************/
// Get the PD gains
bool AI::getPDgain(unsigned char* P, unsigned char* D)
{
	unsigned char	data[6];
	
	// Reserve the serial port (TODO: add timeout)
	while (!serialPort->reserve(ID))
	{
	}
	// Send commands
	data[0]		= HEADER;
	data[1]		= ((MOT_GETGAINS<<5)&0xe0) | ID;
	data[2]		= 0x0C;
	data[3]		= 0;
	data[4]		= 0;
	data[5]		= (data[1]^data[2]^data[3]^data[4])&0x7f;
	if (!serialPort->putBytes(data, 6, ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	// Read the gains
	if (!serialPort->getByte(P ,ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	if (!serialPort->getByte(D ,ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	// Free the serial port for other servos
	serialPort->free(ID);
	
	return(true);
}



/***************************************************
* Public Functions: Set Servos ID
*	Use this to change the servos ID and update that
*	info in this object.
***************************************************/
// Set the ID
bool AI::setID(unsigned char newID)
{
	unsigned char	data[6];
	unsigned char 	tempID;
	
	// Parse input
	if (newID > 30)
	{
		printf("Invalid new ID\n");
		return(false);
	}
	// Reserve the serial port (TODO: add timeout)
	while (!serialPort->reserve(ID))
	{
	}
	// Send commands
	data[0]		= HEADER;
	data[1]		= ((MOT_SETID<<5)&0xe0) | ID;
	data[2]		= 0x0A;
	data[3]		= newID;
	data[4]		= newID;
	data[5]		= (data[1]^data[2]^data[3]^data[4])&0x7f;
	if (!serialPort->putBytes(data, 6, ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	// Read back the ID
	if (!serialPort->getByte(&tempID,ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	if (tempID != ID)
	{
		serialPort->free(ID);
		return(false);
	}		
	if (!serialPort->getByte(&tempID,ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	if (tempID != ID)
	{
		serialPort->free(ID);
		return(false);
	}		
	// Free the serial port for other servos
	serialPort->free(ID);
	// Update the AI objects ID
	ID = newID;
	
	return(true);	
}



/***************************************************
* Public Functions: Set the Resolution of the Servo
* 	This function accepts an input either HIRES or
*	LOWRES to set the resoution of the motion.
***************************************************/
// Set the resolution of the AI servo
bool AI::setRes(unsigned char res)
{
	unsigned char	data[6];
	unsigned char	readback_res;
	
	// Parse input
	if (!(res == HIRES | res == LOWRES))
	{
		printf("Invalid resolution\n");
		return(false);
	}
	// Reserve the serial port (TODO: add timeout)
	while (!serialPort->reserve(ID))
	{
	}
	// Send commands
	data[0]		= HEADER;
	data[1]		= ((MOT_SETRES<<5)&0xe0) | ID;
	data[2]		= 0x0D;
	data[3]		= res;
	data[4]		= res;
	data[5]		= (data[1]^data[2]^data[3]^data[4])&0x7f;
	if (!serialPort->putBytes(data, 6, ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	// Read back the resolution
	if (!serialPort->getByte(&readback_res,ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	if (readback_res != res)
	{
		serialPort->free(ID);
		return(false);
	}		
	if (!serialPort->getByte(&readback_res,ID))
	{
		serialPort->free(ID);
		return(false);
	}		
	if (readback_res != res)
	{
		serialPort->free(ID);
		return(false);
	}		
	// Free the serial port for other servos
	serialPort->free(ID);
	
	return(true);
}

/***************************************************
* Public Functions: Read the Resolution of the Servo
*	This function is passed a pointer to the return
*	location of the resolution
***************************************************/
// Set the resolution of the AI servo
bool AI::getRes(unsigned char *res)
{
	unsigned char	data[6];

	// Reserve the serial port (TODO: add timeout)
	while (!serialPort->reserve(ID))
	{
	}
	// Send commands
	data[0]		= HEADER;
	data[1]		= ((MOT_READRES<<5)&0xe0) | ID;
	data[2]		= 0x0E;
	data[3]		= 0;
	data[4]		= 0;
	data[5]		= (data[1]^data[2]^data[3]^data[4])&0x7f;
	if (!serialPort->putBytes(data, 6, ID))
	{
		serialPort->free(ID);
		return(false);
	}
	// Read the resolution
	if (!serialPort->getByte(res,ID))
	{
		serialPort->free(ID);
		return(false);
	}
	if (!serialPort->getByte(res,ID))
	{
		serialPort->free(ID);
		return(false);
	}
	// Free the serial port for other servos
	serialPort->free(ID);
	
	return(res);
}



/***************************************************
* Public Functions: Set the current threshold 
*	This function sets the current threshold on the
*	servo using a byte = 0.054468085 * current limit
*	(in mA). Note if the current limit is reached 
*	the servo will go into act-down mode (ie fall
*	over)
***************************************************/
// Set the resolution of the AI servo
bool AI::setCurrentThreshold(double threshold)
{
	unsigned char	data[6];
	unsigned char	thresholdByte,readback_thresholdByte;
	
	// Parse Input
	thresholdByte	= (unsigned char)round(threshold*0.05446805); // From datasheet
	if (thresholdByte < 22 | thresholdByte > 54)	// corresponds to limits of 405mA ~ 993.6mA
	{
		printf("Invalid current limit");
		return(false);
	}
	// Reserve the serial port (TODO: add timeout)
	while (!serialPort->reserve(ID))
	{
	}
	// Send commands
	data[0]		= HEADER;
	data[1]		= ((MOT_SETCURTHRESH<<5)&0xe0) | ID;
	data[2]		= 0x0F;
	data[3]		= thresholdByte;
	data[4]		= thresholdByte;
	data[5]		= (data[1]^data[2]^data[3]^data[4])&0x7f;
	if (!serialPort->putBytes(data, 6, ID))
	{
		serialPort->free(ID);
		return(false);
	}
	// Read back the resolution
	if (!serialPort->getByte(&readback_thresholdByte,ID))
	{
		serialPort->free(ID);
		return(false);
	}
	if (readback_thresholdByte != thresholdByte)
	{
		serialPort->free(ID);
		return(false);
	}
	if (!serialPort->getByte(&readback_thresholdByte,ID))
	{
		serialPort->free(ID);
		return(false);
	}
	if (readback_thresholdByte != thresholdByte)
	{
		serialPort->free(ID);
		return(false);
	}
	// Free the serial port for other servos
	serialPort->free(ID);
	
	return(true);
}



/***************************************************
* Public Functions: Get the current threshold 
*	This function gets the current threshold on the
*	servo using current limit = byte * 18.4mA.
***************************************************/
// Return the over current threshold limit for the servo
bool AI::getCurrentThreshold(double* threshold)
{
	unsigned char	data[6];
	unsigned char	thresholdByte;
	
	// Reserve the serial port (TODO: add timeout)
	while (!serialPort->reserve(ID))
	{
	}
	// Send commands
	data[0]		= HEADER;
	data[1]		= ((MOT_SETCURTHRESH<<5)&0xe0) | ID;
	data[2]		= 0x10;
	data[3]		= 0;
	data[4]		= 0;
	data[5]		= (data[1]^data[2]^data[3]^data[4])&0x7f;
	if (!serialPort->putBytes(data, 6, ID))
	{
		serialPort->free(ID);
		return(false);
	}
	// Read the threshold
	if (!serialPort->getByte(&thresholdByte,ID))
	{
		serialPort->free(ID);
		return(false);
	}
	if (!serialPort->getByte(&thresholdByte,ID))
	{
		serialPort->free(ID);
		return(false);
	}
	*threshold	= (double)(thresholdByte*18.4);				// Current in mA
	// Free the serial port for other servos
	serialPort->free(ID);
	
	return(true);
}



/***************************************************
* Public Functions: Set the max/min position
*	This function sets the min and max positions to
*	which the servo can move
***************************************************/
// Set the bondaries of position for the servo
bool AI::setBound(unsigned char minPos, unsigned char maxPos)
{
	unsigned char	data[6];
	unsigned char 	readback_minPos, readback_maxPos;
	
	// Parse input
	if (minPos > 254 | maxPos > 254)
	{
		printf("Invalid position boundary\n");
		return(false);
	}

	// Reserve the serial port (TODO: add timeout)
	while (!serialPort->reserve(ID))
	{
	}
	// Send commands
	data[0]		= HEADER;
	data[1]		= ((MOT_SETBOUND<<5)&0xe0) | ID;
	data[2]		= 0x11;
	data[3]		= minPos;
	data[4]		= maxPos;
	data[5]		= (data[1]^data[2]^data[3]^data[4])&0x7f;
	if (!serialPort->putBytes(data, 6, ID))
	{
		serialPort->free(ID);
		return(false);
	}
	// Read back the resolution
	if (!serialPort->getByte(&readback_minPos,ID))
	{
		serialPort->free(ID);
		return(false);
	}
	if (readback_minPos != minPos)
	{
		serialPort->free(ID);
		return(false);
	}
	if (!serialPort->getByte(&readback_maxPos,ID))
	{
		serialPort->free(ID);
		return(false);
	}
	if (readback_maxPos != maxPos)
	{
		serialPort->free(ID);
		return(false);
	}
	// Free the serial port for other servos
	serialPort->free(ID);
	
	return(true);
}



/***************************************************
* Public Functions: Get the max/min position
*	This function gets the min and max positions to
*	which the servo can move
***************************************************/
// Get the bondaries of position for the servo
bool AI::getBound(unsigned char* minPos, unsigned char* maxPos)
{
	unsigned char	data[6];
	
	// Reserve the serial port (TODO: add timeout)
	while (!serialPort->reserve(ID))
	{
	}
	data[0]		= HEADER;
	data[1]		= ((MOT_GETBOUND<<5)&0xe0) | ID;
	data[2]		= 0x12;
	data[3]		= 0;
	data[4]		= 0;
	data[5]		= (data[1]^data[2]^data[3]^data[4])&0x7f;
	if (!serialPort->putBytes(data, 6, ID))
	{
		serialPort->free(ID);
		return(false);
	}
	// Read the threshold
	if (!serialPort->getByte(minPos,ID))
	{
		serialPort->free(ID);
		return(false);
	}
	if (!serialPort->getByte(maxPos,ID))
	{
		serialPort->free(ID);
		return(false);
	}
	// Free the serial port for other servos
	serialPort->free(ID);
	
	return(true);	
}

bool ang2uchar(double angle, unsigned char* angleChar, bool laserbackmount)
{	
	if(laserbackmount)
	{
		if (angle < MINTILT || angle > MAXTILT)
		{
			printf("%d is an invalid angle\n",angle);
			return(false);
		}
		*angleChar = (unsigned char)round(angle*TILT2UCHAR+127);
	}
	else
	{
		if (angle < MINPAN || angle > MAXPAN)
		{
			printf("%d is an invalid angle\n",angle);
			return(false);
		}
		*angleChar = (unsigned char)round(angle*PAN2UCHAR+127);
	}

	return(true);
}
// Converts from 0~255 to pan in degrees
bool uchar2ang(unsigned char angleChar, double* angle, bool laserbackmount)
{
	if(laserbackmount)
		*angle= ((double)angleChar-127)*UCHAR2TILT;
	else
		*angle= ((double)angleChar-127)*UCHAR2PAN;

	return(true);
}
