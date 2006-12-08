#ifndef AI_H
#define AI_H

/***************************************************
* Title			: AI - AI servo object
* Project 		: RoboRescue Victum Identification
* Revision		: 0.1
* Author		: Oliver Thane (01058902)
* Date			: 12/01/2006
* Description	:
*	This class allows access to the AI servomotors
*	used in the pan-tilt unit on the HOMER.
*	
***************************************************/



/***************************************************
* Include files required
***************************************************/
#include "simpleSerialComms.h"	// For talking to the motor
#include <stdio.h>					// For general IO
#include <math.h>					// For scaling data
#include <sys/time.h>				// For checking timeout on read


/***************************************************
* Definitions
***************************************************/
#define HEADER		 		0xff
#define MOT_SPEED0   		0          	// Motor speeds from Tribotix examples
#define MOT_SPEED1   		1
#define MOT_SPEED2   		2
#define MOT_SPEED3   		3
#define MOT_SPEED4   		4
#define DEFAULT_SPEED 		MOT_SPEED4	// Slowest speed for default move
#define MOT_UPDATE	 		5
#define MOT_ACTDOWN			6
#define MOT_POWERDOWN		6
#define	MOT_ROTATE	 		6
#define MOT_SETBAUD			7
#define MOT_SETGAINS		7
#define MOT_SETID			7
#define MOT_GETGAINS		7
#define MOT_SETRES			7
#define	MOT_READRES			7
#define MOT_SETCURTHRESH	7
#define MOT_GETCURTHRESH	7
#define MOT_SETBOUND		7
#define MOT_GETBOUND		7
#define HIRES 				1	
#define LOWRES				0
#define CW			 		4
#define CCW			 		3
#define BAUDRATE_TRYLIST 	{B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800}

// Converts from pan in degrees to 0~254
#define MINPAN	   -166
#define MAXPAN		166
#define MINTILT	   -83
#define MAXTILT		83
#define	UCHAR2PAN	1.307
#define PAN2UCHAR	0.765
#define UCHAR2TILT	0.654
#define TILT2UCHAR	1.529


/***************************************************
* Prototypes
***************************************************/
class AI
{
	public:
		// Constructors/Destructor
		AI();
		AI(unsigned char ID, simpleSerialComms* serialPort);
		AI(unsigned char ID, simpleSerialComms* serialPort, unsigned char res, unsigned char minPos, unsigned char maxPos, unsigned char Pgain, unsigned char Dgain);		  
		~AI();
		// Initialise
		bool				initalise(unsigned char ID, simpleSerialComms* serialPort, unsigned char res, unsigned char minPos, unsigned char maxPos, unsigned char Pgain, unsigned char Dgain);
		// Get/Set position commands
		bool				setPos(unsigned char pos,unsigned char mode);
		bool				setPos(unsigned char pos);
		bool				getPos(unsigned char* pos);
		bool				getCurrent(double* current);
		bool				update(unsigned char*pos, double* current);
		// Functions for talking to AI servo
		bool				actDown();
		bool				powerDown();
		bool				rotate(unsigned char speed, bool direction);
		bool				setBaudrate(size_t baudrate);
		bool				getBaudrate(size_t* baudrate);
		bool				setID(unsigned char newID);
		bool				setPDgain(unsigned char P, unsigned char D);	
		bool				getPDgain(unsigned char* P, unsigned char* D);
		bool				setRes(unsigned char res);
		bool				getRes(unsigned char* res);
		bool				setCurrentThreshold(double threshold);
		bool				getCurrentThreshold(double* threshold);
		bool				setBound(unsigned char minPos, unsigned char maxPos);
		bool				getBound(unsigned char* minPos, unsigned char* maxPos);
	private:
		// Variables
		unsigned char		ID;
		simpleSerialComms*	serialPort;	
};

bool ang2uchar(double angle, unsigned char* angleChar, bool laserbackmount);
bool uchar2ang(unsigned char angleChar, double* angle, bool laserbackmount);

#endif
