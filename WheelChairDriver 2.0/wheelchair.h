#ifndef WHEELCHAIR_H_
#define WHEELCHAIR_H_

#include "defs.h"
#include <libplayercore/playercore.h>
using namespace std;

class WheelChair
{
	public:
		WheelChair();
		WheelChair(ConfigFile* cf, int section);
		virtual ~WheelChair();
		int getLeftTicks();
		int getRightTicks();		
		int resetUnit();		
		int sendCommand(int WCcmd, bool param);
		int getReading(char Channel);
		void driveMotors(double xspeed, double yawspeed);
	private:
		WheelEncoder * leftEncoder, * rightEncoder;
	 	SerialCom * controlUnit;
		char serialPort[MAX_FILENAME_SIZE];
		int  serialRate,encoderRate;	 	
		char leftEncoderPort[MAX_FILENAME_SIZE];
		char rightEncoderPort[MAX_FILENAME_SIZE];	
		double oldxspeed,oldyaw;
};

#endif /*WHEELCHAIR_H_*/
