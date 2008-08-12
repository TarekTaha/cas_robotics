#include "wheelchair.h"

WheelChair::WheelChair()
{
}

WheelChair::WheelChair(ConfigFile* cf, int section)
{
	leftEncoder = NULL; rightEncoder = NULL;
	strncpy(leftEncoderPort ,cf->ReadString(section, "Left_Encoder_Port", ENCL_DEFAULT_PORT), sizeof(leftEncoderPort));
	strncpy(rightEncoderPort,cf->ReadString(section, "Right_Encoder_Port", ENCR_DEFAULT_PORT), sizeof(rightEncoderPort));
	strncpy(serialPortName,      cf->ReadString(section, "Serial_Port", SHRD_DEFAULT_PORT), sizeof(serialPortName));
	this->encoderRate =	 cf->ReadInt(section, "Encoder_Baud_Rate", ENC_DEFAULT_RATE);
	this->serialRate  =      cf->ReadInt(section, "Serial_Baud_Rate", SHRD_DEFAULT_RATE);

	leftEncoder  = new WheelEncoder(leftEncoderPort , encoderRate);
	rightEncoder = new WheelEncoder(rightEncoderPort, encoderRate);
	controlUnit  = new SerialCom(serialPortName,serialRate);
	this->power = false;
}

int WheelChair::getLeftTicks()
{
	return leftEncoder->GetTicks();
}

int WheelChair::getRightTicks()
{
	return rightEncoder->GetTicks();
}

WheelChair::~WheelChair()
{
	puts("\n- Shutting Down WheelChair Driver");
//	unsigned int retbyte;
	//Resetting Motor and Joystick Voltages//
/*
	controlUnit->Write("O0000\r", 6);
	controlUnit->ReadByte(&retbyte);
	controlUnit->ReadByte(&retbyte);
	controlUnit->Write("L0800\r", 6);
	controlUnit->ReadByte(&retbyte);
	controlUnit->ReadByte(&retbyte);
	controlUnit->Write("L1800\r", 6);
	controlUnit->ReadByte(&retbyte);
	controlUnit->ReadByte(&retbyte);
*/
	cout <<"\n--->> WheelChair's Power is turned OFF\n";
	fflush(stdout);		
	delete controlUnit;
	delete leftEncoder;
	delete rightEncoder;
	controlUnit = NULL;
	leftEncoder = NULL;
	rightEncoder= NULL;	
}

int WheelChair :: resetUnit()
{
	unsigned int retbyte;
	printf("\nResetting unit"); fflush(stdout);
	if (controlUnit->SendCommand((char*)"O0000") != 0) //Reset output and wait response
	{
		PLAYER_ERROR("\n	-->Failed to find Wheelchair Interface Unit");
		return -1;
	}
	else 
		puts("\n- Wheelchair Interface Unit Found and Reseted.");
	controlUnit->ReadByte(&retbyte);
	if (controlUnit->SendCommand((char*)"L0800") != 0) //Set fwd/rev to 2.5v
	{       
		PLAYER_ERROR("\n	-->Failed to set fwd/rev voltage to 2.5V");
		return -1;
	}
	else
		puts("		+ Forward/Reverse Voltage was successfully set to 2.5V");

	if (controlUnit->SendCommand((char*)"L1800") != 0) //Set left/right to 2.5v
	{       
		PLAYER_ERROR("\n	-->Failed to set l/r voltage to 2.5V");
		return -1;
	}
	else
		puts("		+ Left/Right Voltage was successfully set to 2.5V");
	printf("-->Unit Reset"); fflush(stdout);
	return 0;
};

int WheelChair::sendCommand(int WCcmd, bool param) 
{
	static unsigned char status;
	unsigned int retBytes[7];
	char cmd[7];
	unsigned int retbyte, retbyte2;
	int voltageReadings,a,e;
	switch (WCcmd) 
	{
		case POWER:
				while((a=getReading('A'))==-1);{usleep(SLEEP);}
				while((e=getReading('E'))==-1);{usleep(SLEEP);}
				voltageReadings = a + e;
				//printf("\n Voltage Readings=%d",voltageReadings); fflush(stdout);
				if ( voltageReadings > 2000)
					power = true;
				else
					power = false;

				if ((power && param) || (!power && !param)) // Same power state -> nothing to change
					break;
				if((!power && param) || (power && !param))
				{
					while(controlUnit->SendCommand((char*)"O0008") != 0) {usleep(SLEEP);}
					usleep(SLEEP);
					while(controlUnit->SendCommand((char*)"O0000") != 0) {usleep(SLEEP);}
					(power)?power = false:power = true;
				}
				//printf("\nPower is:%d",power); fflush(stdout);
			break;
		
		case SETMODE:
				sprintf(cmd, "U8\r");
				controlUnit->Write(cmd, 3);
				printf("\n	+-->Processing Set Mode Request !!!");
				controlUnit->Read(retBytes,6);
				if ((char)retBytes[0] != 'U') 
				{
					printf("\nReturned: .%c.\n",(char)retbyte);
					PLAYER_ERROR("Failed to read auto/man status from Wheelchair Interface Unit\n");
					return -1;
				}

				if (((char)retBytes[2] == '0' && param) || ((char)retBytes[2] != '0' && !param)) 
				{
					// we need to toggle the mode
					if(controlUnit->SendCommand((char*)"O0020") != 0)
					{	
						PLAYER_ERROR("Failed to set mode\n");
						controlUnit->SendCommand((char*)"O0000");
						return -1;
					}
					usleep(LATCHDELAY);
					if(controlUnit->SendCommand((char*)"O0000") != 0)
					{
						PLAYER_ERROR("Failed to set mode\n");
						return -1;
					}
				}
				break;
		case GEAR:
				printf("\n	+-->Processing Gear Request /");
				if (param) 
				{
					printf(" Going one Gear UP");
					// Gear up
					status |= 0x02;
					sprintf(cmd,"O00%02X", status);
					if(controlUnit->SendCommand(cmd)!=0) 
					{
						PLAYER_ERROR("Failed to increment gear\n");
						return -1;
					}
					usleep(SLEEP);
					usleep(SLEEP);
					usleep(SLEEP);
					status &= ~0x02;
					sprintf(cmd,"O00%02X", status);
					if(controlUnit->SendCommand(cmd)!=0) 
					{
						PLAYER_ERROR("Failed to increment gear\n");
						return -1;
					}
				}
				else 
				{
					// Gear down
					printf(" Going one Gear Down");
					status |= 0x01;
					sprintf(cmd,"O00%02X", status);
					if(controlUnit->SendCommand(cmd)!=0) 
					{
						PLAYER_ERROR("Failed to decrement gear\n");
						return -1;
					}
					usleep(SLEEP);
					usleep(SLEEP);
					usleep(SLEEP);
					status &= ~0x01;
					sprintf(cmd,"O00%02X", status);
					if(controlUnit->SendCommand(cmd)!=0) 
					{
						PLAYER_ERROR("Failed to decrement gear\n");
						return -1;
					}
				}
				break;
		
		case HORN:   
				printf("\n	+-->Processing Horn Request !!!");
				//if (param) 
				{
					// Turn horn on
					status |= 0x04;
					sprintf(cmd,"O00%02X", status);
					if(controlUnit->SendCommand(cmd)!=0) 
					{
						PLAYER_ERROR("Failed to enable horn\n");
						return -1;
					}
				}
			 	//else 
				usleep(100000);
				{
					// Turn horn off
					status &= ~0x04;
					sprintf(cmd,"O00%02X", status);
					if(controlUnit->SendCommand(cmd)!=0) 
					{
						PLAYER_ERROR("Failed to enable horn\n");
						return -1;
					}
				}
			break;
		case GETMODE:
				printf("\n	+-->Processing Get Mode Request ->");
				sprintf(cmd, "U8\r");
				controlUnit->Write(cmd, 3);
				controlUnit->ReadByte(&retbyte);
				if ((char)retbyte != 'U') 
				{
					PLAYER_ERROR("\n	-->Failed to read auto/man status from Wheelchair Interface Unit\n");
					return -1;
				}
				controlUnit->ReadByte(&retbyte);
				controlUnit->ReadByte(&retbyte2);
				controlUnit->ReadByte(&retbyte);
				controlUnit->ReadByte(&retbyte);
				controlUnit->ReadByte(&retbyte);
				
				if ((char)retbyte2 == '0')
				{
					printf("Control is  Manual");
					return (int) MANUAL;
				}
				else
				{
					printf("Control is  Automatic");
					return (int) AUTO;	
				}		
			break;
		default:
			PLAYER_ERROR("Unknown command\n");
			return -1;
	}
	return 0;
	
};

int WheelChair::getReading(char channel)
{
	int count = 0;
	unsigned int readingValue[8];
	unsigned int Position = 0;
	controlUnit->WriteByte('U'); controlUnit->WriteByte(channel); controlUnit->WriteByte(13);
	count = controlUnit->Read(readingValue,6);
	/*for(int i=0;i<count;i++)
	{
		printf("%c",readingValue[i]);
	}*/
	if(count!=6 || (readingValue[0]&=0x000000ff)!='U')
		return -1;

	readingValue[2]&=0x000000ff;readingValue[3]&=0x000000ff;readingValue[4]&=0x000000ff;
//	printf("\nBefore Conv %d %d %d",readingValue[2],readingValue[3],readingValue[4]);
	readingValue[2]-=48; if(readingValue[2]>9) readingValue[2]-=7;
	Position |= (readingValue[2]<<8);
//	printf("\n Converted Value is:%d",Position);
	readingValue[3]-=48; if(readingValue[3]>9) readingValue[3]-=7;
	Position |= (readingValue[3]<<4);
//	printf(" Converted Value is:%d",Position);
	readingValue[4]-=48; if(readingValue[4]>9) readingValue[4]-=7;
	Position |= (readingValue[4]);
//	printf(" Converted Value is:%d",Position);
	return Position;
};

/*!
 * Gear one (1) curve fitting voltage function, it applies to speeds between [-0.2 0.33] m/sec only
 */
void WheelChair::driveMotors(double xspeed, double yawspeed) 
{
	double yawval, xval;
	char cmd[7];
	if ((xspeed != oldxspeed) || (yawspeed != oldyaw)) 
	{
		oldxspeed = xspeed;
		oldyaw = yawspeed;
		// Boundary Checking, Joystick Blocks if voltage limits are exceeded
		if(xspeed > 0.33)	
		{
			xval = 2800.0;
		}
		else if(xspeed < -0.2)
		{
			xval = 850.0;
		}
		else if (xspeed == 0.0) 
		{
			xval = 1987.0;
		}
		else
		{
			//trying to save computational time by avoiding pow math function			
			xval = -14217*xspeed*xspeed*xspeed-1875.5*xspeed*xspeed + 5652.2*xspeed + 1948.7; 
			if (xval > 2800.0) 
				xval=2800.0;
			else if (xval < 850.0) 
				xval= 850.0;
		}
		if(yawspeed < -1.7)	
		{
			yawval = 2800.0;
		}
		else if (yawspeed == 0.0 ) 
		{
			yawval = 1994.0;
		}	
		else
		{
			//trying to save computational time by avoiding pow math function
			yawval   = 183.44*yawspeed*yawspeed*yawspeed - 19.158*yawspeed*yawspeed - 1147.5*yawspeed + 2019.6;		
			if (yawval > 2800.0   ) 
			{
				yawval   = 2800.0;
			}
			else if (yawval <  850.0 ) 
			{
				yawval   = 850.0;
			}
		}
		sprintf(cmd, "L1%03X", (int)yawval);
		controlUnit->SendCommand(cmd);
		sprintf(cmd, "L0%03X", (int)xval);
		controlUnit->SendCommand(cmd);
//		printf("\n Xspeed=%.3f Xval=%.3f Yspeed=%.3f Yval=%.3f",xspeed,xval,yawspeed,yawval);
		fflush(stdout);
	}
};

