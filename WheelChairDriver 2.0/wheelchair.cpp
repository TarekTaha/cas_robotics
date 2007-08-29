#include "wheelchair.h"

WheelChair::WheelChair()
{
}

WheelChair::WheelChair(ConfigFile* cf, int section)
{
	leftEncoder = NULL; rightEncoder = NULL;
	strncpy(leftEncoderPort ,cf->ReadString(section, "Left_Encoder_Port", ENCL_DEFAULT_PORT), sizeof(leftEncoderPort));
	strncpy(rightEncoderPort,cf->ReadString(section, "Right_Encoder_Port", ENCR_DEFAULT_PORT), sizeof(rightEncoderPort));
	strncpy(serialPort,      cf->ReadString(section, "Serial_Port", SHRD_DEFAULT_PORT), sizeof(serialPort));
	this->encoderRate =		 cf->ReadInt(section, "Encoder_Baud_Rate", ENC_DEFAULT_RATE);
	this->serialRate  =      cf->ReadInt(section, "Serial_Baud_Rate", SHRD_DEFAULT_RATE);

	leftEncoder  = new WheelEncoder(leftEncoderPort , encoderRate);
	rightEncoder = new WheelEncoder(rightEncoderPort, encoderRate);
	controlUnit  = new SerialCom(serialPort,serialRate);	
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
	unsigned int retbyte;
	//Resetting Motor and Joystick Voltages//
	controlUnit->Write("O0000\r", 6);
	controlUnit->ReadByte(&retbyte);
	controlUnit->ReadByte(&retbyte);
	controlUnit->Write("L0800\r", 6);
	controlUnit->ReadByte(&retbyte);
	controlUnit->ReadByte(&retbyte);
	controlUnit->Write("L1800\r", 6);
	controlUnit->ReadByte(&retbyte);
	controlUnit->ReadByte(&retbyte);	
	sendCommand(POWER,OFF);	
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

	if (controlUnit->SendCommand("O0000") != 0) //Reset output and wait response
	{
		PLAYER_ERROR("\n	-->Failed to find Wheelchair Interface Unit");
		return -1;
	}
	else 
		puts("\n- Wheelchair Interface Unit Found and Reseted.");
	controlUnit->ReadByte(&retbyte);
	if (controlUnit->SendCommand("L0800") != 0) //Set fwd/rev to 2.5v
	{       
		PLAYER_ERROR("\n	-->Failed to set fwd/rev voltage to 2.5V");
		return -1;
	}
	else
		puts("		+ Forward/Reverse Voltage was successfully set to 2.5V");

	if (controlUnit->SendCommand("L1800") != 0) //Set left/right to 2.5v
	{       
		PLAYER_ERROR("\n	-->Failed to set l/r voltage to 2.5V");
		return -1;
	}
	else
		puts("		+ Left/Right Voltage was successfully set to 2.5V");
	return 0;
};

int WheelChair::sendCommand(int WCcmd, bool param) 
{
	static unsigned char status;
	char cmd[7];
	unsigned int retbyte, retbyte2;
	bool power = false;
	switch (WCcmd) 
	{
		case POWER:  
				if ((getReading('A') + getReading('E')) > 1500)
				{ 
					power = true;
					//this->power=ON;
				}
//				else
//					this->power=OFF;
				printf("\n	+-->Processing Power Request ");
				if ((param && !power) || (!param && power)) 
				{
					printf("\n	+-->Trying to toggle the power");
					// Toggle wheelchair power on
					if (controlUnit->SendCommand("O0008") != 0) 
					{
						PLAYER_ERROR("Failed to set power\n");
						controlUnit->SendCommand("O0000");
						return -1;
					}
					usleep(SLEEP);
					if (controlUnit->SendCommand("O0000") != 0)
					{
						PLAYER_ERROR("Failed to set power\n");
						return -1;
					}
				}
			break;
		
		case SETMODE:
				sprintf(cmd, "U8\r");
				controlUnit->Write(cmd, 3);
				printf("\n	+-->Processing Set Mode Request !!!");
				controlUnit->ReadByte(&retbyte);
				if ((char)retbyte != 'U') 
				{
					printf("\nReturned: .%c.\n",(char)retbyte);
					PLAYER_ERROR("Failed to read auto/man status from Wheelchair Interface Unit\n");
					return -1;
				}
				controlUnit->ReadByte(&retbyte);
				controlUnit->ReadByte(&retbyte2);
				controlUnit->ReadByte(&retbyte);
				controlUnit->ReadByte(&retbyte);
				controlUnit->ReadByte(&retbyte);
				
				if (((char)retbyte2 == '0' && param) || ((char)retbyte2 != '0' && !param)) 
				{
					// we need to toggle the mode
					if (controlUnit->SendCommand("O0020") != 0) 
					{	
						PLAYER_ERROR("Failed to set mode\n");
						controlUnit->SendCommand("O0000");
						return -1;
					}
					usleep(LATCHDELAY);
					if (controlUnit->SendCommand("O0000") != 0) 
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

int WheelChair::getReading(char Channel) 
{
	int Count = 0;
	unsigned int temp;
	int ret;
	unsigned int Position = 0;
		controlUnit->WriteByte(0x55);
		controlUnit->WriteByte(Channel);
		controlUnit->WriteByte(13);

		temp = 0;
		ret = controlUnit->ReadByte(&temp); 
		if (ret > 0) Count++; 
		if (ret < 0) fprintf(stderr,"Wheelchair: error reading data (%d - %s)\n",errno,strerror(errno));
		if (temp != 0x55)			
			return -1;

		temp = 0;
		ret = controlUnit->ReadByte(&temp); 
		if (ret > 0) Count++; 
		if (ret < 0) fprintf(stderr,"Wheelchair: error reading data (%d - %s)\n",errno,strerror(errno));
		if ((char)temp != Channel)			
			return -1;
		
		temp = 0;
		ret = controlUnit->ReadByte(&temp);
		if (ret > 0) Count++;
		if (ret < 0) fprintf(stderr,"Wheelchair: error reading data (%d - %s)\n",errno,strerror(errno));
		temp -= 48;
		if (temp > 9)
			temp -= 7; 
		temp = temp << 8;
		Position = temp;
	
		temp = 0;
		ret = controlUnit->ReadByte(&temp); 
		if (ret > 0) Count++; 
		if (ret < 0) fprintf(stderr,"Wheelchair: error reading data (%d - %s)\n",errno,strerror(errno));
		temp -= 48;
		if (temp > 9)
			temp -= 7; 
		temp = temp << 4;
		Position |= temp;
	
		temp = 0;
		ret = controlUnit->ReadByte(&temp); 
		if (ret > 0) Count++; 
		if (ret < 0) fprintf(stderr,"Wheelchair: error reading data (%d - %s)\n",errno,strerror(errno));
		temp -= 48;
		if (temp > 9)
			temp -= 7;
		Position |= temp;
	
		ret = controlUnit->ReadByte(&temp); 
	if (Count == 5)
		return Position;
	else
		return -1;
};

/*!
 * Gear one (1) curve fitting voltage function, it goes upto 0.33 m/sec only
 */
void WheelChair::driveMotors(double xspeed, double yawspeed) 
{
	double yawval, xval;
	char cmd[7];
	if ((xspeed != oldxspeed) || (yawspeed != oldyaw)) 
	{
		oldxspeed = xspeed;
		oldyaw = yawspeed;
		xval= -14217*xspeed*xspeed*xspeed-1875.5*xspeed*xspeed + 5652.2*xspeed + 1948.7; //trying to save computational time but avoiding pow math function
		yawval   = 183.44*yawspeed*yawspeed*yawspeed - 19.158*yawspeed*yawspeed - 1147.5*yawspeed + 2019.6;

		// Boundary Checking, Joystick Blocks if voltage limits are exceeded
		//cout<<"\nXval:"<<xval<<" Yawval:"<<yawval;
		if (xval > 3200.0) 
			xval=3200.0;
		else if (xval < 850.0) 
			xval= 850.0;
	
		if(xspeed>0.33)	
		{
			xval=3200.0;
		}
		else if(xspeed < -0.2)
		{
			xval= 850.0;
		}	
		else if (xspeed == 0.0) 
		{
			xval=1987.0;
		}
		
		if (yawval > 3200.0   ) 
		{
			yawval   = 3200.0;
		}
		else if (yawval <  850.0 ) 
		{
			yawval   = 850.0;
		}
		else if (yawspeed == 0.0 ) 
		{
			yawval = 1994.0;
		}
		
		sprintf(cmd, "L1%03X", (int)yawval);
		controlUnit->SendCommand(cmd);
		sprintf(cmd, "L0%03X", (int)xval);
		controlUnit->SendCommand(cmd);
//		printf("\n Xval=%.3f Yval=%.3f",xval,yawval);
		fflush(stdout);
	}
};

