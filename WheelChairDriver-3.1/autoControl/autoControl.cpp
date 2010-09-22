#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <strings.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <math.h>

#include <signal.h>
#include <netinet/in.h>
#include <termios.h>

#include "defs.h"
#include "SDL/SDL.h"
//#define debug
#define log
  #ifdef log
	FILE *file;
  #endif
static double oldxspeed, oldyawspeed,px,py,pa;
char Left_Encoder_Port[50];
char Right_Encoder_Port[50];
int first_ltics,first_rtics,last_ltics, last_rtics;
struct timeval last_time;

void UpdateOdom(int ltics, int rtics) 
{
    double delta_left, delta_right, theta, mid_d,sum,diff,circular_r,left_speed,right_speed;
    struct timeval currtime;
    double timediff;
    gettimeofday(&currtime,NULL);
    timediff = (currtime.tv_sec + currtime.tv_usec/1e6) - (last_time.tv_sec + last_time.tv_usec/1e6);
    last_time = currtime;
    delta_left = (ltics - last_ltics) * M_PER_TICK;    //  Distance travelled by left  wheel since last reading
    delta_right =(rtics - last_rtics) * M_PER_TICK;    //  Distance travelled by Right wheel since last reading
    left_speed = delta_left /timediff;
    right_speed= delta_right/timediff;
    sum= delta_right + delta_left;
    diff=delta_right - delta_left;
    theta = (diff/AXLE_LENGTH);                     //  Angle Theta in Radians normalized between -pi and pi
    mid_d = sum / 2.0;                                       //  Distance travelled by mid point
    if ( diff==0 )//If we are moving straight, a specail case is needed cause our formula will crash since denomenator will be 0
    {
	px += (delta_left  * cos(pa)); // in meters
	py += (delta_right * sin(pa)); // in meters
	pa += (theta);   // no need for this in case of straight line -> delta_theta=0 but just for illustration i kept it
    }
    else // we are moving in an arc
    {
	circular_r = mid_d / theta ;                //  Circular Radius of Trajectory of robot's center	
	px +=  circular_r * (sin(theta + pa) - sin(pa) ); // in meters
	py -=  circular_r * (cos(theta + pa) - cos(pa) ); // in meters
	pa +=  theta;
    }
    //pa=NORMALIZE(pa); // Normalize the angle to be between -pi and +pi Normalize(z)= atan2(sin(z),cos(z))
#ifdef debug
    printf("\nDelta L=%5.4lf Delta R=%5.4lf Sum=%5.4lf Diff=%5.4lf Theta=%5.4lf Mid=%5.4lf Cir_r=%5.4lf", delta_left, delta_right, sum,diff,theta,mid_d,circular_r);
    printf("\nLastL=%d LastR=%d LTicks= %d RTicks= %d LSpeed= %5.2lf m/sec RSpeed= %5.2lf m/sec diff=%lf",last_ltics,last_rtics,ltics,rtics,left_speed,right_speed,timediff);
    printf("\nWHEELCHAIR: pose: %5.2lf,%5.2lf,%5.2lf", px,py,RTOD(pa));
#endif
#ifdef log
    fprintf(file,"%d %d %lf %lf %lf\n",ltics,rtics,left_speed,right_speed,timediff);
    //fprintf(file,"%d %d %lf %lf %lf\n",ltics,rtics,px,py,RTOD(pa));
#endif
    last_ltics = ltics;
    last_rtics = rtics;
}

void UpdateMotors(double xspeed, double yaw) 
{
    //double yawval, xval;
    char cmd[7];
    if ((xspeed != oldxspeed) || (yaw != oldyawspeed))
    {
        oldxspeed = xspeed;
        oldyawspeed = yaw;
        //if (xspeed > 3200) xspeed=3200;
        //if (xspeed < 850  & xspeed!=0) xspeed= 850;
        if (xspeed == 0) xspeed=1987;

        //if (yaw > 3200   ) yaw   = 3200;
        //if (yaw <  850   & yaw!=0 ) yaw   = 850;
        if (yaw == 0 ) yaw = 1994;
        /*
		  I found that this is non sense, there is no need to convert to the -100 ,500 system and
		  back to the actual system if the motor accepts only actual system
		  
		yawval = 25.6 * (float)yaw + 2048;
		
		if (xspeed < 0) 
			xval = 11.52 * (float)xspeed + 2048;
		else    
			xval = 2.304 * (float)xspeed + 2048;

		// Check to see values are within bounds. If these values are exceeded,
		// the hand controller may lock up.
		// c00 was c80, 3f0 was 380
		// 0xc80 = to 3200 Decimal and 0x380 = 896 

		if (xval > 0xC80) 
			xval = 0xC80;
		else 
			if (xval < 0x380) 	
				xval = 0x380;

		if (yawval > 0xC80) 
			yawval = 0xC80;
		else 
			if (yawval < 0x380) 
				yawval = 0x380;
		*/
        lock;
        sprintf(cmd, "L1%03X", (int)yaw);
        printf("\n Cmd=%s",cmd);
        SharedSerial->SendCommand(cmd);
        sprintf(cmd, "L0%03X", (int)xspeed);
        printf("\n Cmd=%s",cmd);
        SharedSerial->SendCommand(cmd);
        printf("\n-->Motors Updated with XSpeed=%.2f Yspeed=%.2f",xspeed,yaw);
        unlock;
    }
    fflush(stdout);
}

void Motor(double xspeed, double yaw) 
{
    //double yawval, xval;
    char cmd[7];
    if ((xspeed != oldxspeed) || (yaw != oldyawspeed))
    {
        oldxspeed = xspeed;
        oldyawspeed = yaw;
        //xspeed= -pow(1.4,4)*pow(xspeed,3)-pow(1.9,3)*pow(xspeed,2) + pow(5.7,3)*xspeed + pow(1.9,3);// i got this equation from matlab after modeling the velocity
        xspeed= -14217*pow(xspeed,3)-1875.5*pow(xspeed,2) + 5652.2*xspeed + 1948.7;// i got this equation from matlab after modeling the velocity
        printf("\n-->Motors Updated with XSpeed=%.2f Yspeed=%.2f",xspeed,yaw);
        if (xspeed > 3200) xspeed=3200;
        else if ((xspeed < 850)  & (xspeed!=0)) xspeed= 850;
        else if (xspeed == 0) xspeed=1987;

        if (yaw > 3200   ) yaw   = 3200;
        else if ((yaw <  850)   & (yaw!=0) ) yaw   = 850;
        else if (yaw == 0 ) yaw = 1994;
        sprintf(cmd, "L1%03X", (int)yaw);
        printf("\n Cmd=%s",cmd);
        SharedSerial->SendCommand(cmd);
        sprintf(cmd, "L0%03X", (int)xspeed);
        printf("\n Cmd=%s",cmd);
        SharedSerial->SendCommand(cmd);
        //printf("\n-->Motors Updated with XSpeed=%.2f Yspeed=%.2f",xspeed,yaw);
    }
    fflush(stdout);
}

int GetReading(char Channel)
{
    int Count = 0;
    unsigned int temp;
    int ret;
    unsigned int Position = 0;
    lock;
    SharedSerial->WriteByte(0x55);
    SharedSerial->WriteByte(Channel);
    SharedSerial->WriteByte(13);

    temp = 0;
    ret = SharedSerial->ReadByte(&temp);
    if (ret > 0) Count++;
    if (ret < 0) fprintf(stderr,"Wheelchair: error reading data (%d - %s)\n",errno,strerror(errno));
    if (temp != 0x55){

        return -1;
    }

    temp = 0;
    ret = SharedSerial->ReadByte(&temp);
    if (ret > 0) Count++;
    if (ret < 0) fprintf(stderr,"Wheelchair: error reading data (%d - %s)\n",errno,strerror(errno));
    if ((char)temp != Channel){

        return -1;
    }

    temp = 0;
    ret = SharedSerial->ReadByte(&temp);
    if (ret > 0) Count++;
    if (ret < 0) fprintf(stderr,"Wheelchair: error reading data (%d - %s)\n",errno,strerror(errno));
    temp -= 48;
    if (temp > 9)
        temp -= 7;
    temp = temp << 8;
    Position = temp;

    temp = 0;
    ret = SharedSerial->ReadByte(&temp);
    if (ret > 0) Count++;
    if (ret < 0) fprintf(stderr,"Wheelchair: error reading data (%d - %s)\n",errno,strerror(errno));
    temp -= 48;
    if (temp > 9)
        temp -= 7;
    temp = temp << 4;
    Position |= temp;

    temp = 0;
    ret = SharedSerial->ReadByte(&temp);
    if (ret > 0) Count++;
    if (ret < 0) fprintf(stderr,"Wheelchair: error reading data (%d - %s)\n",errno,strerror(errno));
    temp -= 48;
    if (temp > 9)
        temp -= 7;
    Position |= temp;

    ret = SharedSerial->ReadByte(&temp);
    //printf("\nReading Joystick.. %d %d, ", (Position), Count);
    unlock;

    if (Count == 5)
        return Position;
    else
        return -1;
}

int Control(int WCcmd, bool param)
{
    static unsigned char status;
    char cmd[7];
    unsigned int retbyte, retbyte2;
    bool power = false;
    lock;
    switch (WCcmd) {
    case POWER:
        if ((GetReading('A') + GetReading('E')) > 1500)
        {
            power = true;
        }
        if ((param && !power) || (!param && power))
        {
            printf("\nTrying to toggle the power");
            // Toggle wheelchair power on
            if (SharedSerial->SendCommand((char *)"O0008") != 0)
            {
                puts("Failed to set power\n");
                SharedSerial->SendCommand((char *)"O0000");
                return -1;
            }
            usleep(SLEEP);
            if (SharedSerial->SendCommand((char *)"O0000") != 0)
            {
                puts("Failed to set power\n");
                return -1;
            }
        }
        break;

     case SETMODE:

        sprintf(cmd, "U8\r");
        SharedSerial->Write(cmd, 3);

        SharedSerial->ReadByte(&retbyte);
        if ((char)retbyte != 'U')
        {
            //printf("\nReturned: .%c.\n",(char)retbyte);
            puts("Failed to read auto/man status from Wheelchair Interface Unit\n");
            return -1;
        }
        SharedSerial->ReadByte(&retbyte);
        SharedSerial->ReadByte(&retbyte2);
        SharedSerial->ReadByte(&retbyte);
        SharedSerial->ReadByte(&retbyte);
        SharedSerial->ReadByte(&retbyte);


        if (((char)retbyte2 == '0' && param) || ((char)retbyte2 != '0' && !param))
        {
            // we need to toggle the mode
            if (SharedSerial->SendCommand((char *)"O0020") != 0)
            {
                puts("Failed to set mode\n");
                SharedSerial->SendCommand((char *)"O0000");
                return -1;
            }
            usleep(LATCHDELAY);
            if (SharedSerial->SendCommand((char *)"O0000") != 0)
            {
                puts("Failed to set mode\n");
                return -1;
            }
        }
        break;

     case GEAR:
        printf("\n	+-->Processing Gear Request Param=:%d",param);
        if (param)
        {
            printf(" Going one Gear UP");
            // Gear up
            status |= 0x02;
            printf("\n	+-->This is the Gear UP Status Set Value =:%d",status);
            sprintf(cmd,"O00%02X", status);
            if(SharedSerial->SendCommand(cmd)!=0)

            {
                return -1;
            }
            usleep(SLEEP);
            usleep(SLEEP);
            usleep(SLEEP);
            status &= ~0x02;
            sprintf(cmd,"O00%02X", status);
            printf("\n	+-->This is the Gear UP Status Reset Value =:%d",status);
            if(SharedSerial->SendCommand(cmd)!=0)
            {
                //	PLAYER_ERROR("Failed to increment gear\n");
                return -1;
            }
        }
        else
        {
            // Gear down
            printf(" Going one Gear Down");
            status |= 0x01;
            printf("\n	+-->This is the Gear Down Status Set Value =:%d",status);
            sprintf(cmd,"O00%02X", status);
            if(SharedSerial->SendCommand(cmd)!=0)
            {
                return -1;
            }
            usleep(SLEEP);
            usleep(SLEEP);
            usleep(SLEEP);
            status &= ~0x01;
            printf("\n	+-->This is the Gear Dwon Status Rest Value =:%d",status);
            sprintf(cmd,"O00%02X", status);
            if(SharedSerial->SendCommand(cmd)!=0)
            {
                return -1;
            }
        }
        printf("\n	+-->This is the Final Status Value =:%d",status);
        break;

		case HORN:
        if (param) {
            // Turn horn on
            status |= 0x04;
            sprintf(cmd,"O00%02X", status);
            if(SharedSerial->SendCommand(cmd)!=0)
            {
                puts("Failed to enable horn\n");
                return -1;
            }
        } else {
            // Turn horn off
            status &= ~0x04;
            sprintf(cmd,"O00%02X", status);
            if(SharedSerial->SendCommand(cmd)!=0)
            {
                puts("Failed to enable horn\n");
                return -1;
            }
        }
        break;
		case GETMODE:

        sprintf(cmd, "U8\r");
        SharedSerial->Write(cmd, 3);

        SharedSerial->ReadByte(&retbyte);
        if ((char)retbyte != 'U')
        {
            puts("Failed to read auto/man status from Wheelchair Interface Unit\n");
            return -1;
        }
        SharedSerial->ReadByte(&retbyte);
        SharedSerial->ReadByte(&retbyte2);
        SharedSerial->ReadByte(&retbyte);
        SharedSerial->ReadByte(&retbyte);
        SharedSerial->ReadByte(&retbyte);


        if ((char)retbyte2 == '0')
            return MANUAL;
        else
            return AUTO;
        break;
		default:
        puts("Unknown command\n");
        return -1;
    }
    unlock;
    return 0;
}


SDL_Surface *screen;

int main()
{
    //pthread_mutex_init(&mutex,pthread_mutexattr_default);
    SharedSerial = new SerialCom((char *)SHRD_DEFAULT_PORT,SHRD_DEFAULT_RATE);
    SDL_Event event;
    int x,y,running = 1;
    int dirFR = 0;
    int dirLR = 0;
    oldxspeed=0;
    oldyawspeed=0;
    px=py=pa=0;
    int lrOffset = 0;
    int frOffset = 0;
    unsigned int spriteColour = 0xFFFFFFFF;
    WheelEncoder * LeftEncoder;
    WheelEncoder * RightEncoder;
    LeftEncoder  = new WheelEncoder((char *)ENCL_DEFAULT_PORT, ENC_DEFAULT_RATE);
    RightEncoder = new WheelEncoder((char *)ENCR_DEFAULT_PORT, ENC_DEFAULT_RATE);

    if (SDL_Init(SDL_INIT_VIDEO) != 0)
    {
        printf("Unable to initialize SDL : %s\n",SDL_GetError());
        return 1;
    }

    atexit(SDL_Quit);

    screen = SDL_SetVideoMode(320, 240, 0, SDL_ANYFORMAT);
    if (screen == NULL) {
        printf("Unable to set video mode: %s\n", SDL_GetError());
        return 1;
    }
    first_ltics=LeftEncoder->GetTicks();
    first_rtics=RightEncoder->GetTicks();
    last_ltics=0;
    last_rtics=0;

    printf("\033[2J");
    printf("\033[;H");
    printf("Loading...");
    fflush(stdout);

    printf("\033[;H");
    printf("  Welcome to a simple demonstration client that showcases the functionality of\n  the CAS wheelchair\n\n");
    printf("  The following keys can be used  to interact with the wheelchair:\n\n");
    printf("    Arrow keys - used to engage the wheelchair's motors\n");
    printf("    h          - used to sound the wheelchair's horn\n");
    printf("    p          - used to turn the wheelchair on\n");
    printf("    o          - used to turn the wheelchair off\n");
    printf("    m          - used to toggle the mode between auto and manual\n");
    printf("    + / -      - used to increment / decrement the gear\n");
    printf("    i          - used to Manualy Enter Speed and angle\n");
    fflush(stdout);
    /* go into read-think-act loop */
    double speed,angle;
    Control(POWER,ON);
    Control(SETMODE,AUTO);
    //Control(SETMODE,MANUAL);
    usleep(1000000);
#ifdef log
    file=fopen("odomlog.txt","wt");
#endif
    while(running)
    {
        while(SDL_PollEvent(&event))
        {
            //What kind of event has occurred?
            switch(event.type)
            {
            case SDL_KEYDOWN:	//A key has been pressed
                switch (event.key.keysym.sym)
                {
                case SDLK_h:	Control(HORN,1); usleep(100000); Control(HORN,0);       break;
                case SDLK_MINUS:Control(GEAR,DOWN);	break;
                case SDLK_EQUALS:Control(GEAR,UP);	break;
                case SDLK_m:	printf("\n- The Control Mode was:%d",Control(GETMODE,0));
                    if (Control(GETMODE,0) == MANUAL)
                    {
                        printf("\n	---> Changing Control Mode to Auto");
                        Control(SETMODE,AUTO);
                        spriteColour = 0x000FF000;
                    }
                    else
                    {
                        printf("\n	---> Changing Control Mode to Manual");
                        Control(SETMODE,MANUAL);
                        spriteColour = 0x00000FF0;
                    }
                    printf("\n- The Control Mode is now:%d",Control(GETMODE,0));
                    break;
            case SDLK_p:	Control(POWER,1);	break;
            case SDLK_o:    Control(POWER,0);	break;
            case SDLK_q:    running = 0;		break;
            case SDLK_LEFT: dirLR = 850;		break;
            case SDLK_RIGHT:dirLR = 3200;		break;
            case SDLK_UP:   dirFR = 3200;		break;
            case SDLK_DOWN: dirFR = 850;		break;
            case SDLK_w:    frOffset += 1000;
                printf("FR Offest is %d\n", frOffset);
                fflush(stdout);
                break;
            case SDLK_s:  	frOffset -= 1000;
                printf("FR Offest is %d\n", frOffset);
                fflush(stdout);
                break;
            case SDLK_a:  	lrOffset += 1000;
                printf("LR Offest is %d\n", lrOffset);
                fflush(stdout);
            break;
            case SDLK_d:  	lrOffset -= 1000;
                printf("LR Offest is %d\n", lrOffset);
                fflush(stdout);
                break;
            case SDLK_i:	printf("\n Please enter the speed---->");
                scanf("%lf",&speed);
                printf("\n Please enter the angle---->");
                scanf("%lf",&angle);
                break;
            default    :    printf("\n Unknown Key Pressed !!!");
            }
            break;
	case SDL_KEYUP:		//A key has been released
            if (strcmp(SDL_GetKeyName(event.key.keysym.sym),"left") == 0)
            {
	  	dirLR = 0;
            }
            if (strcmp(SDL_GetKeyName(event.key.keysym.sym),"right") == 0)
            {
	  	dirLR = 0;
            }
            if (strcmp(SDL_GetKeyName(event.key.keysym.sym),"up") == 0)
            {
	  	dirFR = 0;
            }
            if (strcmp(SDL_GetKeyName(event.key.keysym.sym),"down") == 0)
            {
	  	dirFR = 0;
            }
            break;
	case SDL_QUIT:	unsigned int retbyte;
            SharedSerial->Write((char *)"O0000\r", 6);
            SharedSerial->ReadByte(&retbyte);
            SharedSerial->ReadByte(&retbyte);
            SharedSerial->Write((char *)"L0800\r", 6);
            SharedSerial->ReadByte(&retbyte);
            SharedSerial->ReadByte(&retbyte);
            SharedSerial->Write((char *)"L1800\r", 6);
            SharedSerial->ReadByte(&retbyte);
            SharedSerial->ReadByte(&retbyte);
            delete SharedSerial;
            SharedSerial=NULL;//The user has closed the SDL window
            running = 0;
            break;
	}
        }

	x=(LeftEncoder->GetTicks()-first_ltics)*-1;  //Left Motor goes counter clock wise
	y=(RightEncoder->GetTicks()-first_rtics);
	UpdateMotors((dirFR + frOffset), (double)(dirLR + lrOffset));    
        //a = GetReading('A');
	//b = GetReading('B');
	//fprintf(file,"%d ",m);	
	//printf("\n X Readings=%d  Y Readings=%d",b,a);
	//printf("\nM is %d",m);
	//UpdateMotors(m,0);
	//UpdateMotors(a,b);
	usleep(100000);
    	fflush(stdout);
    }

    UpdateMotors(0,0);

#ifdef log
    fclose(file);
#endif
    printf("\n");
    return(0);
}
    
