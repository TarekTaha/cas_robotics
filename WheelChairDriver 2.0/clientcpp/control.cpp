#include <unistd.h> 
#include <stdio.h>
#include <string.h>  
#include <stdlib.h> 
#include <time.h> 
#include <iostream>
#include <libplayerc++/playerc++.h>
#include "stdlib.h"
#include "SDL/SDL.h"
#include "common.h"
#include "wheelchairproxy.h" 
#define log
// #define localize
SDL_Surface *screen;
class WheelChairProxy;
int main()
{
  	using namespace PlayerCc;
  	SDL_Event event;
  	int running = 1,count=0;
  	double dirFR = 0,dirLR = 0,lrOffset = 0,frOffset = 0,wcPrevX, wcPrevY;
  	bool changed=0;
  	unsigned int spriteColour = 0xFFFFFFFF;
  	if (SDL_Init(SDL_INIT_VIDEO) != 0)
  	{
  		printf("Unable to initialize SDL: %s\n", SDL_GetError());
    	return 1;
  	}
  	//When this program exits, SDL_Quit must be called
  	atexit(SDL_Quit);
  	//Set the video mode to anything, just need a window
  	screen = SDL_SetVideoMode(320, 240, 0, SDL_ANYFORMAT);
  	if (screen == NULL)
   	{
    	printf("Unable to set video mode: %s\n", SDL_GetError());
    	return 1;
  	}  
  	printf("\033[2J");
  	printf("\033[;H");  fflush(stdout);
  	
  	PlayerClient    robot("192.168.0.101",6665);
	Position2dProxy pp(&robot,0);
	WheelChairProxy WCp(&robot,0);
  	printf("\n Turning ON WheelChair"); fflush(stdout);
  	WCp.SetPower(ON);
  	WCp.SetMode(AUTO);
  	printf("\n WheelChair is on"); fflush(stdout);
  	fflush(stdout);
  	usleep(2000000);  
  
 	printf("\033[;H");  
  	printf("  Welcome to a simple demonstration client that showcases the functionality of\n  the CAS wheelchair\n\n");
  	printf("  The following keys can be used  to interact with the wheelchair:\n\n");
  	printf("    Arrow keys - used to engage the wheelchair's motors\n");
  	printf("    h          - used to sound the wheelchair's horn\n");
  	printf("    p          - used to turn the wheelchair on\n");
  	printf("    o          - used to turn the wheelchair off\n");
  	printf("    m          - used to change mode to manual\n");
  	printf("    n          - used to change mode to Auto\n");  
 	printf("    + / -      - used to increment / decrement the gear\n");
  	printf("    r          - used to reset speed and turn angle\n");
  	fflush(stdout);

  	#ifdef log
  		FILE * file;
  		file=fopen("log.txt","wt");
  	#endif
	while(running)
 	{
    	robot.Read();
    	while(SDL_PollEvent(&event))
     	{
     		changed=0;
      		switch(event.type)
      		{
        		case SDL_KEYDOWN:	//A key has been pressed
					switch (event.key.keysym.sym)
					{
						case SDLK_h:	 
							WCp.SoundHorn(100);       
							break;
						case SDLK_MINUS: 
							WCp.DecrementGear(3);	
							break;
						case SDLK_EQUALS:
							WCp.IncrementGear(2);	
							break;
						case SDLK_m:	
							printf("\n	---> Changing Control Mode to Manual");
		  					WCp.SetMode(MANUAL);
		  					break;
						case SDLK_n:	
							printf("\n	---> Changing Control Mode to Auto");
		  					WCp.SetMode(AUTO);
		  					break;		
						case SDLK_p:	
							printf("\n	---> Turning Wheelchair ON");						
							WCp.SetPower(ON);	
							break;
						case SDLK_o:    
							printf("\n	---> Turning Wheelchair OFF");												
							WCp.SetPower(OFF);	
							break;
						case SDLK_q:    
							running = 0;		
							break;
						case SDLK_u:    
							//WCp.Print();		
							break;
						case SDLK_LEFT: 
							dirLR = 0.25;	
							changed=1;	
							break;
						case SDLK_RIGHT:
							dirLR = -0.25;	
							changed=1;	
							break;
						case SDLK_UP:   
							dirFR = 0.2;	
							changed=1;	
							break;
						case SDLK_DOWN: 
							dirFR = -0.2;	
							changed=1;	
							break;
						case SDLK_w:    
							frOffset += 0.01;		
							changed=1;	
		  					printf("FR Offest is %d\n", frOffset);
							fflush(stdout);
							break;
						case SDLK_s:  	
							frOffset -= 0.01;			
							changed=1;
							printf("FR Offest is %d\n", frOffset);
							fflush(stdout);
							break;
						case SDLK_a:  	
							lrOffset += 0.01;			
							changed=1;
							printf("LR Offest is %d\n", lrOffset);
							fflush(stdout);
							break;
						case SDLK_d:  	
							lrOffset -= 0.01;			
							changed=1;
							printf("LR Offest is %d\n", lrOffset);
							fflush(stdout); 
							break;
						case SDLK_r:  	
							dirFR = 0;
				  			dirLR = 0;
				  			lrOffset = 0;
				  			frOffset = 0; 
							changed=1;  
							WCp.SetMode(MANUAL);
							break; 
						default    :    
							printf("\n Unknown Key Pressed !!!");
						}
						break;
					case SDL_KEYUP:		//A key has been released
					  		if (strcmp(SDL_GetKeyName(event.key.keysym.sym),"left") == 0)    { dirLR = 0;changed=1;}
					  		if (strcmp(SDL_GetKeyName(event.key.keysym.sym),"right") == 0)   { dirLR = 0;changed=1;}
					  		if (strcmp(SDL_GetKeyName(event.key.keysym.sym),"up") == 0)      { dirFR = 0;changed=1;}
					  		if (strcmp(SDL_GetKeyName(event.key.keysym.sym),"down") == 0)    { dirFR = 0;changed=1;}
					  		break;
					case SDL_QUIT:		//The user has closed the SDL window
			          		running = 0;
			          		break;
				}	
    	} 
		fflush(stdout);   
	    // WCp.Print();
	    // pp.Print();
	    if (changed==1)
	    	pp.SetSpeed((dirFR + frOffset),(dirLR + lrOffset)); 
		//printf("\n X reading=%lf Y readings=%lf",WCp.JoyX(),WCp.JoyY());
		if(count==50)
	    {
	    	//localp.Print();
	        count=0;
	    }
	    count++;
  	}
	#ifdef log
		fclose(file);
	#endif
	printf("\n");
	return(0);
}
    
