#include <unistd.h> 
#include <stdio.h>
#include <string.h>  
#include <stdlib.h>  
#include <playerclient.h>   
#include <time.h> 
#include "SDL/SDL.h"
 #define USAGE \
 "USAGE: ./simplenav [-g <goal>] [-l <pose>]]\n"\
 "-g <goal>:what is the goal location for navigator:<x y psi>(ex.4 7 90)\n"\
 
 SDL_Surface *screen;
 double goto_x,goto_y,goto_psi,dirLR,dirFR;
 double pose_x,pose_y,pose_a;
 int cmd_line_gotoxy = 0,cmd_line_pose=0;
int main (int argc, char **argv)
{
SDL_Event event;
  
if (SDL_Init(SDL_INIT_VIDEO) != 0) 
	{
  	printf("Unable to initialize SDL: %s\n", SDL_GetError());
    	return 1;
  	}
atexit(SDL_Quit);
screen = SDL_SetVideoMode(320, 240, 0, SDL_ANYFORMAT);
if (screen == NULL) 
{
    printf("Unable to set video mode: %s\n", SDL_GetError());
    return 1;
}

PlayerClient robot("localhost", 6665);
PositionProxy pp(&robot,1,'a');
printf("%s\n",robot.conn.banner);
if(pp.GetAccess() == 'e') 
  	{
    		puts("\nError getting position device access!");
    		exit(1);
  	}
int count=0; 
int changed,running=1;
for(;;)
{

  while(running)
  {
    
    if(robot.Read()) exit(1);
    while(SDL_PollEvent(&event))
     {
     changed=0;
     //What kind of event has occurred?
      switch(event.type)
      {
        case SDL_KEYDOWN:	//A key has been pressed
	switch (event.key.keysym.sym)
	{
	case SDLK_q:    running = 0;	exit(1);	break;
	case SDLK_c:	pp.ResetOdometry();		break;
	case SDLK_LEFT: dirLR = 20;	changed=1;	break;
	case SDLK_RIGHT:dirLR = -20;	changed=1;	break;
	case SDLK_UP:   dirFR = 0.5;	changed=1;	break;
	case SDLK_DOWN: dirFR = -0.5;	changed=1;	break;
	case SDLK_r:  	
			dirFR = 0;
  			dirLR = 0;
			changed=1;  
			break; 
	default    :    printf("\n Unknown Key Pressed !!!");
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
			exit(1);
          		break;
	}
    } 
    fflush(stdout);  
    if (changed==1) 
	pp.SetSpeed((dirFR), 0,(double)(DTOR(dirLR)));    
if(count==50)
      {
       //	lp.Print();
         	count=0;
      }
      count++;
}
}
return 0;
}


