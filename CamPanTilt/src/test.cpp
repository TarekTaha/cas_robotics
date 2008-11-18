#include <unistd.h> 
#include <stdio.h>
#include <string.h>  
#include <stdlib.h> 
#include <time.h> 
#include <iostream>
#include "stdlib.h"
#include "SDL/SDL.h"
#include "PanTilt.h"

int main()
{
	SDL_Surface *screen;	
  	SDL_Event event;
  	bool running = true;
  	PanTilt panTiltCam1((char*)"/dev/video0");
  	PanTilt panTiltCam2((char*)"/dev/video1");
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
  	printf("\033[;H");  
  	fflush(stdout);

	while(running)
 	{
		while(true)
		{
			usleep(10000);
			SDL_PollEvent(&event);			
//    	while(SDL_PollEvent(&event))
//     	{
      		switch(event.type)
      		{
        		case SDL_KEYDOWN:
					switch (event.key.keysym.sym)
					{
						case SDLK_LEFT: 
							panTiltCam1.pan(-INCPANTILT*PANSTEP);
							panTiltCam2.pan(-INCPANTILT*PANSTEP);							
							break;
						case SDLK_RIGHT:
							panTiltCam1.pan(INCPANTILT*PANSTEP);
							panTiltCam2.pan(INCPANTILT*PANSTEP);							
							break;
						case SDLK_UP:   
							panTiltCam1.tilt(-INCPANTILT*TILTSTEP);
							panTiltCam2.tilt(-INCPANTILT*TILTSTEP);							
							break;
						case SDLK_DOWN: 
							panTiltCam1.tilt(INCPANTILT*TILTSTEP);
							panTiltCam2.tilt(INCPANTILT*TILTSTEP);							
							break;
						case SDLK_a: 
							panTiltCam2.pan(-INCPANTILT*PANSTEP);					
							break;
						case SDLK_d:
							panTiltCam2.pan(INCPANTILT*PANSTEP);					
							break;
						case SDLK_w:   
							panTiltCam2.tilt(-INCPANTILT*TILTSTEP);							
							break;
						case SDLK_s: 
							panTiltCam2.tilt(INCPANTILT*TILTSTEP);							
							break;							
						case SDLK_r: 
							panTiltCam1.reset(PANTILT);			
							break;
						case SDLK_q: 
			          		running = false;
							break;							

							//						default    :    
//							printf("\n Unknown Key Pressed !!!");
					}
						break;
					case SDL_KEYUP:
					  		if (strcmp(SDL_GetKeyName(event.key.keysym.sym),"left") == 0)    {}
					  		if (strcmp(SDL_GetKeyName(event.key.keysym.sym),"right") == 0)   {}
					  		if (strcmp(SDL_GetKeyName(event.key.keysym.sym),"up") == 0)      {}
					  		if (strcmp(SDL_GetKeyName(event.key.keysym.sym),"down") == 0)    {}
					  		break;
					case SDL_QUIT:
			          		running = false;
			          		break;
				}
    	}
  	}
	return(0);
}
    
