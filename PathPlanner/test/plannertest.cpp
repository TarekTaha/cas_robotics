/***************************************************************************
 *   Copyright (C) 2007 by Tarek Taha                                      *
 *   tataha@eng.uts.edu.au                                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include "pathplanner.h"
#include "robot.h"
#include "map.h"
#include <iostream>
#include <fstream>
#include <errno.h>

using namespace CasPlanner;

Pose start,end;
Point robotCenter;
double robotHeight,robotWidth,narrowestPassage,mapResolution;
double distanceToGoal,bridgeLength,bridgeResolution,regularGridResolution,regularGridConnectionRadius,obstaclePenalty,bridgeConnectionRadius;
char mapFileName[50];
bool negateColor;

void usage(const char* cmdName) 
{
  cerr <<
    "usage: " << cmdName << " --argument <type>\n"
    "  --h  or --help shows this usage help\n"
    "  --c  or --configFile <fileName> reads the configuration parameters from the file\n"    
    "  --m  or --map <fileName> Specifies the map filename to be used\n"
    "  --n  or --negateColor <bool> if false then white color is free, if true then white color is occupied\n"    
    "  --r  or --mapResolution <float> Specifies the map pixel resolution to be used\n"
    "  --nP or --narrowestPassage <float> Specifies the narrowest passage in the map\n"    
	"  --rC or --robotCenter <float float> Specifies the robot's Center of Rotation in respec to the center of Area\n"    
	"  --rH or --robotHeight <float> Specifies the robot's Height\n"
	"  --rW or --robotWidth <float> Specifies the robot's Width\n"	
	"  --dG or --distanceToGoal <float> Specifies the distance to the goal to end the search\n"	
	"  --bL or --bridgeLength <float> Specifies the length of the bridge test.\n"
	"  --bR or --bridgeResolution <float> Specifies the resolution of the bridge test sampling.\n"
	"  --bC or --bridgeConnectionRadius <float> Specifies the distance for connecting bridge samples.\n"	
	"  --gC or --regularGridConnectionRadius <float> Specifies the distance for connecting regular grid samples.\n"	
	"  --gR or --regularGridResolution <float> Specifies the resolution for regular grid sampling.\n"	
	"  --oP or --obstaclePenalty <float> Specifies the distance to obstacle to start penalizing.\n"	
    "  --s  or --start <float float float> Specifies the X Y and Phi start Pose\n"
    "  --e  or --end   <float float float> Specifies the X Y and Phi end   Pose\n"
    "Example:\n"
    "  " << cmdName << " --map casarea.png --r 0.05 --s 5.721 -4.324 180 --e -6.5 6.5 180 --n 0 --rC -0.3 0 --rW 0.5 --rH 0.9 --nP 1 \n"
    "\n";
  exit(-1);
}

bool isEmptyLine(const char* buf)
{
	for (const char* p = buf; *p != '\0'; p++) 
	{
    	if (!isspace(*p)) 
    		return false;
  	}
  	return true;
}

void readParamFromFile(const std::string& fileName)
{
	char buffer[512], param[512], value[512];
	int line = 0;
	std::ifstream inputf(fileName.c_str());
  	if (!inputf) 
  	{
    	fprintf(stderr, "ERROR: couldn't open config file '%s' for reading: %s\n",fileName.c_str(), strerror(errno));
    	exit(EXIT_FAILURE);
  	}
  	while (!inputf.eof()) 
  	{
    	inputf.getline(buffer, sizeof(buffer));
    	line++;
    	if (isEmptyLine(buffer) || buffer[0]=='#') 
    		continue;
    	sscanf(buffer, "%s", param);
    	if(!strcmp(param,"map"))
    	{
    		if(sscanf(buffer, "%s %s", param,mapFileName)!=2)
			{
      			fprintf(stderr, "ERROR: %s:%d: syntax error, expected '<param> <string>'\n",fileName.c_str(), line);
      			exit(EXIT_FAILURE);
    		}
    	}
    	else if(!strcmp(param,"negateColor"))
    	{
    		int x;
    		if(sscanf(buffer, "%s %d", param,&x)!=2)
			{
      			fprintf(stderr, "ERROR: %s:%d: syntax error, expected '<param> <bool>'\n",fileName.c_str(), line);
      			exit(EXIT_FAILURE);
    		}
    		negateColor = x;
    	}
    	if(!strcmp(param,"mapResolution"))
    	{
    		if(sscanf(buffer, "%s %lf", param,&mapResolution)!=2)
			{
      			fprintf(stderr, "ERROR: %s:%d: syntax error, expected '<param> <float>'\n",fileName.c_str(), line);
      			exit(EXIT_FAILURE);
    		}
    	}
    	else if(!strcmp(param,"narrowestPassage"))
    	{
    		if(sscanf(buffer, "%s %lf", param,&narrowestPassage)!=2)
			{
      			fprintf(stderr, "ERROR: %s:%d: syntax error, expected '<param> <float>'\n",fileName.c_str(), line);
      			exit(EXIT_FAILURE);
    		}
    	}    	
    	else if(!strcmp(param,"robotCenter"))
    	{
    		double x,y;
    		if(sscanf(buffer, "%s %lf %lf", param,&x,&y)!=3)
			{
      			fprintf(stderr, "ERROR: %s:%d: syntax error, expected '<param> <float> <float>'\n",fileName.c_str(), line);
      			exit(EXIT_FAILURE);
    		}   		
    		robotCenter.setX(x);
    		robotCenter.setY(y);
    	}
    	else if(!strcmp(param,"robotHeight"))
    	{
    		if(sscanf(buffer, "%s %lf", param,&robotHeight)!=2)
			{
      			fprintf(stderr, "ERROR: %s:%d: syntax error, expected '<param> <float>'\n",fileName.c_str(), line);
      			exit(EXIT_FAILURE);
    		}		
    	}
    	else if(!strcmp(param,"robotWidth"))
    	{
    		if(sscanf(buffer, "%s %lf", param,&robotWidth)!=2)
			{
      			fprintf(stderr, "ERROR: %s:%d: syntax error, expected '<param> <float>'\n",fileName.c_str(), line);
      			exit(EXIT_FAILURE);
    		}  		
    	}
    	else if(!strcmp(param,"distanceToGoal"))
    	{
    		if(sscanf(buffer, "%s %lf", param,&distanceToGoal)!=2)
			{
      			fprintf(stderr, "ERROR: %s:%d: syntax error, expected '<param> <float>'\n",fileName.c_str(), line);
      			exit(EXIT_FAILURE);
    		}    	
    	}
    	else if(!strcmp(param,"bridgeLength"))
    	{
    		if(sscanf(buffer, "%s %lf", param,&bridgeLength)!=2)
			{
      			fprintf(stderr, "ERROR: %s:%d: syntax error, expected '<param> <float>'\n",fileName.c_str(), line);
      			exit(EXIT_FAILURE);
    		}    	
    	}
    	else if(!strcmp(param,"bridgeResolution"))
    	{
    		if(sscanf(buffer, "%s %lf", param,&bridgeResolution)!=2)
			{
      			fprintf(stderr, "ERROR: %s:%d: syntax error, expected '<param> <float>'\n",fileName.c_str(), line);
      			exit(EXIT_FAILURE);
    		}    		
    	}
    	else if(!strcmp(param,"bridgeConnectionRadius"))
    	{
    		if(sscanf(buffer, "%s %lf", param,&bridgeConnectionRadius)!=2)
			{
      			fprintf(stderr, "ERROR: %s:%d: syntax error, expected '<param> <float>'\n",fileName.c_str(), line);
      			exit(EXIT_FAILURE);
    		}    		
    	}
    	else if(!strcmp(param,"regularGridConnectionRadius"))
    	{
    		if(sscanf(buffer, "%s %lf", param,&regularGridConnectionRadius)!=2)
			{
      			fprintf(stderr, "ERROR: %s:%d: syntax error, expected '<param> <float>'\n",fileName.c_str(), line);
      			exit(EXIT_FAILURE);
    		}    		
	   	}
    	else if(!strcmp(param,"regularGridResolution"))
    	{
    		if(sscanf(buffer, "%s %lf", param,&regularGridResolution)!=2)
			{
      			fprintf(stderr, "ERROR: %s:%d: syntax error, expected '<param> <float>'\n",fileName.c_str(), line);
      			exit(EXIT_FAILURE);
    		}    		
    	}
    	else if(!strcmp(param,"obstaclePenalty"))
    	{
    		if(sscanf(buffer, "%s %lf", param,&obstaclePenalty)!=2)
			{
      			fprintf(stderr, "ERROR: %s:%d: syntax error, expected '<param> <float>'\n",fileName.c_str(), line);
      			exit(EXIT_FAILURE);
    		}    		
    	}
    	else if(!strcmp(param,"start"))
    	{
    		double x,y,t;
    		if(sscanf(buffer, "%s %lf %lf %lf", param,&x,&y,&t)!=4)
			{
      			fprintf(stderr, "ERROR: %s:%d: syntax error, expected '<param> <float> <float> <float>'\n",fileName.c_str(), line);
      			exit(EXIT_FAILURE);
    		}    		
    		start.p.setX(x);
    		start.p.setY(y);
    		start.phi=DTOR(t);
    	}
    	else if(!strcmp(param,"end"))
    	{
    		double x,y,t;
    		if(sscanf(buffer, "%s %lf %lf %lf", param,&x,&y,&t)!=4)
			{
      			fprintf(stderr, "ERROR: %s:%d: syntax error, expected '<param> <float> <float> <float> <float>'\n",fileName.c_str(), line);
      			exit(EXIT_FAILURE);
    		}    		
    		end.p.setX(x);
    		end.p.setY(y);
    		end.phi=DTOR(t);
    	}  	    	    	    	    	    	    	    	    	    	    	    	    	
  	}
  	inputf.close();
}

int main( int argc, char ** argv ) 
{
	// Defaults
	std::string temp;
	readParamFromFile("src/planner.config");
	// Override default values with arguments
	for (int argi=1; argi < argc; argi++) 
	{
    	std::string args = argv[argi];
		if (args == "--h" || args == "--help") 
  		{
			usage(argv[0]);
  		}
  		else if (args == "--c" || args == "--configFile") 
  		{
  			temp = argv[++argi];
			if (argi == argc || !temp.find("--")) 
			{
  				fprintf(stderr, "ERROR: found --m option without argument (use -h for help)\n");
  				exit(-1);
			}
			readParamFromFile(argv[argi]);
  		} 
  		else if (args == "--m" || args == "--map") 
  		{
  			temp = argv[++argi];
			if (argi == argc || !temp.find("--")) 
			{
  				fprintf(stderr, "ERROR: found --m option without argument (use -h for help)\n");
  				exit(-1);
			}
			strcpy(mapFileName,argv[argi]);
  		}
  		else if (args == "--n" || args == "--negateColor") 
  		{
  			temp = argv[++argi];
			if (argi == argc || !temp.find("--")) 
			{
  				fprintf(stderr, "ERROR: found --negateColor option without argument (use -h for help)\n");
  				exit(-1);
			}
			negateColor = atoi(argv[argi]);
  		}  	  		
  		else if (args == "--r" || args == "--resolution") 
  		{
  			temp = argv[++argi];
			if (argi == argc || !temp.find("--")) 
			{
  				fprintf(stderr, "ERROR: found --resolution option without argument (use -h for help)\n");
  				exit(-1);
			}
			mapResolution = atof(argv[argi]);
  		}
  		else if (args == "--nP" || args == "--narrowestPassage") 
  		{
  			temp = argv[++argi];
			if (argi == argc || !temp.find("--")) 
			{
  				fprintf(stderr, "ERROR: found --resolution option without argument (use -h for help)\n");
  				exit(-1);
			}
			narrowestPassage = atof(argv[argi]);
  		}  		
  		else if (args == "--bL" || args == "--bridgeLength") 
  		{
  			temp = argv[++argi];
			if (argi == argc || !temp.find("--")) 
			{
  				fprintf(stderr, "ERROR: found --robotWidth option without argument (use -h for help)\n");
  				exit(-1);
			}
			bridgeLength = atof(argv[argi]);
  		}
  		else if (args == "--bR" || args == "--bridgeResolution") 
  		{
  			temp = argv[++argi];
			if (argi == argc || !temp.find("--")) 
			{
  				fprintf(stderr, "ERROR: found --robotWidth option without argument (use -h for help)\n");
  				exit(-1);
			}
			bridgeResolution = atof(argv[argi]);
  		}   	
  		else if (args == "--bC" || args == "--bridgeConnectionRadius") 
  		{
  			temp = argv[++argi];
			if (argi == argc || !temp.find("--")) 
			{
  				fprintf(stderr, "ERROR: found --robotWidth option without argument (use -h for help)\n");
  				exit(-1);
			}
			bridgeConnectionRadius = atof(argv[argi]);
  		}
  		else if (args == "--gR" || args == "--regularGridResolution") 
  		{
  			temp = argv[++argi];
			if (argi == argc || !temp.find("--")) 
			{
  				fprintf(stderr, "ERROR: found --robotWidth option without argument (use -h for help)\n");
  				exit(-1);
			}
			regularGridResolution = atof(argv[argi]);
  		}
  		else if (args == "--gC" || args == "--regularGridConnectionRadius") 
  		{
  			temp = argv[++argi];
			if (argi == argc || !temp.find("--")) 
			{
  				fprintf(stderr, "ERROR: found --robotWidth option without argument (use -h for help)\n");
  				exit(-1);
			}
			regularGridConnectionRadius = atof(argv[argi]);
  		}
  		else if (args == "--oP" || args == "--obstaclePenalty") 
  		{
  			temp = argv[++argi];
			if (argi == argc || !temp.find("--")) 
			{
  				fprintf(stderr, "ERROR: found --robotWidth option without argument (use -h for help)\n");
  				exit(-1);
			}
			obstaclePenalty = atof(argv[argi]);
  		}
  		else if (args == "--dG" || args == "--distanceToGoal") 
  		{
  			temp = argv[++argi];
			if (argi == argc || !temp.find("--")) 
			{
  				fprintf(stderr, "ERROR: found --robotWidth option without argument (use -h for help)\n");
  				exit(-1);
			}
			distanceToGoal = atof(argv[argi]);
  		}
  		else if (args == "--rH" || args == "--robotHeight") 
  		{
  			temp = argv[++argi];
			if (argi == argc || !temp.find("--")) 
			{
  				fprintf(stderr, "ERROR: found --robotHeight option without argument (use -h for help)\n");
  				exit(-1);
			}
			robotHeight = atof(argv[argi]);
  		}
  		else if (args == "--rW" || args == "--robotWidth") 
  		{
  			temp = argv[++argi];
			if (argi == argc || !temp.find("--")) 
			{
  				fprintf(stderr, "ERROR: found --robotWidth option without argument (use -h for help)\n");
  				exit(-1);
			}
			robotWidth = atof(argv[argi]);
  		}  		  		
  		else if (args == "--rC" || args == "--robotCenter") 
  		{
  			temp = argv[++argi];
			if (argi == argc || !temp.find("--")) 
			{
  				fprintf(stderr, "ERROR: found --robotCenter option without enough arguments (use --h for help)\n");
  				exit(-1);
			}
			robotCenter.setX(atof(argv[argi]));
  			temp = argv[++argi];
			if (argi == argc || !temp.find("--")) 
			{
  				fprintf(stderr, "ERROR: found --robotCenter option without enough arguments (use --h for help)\n");
  				exit(-1);
			}
			robotCenter.setY(atof(argv[argi]));
  		}
  		else if (args == "--s" || args == "--start") 
  		{
  			temp = argv[++argi];
			if (argi == argc || !temp.find("--")) 
			{
  				fprintf(stderr, "ERROR: found --start option without enough arguments (use --h for help)\n");
  				exit(-1);
			}
			start.p.setX(atof(argv[argi]));
  			temp = argv[++argi];
			if (argi == argc || !temp.find("--")) 
			{
  				fprintf(stderr, "ERROR: found --start option without enough arguments (use --h for help)\n");
  				exit(-1);
			}
			start.p.setY(atof(argv[argi]));
  			temp = argv[++argi];
			if (argi == argc || !temp.find("--")) 
			{
  				fprintf(stderr, "ERROR: found --start option without enough arguments (use --h for help)\n");
  				exit(-1);
			}			
			start.phi = DTOR(atof(argv[argi]));
  		}
  		else if (args == "--e" || args == "--end") 
  		{
  			temp = argv[++argi];
			if (argi == argc || !temp.find("--")) 
			{
  				fprintf(stderr, "ERROR: found --end option without enough arguments (use --h for help)\n");
  				exit(-1);
			}
			end.p.setX(atof(argv[argi]));
  			temp = argv[++argi];
			if (argi == argc || !temp.find("--")) 
			{
  				fprintf(stderr, "ERROR: found --end option without enough arguments (use --h for help)\n");
  				exit(-1);
			}
			end.p.setY(atof(argv[argi]));
  			temp = argv[++argi];
			if (argi == argc || !temp.find("--")) 
			{
  				fprintf(stderr, "ERROR: found --end option without enough arguments (use --h for help)\n");
  				exit(-1);
			}
			end.phi = DTOR(atof(argv[argi]));
  		}
	}
	
	Robot *robot= new Robot(string("Robot"),robotHeight,robotWidth,narrowestPassage,robotCenter);
	Map   *map  = new Map(mapFileName,mapResolution,negateColor);
	PathPlanner * pathPlanner = new PathPlanner(robot,map,distanceToGoal,bridgeLength,bridgeResolution,regularGridResolution,regularGridConnectionRadius,obstaclePenalty,bridgeConnectionRadius);;
    
    pathPlanner->enableBridgeTest(true);
    pathPlanner->enableRegGrid(true);
    pathPlanner->enableObstPen(true);
    pathPlanner->enableExpObst(true);
	/*
	 *  Search Space generation, Please note that you will need
	 * to generate only one search space for each map. After
	 * the Search Space generation you can search for as many paths
	 * as you want.
	 */
    pathPlanner->generateSpace();
    pathPlanner->printNodeList();
    
    pathPlanner->findPath(start,end,METRIC);
 	/* 
 	 * Draws the Search Space and the Path into the map image and saves
 	 * them into two different images ending with _searchSpace.png and 
 	 * _path.png
 	 */
    pathPlanner->drawSearchSpace();
    pathPlanner->drawPath();
 
    delete robot;
    delete map;
    delete pathPlanner;
	return 1;
}
