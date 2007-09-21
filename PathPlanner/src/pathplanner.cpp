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
/*!
 * @author Tarek Taha - Centre of Autonomous Systems - University of Technology Sydney
 */
#include "pathplanner.h"
namespace CasPlanner
{
//! Constructor for the PathPlanner Class
/*! This is a constructor of the PathPlanner Class:
 * @param rob a Pointer to a #CasPlanner::Robot Class.
 * @param map_in a Pointer to a #CasPlanner::Map Class.
 * @param dG a double variable representing an acceptable proximity to the game for ending the Search.
 * @param bridge_len a double variable representing the length of the Bridge for the Bridge Test.
 * @param bridge_r a double variable representing the sampling resolution of the bridge test.
 * @param reg_g is a double variable representing the regular grid resolution.
 * @param reg_conn_rad is a double variable representing the distance used to connect neigbouring nodes.
 * @param obst_pen is a double variable representing the distance from the obstacles to start penalizing.
 * @param bridge_conn_rad_in is a double variable representing the distance used to connect bridge nodes with 
 * adjacent nodes.
 * 
 */
PathPlanner::PathPlanner(Robot *rob,Map * map_in,double dG,double bridge_len,
			double bridge_r,double reg_g,double reg_conn_rad,double obst_pen,double bridge_conn_rad_in):
			Astar(rob,dG),
			obstacle_radius(rob->getExpansionRadius()),
			bridge_length(bridge_len),
			bridge_res(bridge_r),
			regGridDist(reg_g),
			reg_grid_conn_rad(reg_conn_rad),
			obst_dist(obst_pen),
			bridge_conn_rad(bridge_conn_rad_in)
{
	printf("\n-> Starting Planner."); fflush(stdout);
	this->setMap(map_in);	
	printf("\n\tPlanning Parameters:"); fflush(stdout);
	printf("\n\t\t\t Pixel Resolution = %f",map->getMapRes());fflush(stdout); 
	printf("\n\t\t\t Distance to Goal = %f",dG); fflush(stdout);
	printf("\n\t\t\t Bridge Test Lenght = %f",bridge_len); fflush(stdout);
	printf("\n\t\t\t Bridge Test Res = %f",bridge_res); fflush(stdout);
	printf("\n\t\t\t Reg Grid Res  = %f",reg_g); fflush(stdout);
	printf("\n\t\t\t Obstacle Expansion Radius = %f",obstacle_radius);fflush(stdout);         
	printf("\n\t\t\t Reg Connection Radius = %f",reg_grid_conn_rad);fflush(stdout);
	printf("\n\t\t\t Bridge Connection Radius = %f",bridge_conn_rad);fflush(stdout);
	printf("\n\t\t\t Obstacle Penalty = %f",obst_pen);	fflush(stdout);
};
//! Destructor of the PathPlanner Class
PathPlanner :: ~PathPlanner()
{
	freeResources();
	cout<<"\n	--->>> Allocated Memory FREED <<<---\n";fflush(stdout);
};
//! Frees resources
/*! Frees the allocated Memory resources so that the program
 * doesn't end up with unfreed memory
 */
void PathPlanner::freeResources()
{
	freeSearchSpace();
	freePath();
	root=NULL;
}
//! Frees a previously allocated path.
void PathPlanner::freePath()
{
	Node *p;
	while(path != NULL) 
	{
		p = path->next;
		delete path;
		path = p;
	}
};
//! Sets the status of bridge Test.
void PathPlanner:: enableBridgeTest(bool bt)
{
	bridgeTestEnabled = false;
}
//! Sets the status of Regular Grid Generation
void PathPlanner:: enableRegGrid(bool bt)
{
	regGridEnabled = bt;
}
//! Sets the status of adding an Obstacle Penality
void PathPlanner:: enableObstPen(bool bt)
{
	obstPenEnabled = bt;
}
//! Sets the status of Expanding Obstacles
void PathPlanner:: enableExpObst(bool bt)
{
	expObstEnabled = bt;
}
//! Sets the status for constructing a debugging tree
void PathPlanner:: enableShowTree(bool bt)
{
	showTreeEnabled = bt;
}
//! Sets the Obstacle expansion Radius
void   PathPlanner ::setExpRad(double a)
{
	obstacle_radius = a;
}
//! Sets the Bridge Test Length
void   PathPlanner ::setBridgeLen(double a)
{
	bridge_length = a;
}
//! Sets the Bridge Test sampling resolution
void   PathPlanner ::setBridgeRes(double a)
{
	bridge_res = a;
}
//! Sets the Regular Grid sampling resolution
void   PathPlanner ::setRegGrid(double a)
{
	regGridDist = a;
}
//! Sets the Regular Grid node connection radius
void   PathPlanner ::setRegGridConRad(double a)
{
	reg_grid_conn_rad = a;
}
//! Sets the Bridge Test node connection radius 
void   PathPlanner ::setBridgeConRad(double a)
{
	bridge_conn_rad = a;
}
//! Sets the distance for penalizing approching obstacles
void   PathPlanner ::setObstDist(double a)
{
	obst_dist = a;
}
//! Sets the start Pose for path Planning
void PathPlanner::setStart(Pose start)
{
	this->start = start;
}
//! Sets the end Pose for Path Planning
void PathPlanner::setEnd(Pose end)
{
	this->end = end;
}

//! Nifty way of doing an obstacle expansion, not finalized, ----->>>>> @ TO DO @ <<<<<------
void PathPlanner::expandObstacles()
{
	if(!mapInitialized)
	{
		printf("\n	--->>> You have to read the map before Expanding the Obstacles <<<---	");fflush(stdout);
		return;
	}
	int thickness;
	int m,n,x,y,radius;
	thickness = int(this->obstacle_radius/map->getMapRes());
	radius =2; // This covers vertical, horizontal and diagonal cells
	Map temp_map(map->getWidth(),map->getHeight(),map->getMapRes(),map->getNegate());
	for(int i = 0;i<map->getWidth();i++)
		for(int j = 0;j<map->getHeight();j++)
		{
			temp_map.grid[i][j] = map->grid[i][j];
		}
	for(int i=0;i<this->map->getWidth() - 1;i++)
		for(int j=0;j<this->map->getHeight() - 1 ;j++)
		{
			for(x = (i - radius) ; x <= (i + radius) ; x ++)
			{
				if (x < 0) x = 0;
				if (x >= this->map->getWidth())  break;
				y = (int)(- sqrt(radius*radius - (x - i)*(x - i)) + j);
				if (y < 0) y = 0;
				if (y >= this->map->getHeight()) y = this->map->getHeight() - 1;
				if (temp_map.grid[i][j] && !temp_map.grid[x][y])
				{
				for (int r = 1; r <(int)(thickness);r++)
					for( m = (int)(i - r); m <= (int)(i + r);m++)
					{
						if (m < 0) m = 0;
						if (m >= this->map->getWidth())  break;
						n = (int)(- sqrt(r*r - (m - i)*(m - i)) + j);
						if (n < 0) n = 0;
						if (n >= this->map->getHeight()) n = this->map->getHeight() - 1;
						this->map->grid[m][n] = true;
						n = (int)(+ sqrt(r*r - (m - i)*(m - i)) + j);
						if (n < 0) n = 0;
						if (n >= this->map->getHeight()) n = this->map->getHeight() - 1;
						this->map->grid[m][n] = true;
					}
				break;
				}
				y = (int)(+ sqrt(radius*radius - (x - i)*(x - i)) + j);
				if (y < 0) y = 0;
				if (y >= this->map->getHeight()) y = this->map->getHeight() - 1;
				if (temp_map.grid[i][j] && !temp_map.grid[x][y])
				{
				for (int r = 1; r <(int)(thickness);r++)
					for( m = (int)(i - r); m <= (int)(i + r);m++)
					{
						if (m < 0) m = 0;
						if (m >= this->map->getWidth())  break;
						n = (int)(- sqrt(r*r - (m - i)*(m - i)) + j);
						if (n < 0) n = 0;
						if (n >= this->map->getHeight()) n = this->map->getHeight() - 1;
						this->map->grid[m][n] = true;
						n = (int)(+ sqrt(r*r - (m - i)*(m - i)) + j);
						if (n < 0) n = 0;
						if (n >= this->map->getHeight()) n = this->map->getHeight() - 1;
						this->map->grid[m][n] = true;
					}
				break;
				}
			}
		}
	printf("\n	--->>> OBSTACLES EXPANDED SUCCESSFULLY <<<---	"); fflush(stdout);
	//ShowConnections();
};
bool PathPlanner::isEmptyLine(const char* buf)
{
	for (const char* p = buf; *p != '\0'; p++) 
	{
    	if (!isspace(*p)) 
    		return false;
  	}
  	return true;
}  	

bool PathPlanner::parametersChanged(const char *fileName)
{
  	assert(fileName != NULL);
	char buffer[1024];
	double bL,bR,gR,mR,oD;
	char mN[50];
	std::ifstream inputf(fileName);
  	if (!inputf)
  	{
    	fprintf(stderr, "ERROR: couldn't open config file '%s' for reading: %s\n",fileName, strerror(errno));
    	return true;
  	}
  	while (!inputf.eof()) 
  	{
    	inputf.getline(buffer, sizeof(buffer));
    	if (isEmptyLine(buffer) || buffer[0]=='#') 
    		continue;
   		sscanf(buffer,"%lf %lf %lf %lf %lf %s\n",&bL,&bR,&gR,&oD,&mR,mN);
   		break;
  	}
  	printf("\n bL:%f bR:%f gR:%f oD:%f mR:%f mN:%s",bL,bR,gR,oD,mR,mN);
  	if(!isEqual(bL,bridge_length) || !isEqual(bR,bridge_res) || !isEqual(gR,regGridDist)  || !isEqual(oD,obst_dist)  || !isEqual(mR,map->getMapRes()) )
  	{
  		printf("\n Something is not equal to something");
  		printf("\n%f %f",bL,bridge_length);
  		printf("\n%f %f",bR,bridge_res);
  		printf("\n%f %f",gR,regGridDist);
  		printf("\n%f %f",oD,obst_dist);
  		printf("\n%f %f",mR,map->getMapRes());
  		return true;
  	}
  	if(strcmp(map->mapFileName,mN))
  		return true;
  	return false;
}
//! Reads a previously saves Search Space
bool PathPlanner::readSpaceFromFile(const char *fileName)
{
  	double locationx,locationy,obstacle_cost;
  	int type;
  	SearchSpaceNode *temp;
  	assert(fileName != NULL);
	char buffer[1024];
	int line = 0;
	std::ifstream inputf(fileName);
  	if (!inputf)
  	{
    	fprintf(stderr, "ERROR: couldn't open config file '%s' for reading: %s\n",fileName, strerror(errno));
    	exit(EXIT_FAILURE);
  	}
  	while (!inputf.eof()) 
  	{
    	inputf.getline(buffer, sizeof(buffer));
    	if (isEmptyLine(buffer) || buffer[0]=='#') 
    		continue;
    	line++;
    	if(line ==1)
    		continue;    		
  		sscanf(buffer,"%lf %lf %lf %d\n",&locationx,&locationy,&obstacle_cost,&type);
		if (searchSpace == NULL ) // Constructing the ROOT NODE
		{
			temp = new SearchSpaceNode;
			temp->location.setX(locationx);
			temp->location.setY(locationy);
			temp->obstacle_cost = obstacle_cost;
			temp->parent   = NULL;
			temp->next     = NULL;
			temp->type     = type;
			searchSpace = temp;
		}
		else
		{
			temp = new SearchSpaceNode;
			temp->location.setX(locationx);
			temp->location.setY(locationy);
			temp->obstacle_cost = obstacle_cost;
			temp->parent = NULL; 
			temp->next   = searchSpace;
			temp->type     = type;
			searchSpace = temp;
		}
  	}
  	return true;
}
//! Saves a generates Search Space into a File
bool PathPlanner::saveSpace2File(const char *filename)
{
  	assert(filename != NULL);
  	filename = strdup(filename);
  	FILE *file = fopen(filename, "wb");
  	if (!file)
  	{
  		printf("Error Opening File");fflush(stdout);
    	fclose(file);
    	return false;
  	}
  	SearchSpaceNode *temp=searchSpace;
	fprintf(file,"# Search Space File, This file is automatically generated\n");
	fprintf(file,"# and it contains a set of sample nodes that will be used\n");
	fprintf(file,"# to Generate the search space\n");
	fprintf(file,"# The search Space was generated with the Following Parameteres:\n");
	fprintf(file,"%f %f %f %f %f %s\n",bridge_length,bridge_res,regGridDist,obst_dist,map->getMapRes(),map->mapFileName);			  	
  	while (temp)
  	{
  		fprintf(file,"%f %f %f %d\n",temp->location.x(),temp->location.y(),temp->obstacle_cost,temp->type);
		temp = temp->next;
  	}
  	fclose(file);
  	return true;
}
//! Generates a regular grid with a certain resolution
void PathPlanner::generateRegularGrid()
{
	SearchSpaceNode *temp;
	if(!mapInitialized)
	{
		printf("\n	--->>> You have to read the map First <<<---	");fflush(stdout);
		return;
	}
	Point p;
	for(int i=0; i<this->map->getWidth(); i++)
	{
		for(int j=0;j<this->map->getHeight();j++)
		{
			if (!this->map->grid[i][j]) //Free Pixel
			{
				if (searchSpace == NULL ) // Constructing the ROOT NODE
				{
					temp = new SearchSpaceNode;
					temp->location.setX(i);
					temp->location.setY(j);
					map->convertPix(&temp->location);
					temp->parent   = NULL;
					temp->next     = NULL;
					temp->type     = RegGridNode;
					searchSpace = temp;
				}
				else
				{
					p.setX(i);
					p.setY(j);
					map->convertPix(&p);
					if (checkShortestDistance(p.x(),p.y(),regGridDist))
					{
						temp = new SearchSpaceNode;
						temp->location.setX(p.x());
						temp->location.setY(p.y());
						temp->parent = NULL; 
						temp->next   = searchSpace;
						temp->type     = RegGridNode;						
						searchSpace = temp;
					}
				}
			}
		}
	}
	printf("\n	--->>> REGULAR GRID GENERATED SUCCESSFULLY <<<---	");fflush(stdout);
};
//! Generates node samples using Bridge Test
void PathPlanner::bridgeTest()
{
	int radius,pixels_per_bridge;
	SearchSpaceNode *temp;
	Point p;
	double x,y,x2,y2;
	
//	pixels_per_bridge = (int) (robot->robotLength/map->getMapRes());
//	radius = (int) (robot->robotLength/(map->getMapRes()*2.0));

	pixels_per_bridge = (int) (bridge_length/map->getMapRes());
	radius = (int) (bridge_length/(map->getMapRes()*2.0));

	for(int i=0; i < this->map->getWidth() - pixels_per_bridge; i++)
		{
		for(int j=0;j<this->map->getHeight() - pixels_per_bridge ;j++)
			{
			int ab=0;
				for( x = (i - radius) ; x < (i + radius) ; x+=(radius/4.0))//x+=(radius))
				{
					// The circle is of Center (i,j) and radius R=radius
					ab++;
					if (x < 0) x = 0;
					if (x > this->map->getWidth()) break;						
					y = (int)(+ sqrt(radius*radius -  ( x - i)*( x - i)) + j);
					if (y < 0) y = 0;
					if (y >= this->map->getHeight()) y = this->map->getHeight() - 1;

					x2 = i + (i - x);
					if (x2 < 0) x2 = 0;
					if (x2 > this->map->getWidth()) break;	
					y2 = (int)(- sqrt(radius*radius - (x2 - i)*(x2 - i)) + j);
					if (y2 < 0) y2 = 0;
					if (y2 >= this->map->getHeight()) y2 = this->map->getHeight() - 1;
					//cout<<"\n x="<<x<<" y="<<y<<" x2="<<x2<<" y2="<<y2<<" i="<<i<<" j="<<j<<" R="<<radius;
					//fflush(stdout);
					if (!this->map->grid[i][j]&&this->map->grid[(int)x][(int)y]&&this->map->grid[(int)x2][(int)y2])
					{
						p.setX(i);
						p.setY(j);
						map->convertPix(&p);
						if (searchSpace == NULL ) // Constructing the ROOT NODE
						{
							temp = new SearchSpaceNode;
							temp->location.setX(p.x());
							temp->location.setY(p.y());
							temp->parent   = NULL;
							temp->next     = NULL;
							temp->type     = BridgeNode;							
							searchSpace = temp;
						}
						else
						{
							if (checkShortestDistance(p.x(),p.y(),bridge_res))
							{
								temp = new SearchSpaceNode;
								temp->location.setX(p.x());
								temp->location.setY(p.y());
								temp->parent = NULL;
								temp->next   = searchSpace;
								temp->type     = BridgeNode;									
								searchSpace = temp;
							}
						}
					}
				}
			//cout<<"\n I have iterated = "<<ab;
			//fflush(stdout);
			}
		}
	printf("\n	--->>> BRIDGE TEST FINISHED SUCCESSFULLY <<<---");fflush(stdout);
};
//! Print the list of Sampling Nodes in the path found
void PathPlanner :: printNodeList()
{
	int step=1;
	Point  pixel;
	Node *p;
	if(!(p = this->path))
		return ;
	printf("\n  --------------------   START OF LIST ----------------------");fflush(stdout);
	while(p !=NULL)
	{
		pixel =  p->pose.p;
		cout <<"\nStep [" << step++ <<"] x["<< pixel.x()<<"]y["<<pixel.y()<<"]"<<" Direction="<<p->direction; fflush(stdout);
		cout <<"\tG cost="<<p->g_value<<"\tH cost="<<p->h_value<<"\tFcost="<<p->f_value;fflush(stdout);
		//cout<<"\tStored Angle = "<< setiosflags(ios::fixed) << setprecision(2)<<RTOD(p->angle);
		if (p->next !=NULL)
		{
			//cout<<"\tNext Angle = "<< setiosflags(ios::fixed) << setprecision(2)<<RTOD(atan2(p->next->location.y - p->location.y, p->next->location.x - p->location.x));
			//cout<<"\tAngle Diff ="<< setiosflags(ios::fixed) << setprecision(2)<<RTOD(anglediff(p->angle,atan2(p->next->location.y - p->location.y, p->next->location.x - p->location.x)));
		}
		p = p->next;
	}
	printf("\n --------------------   END OF LIST ---------------------- ");fflush(stdout);
}
//! adds distance penalties to nodes.
void PathPlanner::addCostToNodes()
{
	SearchSpaceNode * S;
	Point  point;
	double number_of_pixels,radius,nearest_obstacle;
	int i,j;
	//int n=0;
	if (!searchSpace) // Do nothing if Search Space is not Yet Created
		return;
	S = searchSpace;
	number_of_pixels = obst_dist / map->getMapRes();
	while (S!=NULL)
	{
		//cout<<"\n Node= "<<++n;
		//fflush(stdout);
		point.setX(S->location.x());
		point.setY(S->location.y());
		map->convert2Pix(&point);
		nearest_obstacle = 0;
		for(radius = (int)number_of_pixels ; radius >0 ; radius --)
			{
				for( i= (int)(point.x() - radius) ; i < (int)(point.x() + radius); i+=1)
					{
						if (i < 0) i = 0;
						if (i >= this->map->getWidth())  break;
						j = (int)(- sqrt(radius*radius - (i - point.x())*(i - point.x())) + point.y());
						if (j < 0) j = 0;
						if (j >= this->map->getHeight()) j = this->map->getHeight() - 1;
						if(this->map->grid[i][j])
								 nearest_obstacle = radius;
						j = (int)(+ sqrt(radius*radius - (i - point.x())*(i - point.x())) + point.y());
						if (j < 0) j = 0;
						if (j >= this->map->getHeight()) j = this->map->getHeight() - 1;
	
						if(this->map->grid[i][j])
								 nearest_obstacle = radius;
					}
			}
		// this is a normalized cost, it will make more sense later on 
		S->obstacle_cost =  (obst_dist - nearest_obstacle*map->getMapRes())/obst_dist; 
		S = S->next;
	}
	printf("\n	--->>> Penalty Added <<<---	");fflush(stdout);
};
//! connects node according to the distance and type
void PathPlanner::connectNodes()
{
	SearchSpaceNode * S;
	SearchSpaceNode *temp;
	double distance,angle;
	// Do nothing if Search Space is not Yet Created
	if (!searchSpace) 
		return;
	temp = searchSpace;
	//	cout<<"\n RegGrid Conn:"<<reg_grid_conn_rad<<" Bridge Conn:"<<bridge_conn_rad;
	while (temp!=NULL)
	{
		S = searchSpace;
		while (S!=NULL)
		{
			distance = Dist(S->location,temp->location);
			double testDist;
			if(!(S->type==RegGridNode && temp->type==RegGridNode))
			{
				testDist = bridge_conn_rad;
			}
			else
			{
				testDist = reg_grid_conn_rad;
			}
			
			if (distance <= testDist && distance !=0)
			{
				angle = atan2(S->location.y() - temp->location.y() ,S->location.x() - temp->location.x());
				if(!inObstacle(temp->location,angle) && !inObstacle(S->location,angle))
				{
					temp->children.push_back(S);
				}
			}
			S = S->next;
		}
		temp = temp->next;
	}
	printf("\n	--->>> NODES CONNECTED <<<---	");fflush(stdout);
};
//! Checks if a give distance is the shortest between 2 nodes
bool PathPlanner::checkShortestDistance(double x,double y,double neigbhour_distance)
{
	SearchSpaceNode * S;
	double distance,shortest_distance = 10000000;

	S = searchSpace;
	while (S!=NULL)
	{
		distance = sqrt(pow(S->location.x() - x,2)+pow(S->location.y() - y,2));
		if (distance <= shortest_distance)
			shortest_distance = distance;
		S = S->next;
	}
	if( shortest_distance > neigbhour_distance )
		return 1;
	else
		return 0;
};
//! Shows the number of nodes and connections
void PathPlanner::showConnections()
{
	Point loc1,loc2;
	SearchSpaceNode *temp = searchSpace;
	int m=0,n=0;
	while (temp != NULL)
	{
		for(unsigned int i=0; i < temp->children.size();i++)
		{
			loc1 = temp->location;
			map->convert2Pix(&loc1);
			map->drawPixel(1,0,0,(int)loc1.x(),(int)loc1.y());
			loc2 = temp->children[i]->location;
			map->convert2Pix(&loc2);
			m++;
		}
		temp = temp->next;
		n++;
	}
	printf("\n---->>> TOTAL NUMBER OF CONNECTIONS =%d\n---->>> Total Nodes in search Space =%d",m,n);fflush(stdout);
}
//! Saves the search Space into a file
void PathPlanner::saveSearchSpace()
{
	Point p;
	SearchSpaceNode *temp = searchSpace;
	while (temp != NULL)
	{
		p = temp->location;
		map->convert2Pix(&p);
		this->map->grid[int(p.x())][(int)p.y()]= true ;
		temp =temp->next;
	}
}
//! checks if the search space file already exists
bool PathPlanner::fileExist(const char * fname)
{
	struct stat stat_buf;
 	if (stat(fname,&stat_buf) != 0)
 		return false;
  	return (stat_buf.st_mode & S_IFMT) == S_IFREG;
}
//!Generates the search Space
/*! This method generates the search Space. It checks first for a previously generates search Space file
 * and if found then use it instead of generating the search space from scratch (upto 10 times faster). If the file 
 * is not found then it generates the search space using the enabled sampling methods (BridgeTest/Regular Grid). The generated
 * Space will then be saved for later use.
 */
void PathPlanner::generateSpace()
{
	Timer timer;
	const char * filename = "SearchSpace.txt";
	if(this->searchSpace)
	{
		//pathPlanner->FreeSearchSpace();		
		return;
	}		
	timer.restart();
	if(fileExist(filename) && !parametersChanged(filename))
	{
		printf("\nLoading Space From file ..."); fflush(stdout);
		this->readSpaceFromFile(filename);
		if(expObstEnabled)
			this->expandObstacles();
		this->connectNodes();	
		printf("\nFile loading took:%f sec",timer.secElapsed());fflush(stdout);
	}
	else
	{
		printf("\nGenerating Space ...");		fflush(stdout);
		if(expObstEnabled)
			this->expandObstacles();
		if(regGridEnabled)
			this->generateRegularGrid();
		if(bridgeTestEnabled)
			this->bridgeTest();
		if(obstPenEnabled)
			this->addCostToNodes();
		this->connectNodes();
		this->saveSpace2File(filename);
		printf("\nSpace Generation took:%f sec",timer.secElapsed());	fflush(stdout);	
	}
	this->renderTree = true;
	this->showConnections();
}
//! Finds a path from start Pose to the end Pose using the prefered coordinate system
Node* PathPlanner::findPath(Pose start, Pose end,int coord)
{
	this->start = start;
	this->end = end;
	return findPath(coord);
}
//! Finds a path from start to end (previously assigned)
Node * PathPlanner::findPath(int coord)
{
	Node * retval;
	if(!this->searchSpace)
	{
		generateSpace();
	}

	retval = this->startSearch(start,end,coord);		

	if(retval)
	{
		this->printNodeList();
	}
	else
	{
		printf("\nNo Path Found");fflush(stdout);
	}
	return retval;
}
/*! Sets the converted QImage or the Laser Scan Map to 
 * the Current Planner
 */ 
void PathPlanner :: setMap(Map * map_in)
{
//	printf("SETTING MAP"); fflush(stdout);
	if(!map_in)
	{
		printf("\nWhy the hell ur giving me an empty map ???");fflush(stdout);
		fflush(stdout);
		return;
	}
	if(!map_in->grid)
	{
		printf("\nWhy the hell ur giving me an empty map data ???");fflush(stdout);
		fflush(stdout);
		return;		
	}
	this->map = map_in;
	MAXNODES = MaxLong;
//	printf("W_in:%d H_in:%d",map->getWidth(),map->getHeight());
	mapInitialized = true;	
//	printf("MAP SET"); fflush(stdout);	
};
//! draws the generated path into an image file (useful for debugging).
void PathPlanner::drawPath()
{
	if (!path)
	{
		printf("\n->NO Path Generated Yet, plan a Path First");
		return;
	}

	Point l_start,l_end;
  	Node *p;
  	p = path;
	while(p != NULL)
	{
		l_start =  p->pose.p;
		map->convert2Pix(&l_start);
		map->drawPixel(0,0,1,(int)l_start.x(),(int)l_start.y());
		p = p->next;
	}
	map->savePixelBuffer("_path.png");
}
//! Draws the Search space into an image file (useful for debugging).
void PathPlanner::drawSearchSpace()
{
	Point loc;
	SearchSpaceNode *temp = searchSpace;
	while (temp != NULL)
	{
		for(unsigned int i=0; i < temp->children.size();i++)
		{
			loc = temp->location;
			map->convert2Pix(&loc);
			map->drawPixel(1,0,0,(int)loc.x(),(int)loc.y());
		}
		temp = temp->next;
	}
	map->savePixelBuffer("_searchSpace.png");	
}

}
