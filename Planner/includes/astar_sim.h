/***************************************************************************
 *   Copyright (C) 2005 by Tarek Taha                                      *
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
#define OPEN 1
#define NEW 0
#define CLOSED 2
#include <unistd.h> 
#include <time.h> 
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <string.h>
#include <error.h>
#include "SDL/SDL.h"
#include <gdk-pixbuf/gdk-pixbuf.h>
#include <gtk/gtkmain.h>
#include<gtk/gtk.h>
#include<glib/gprintf.h>
#include <assert.h>
#include <playerclient.h>
#include "common.h"
#include "wheelchairproxy.h"
#define max_speed 0.1
#define max_turn_rate 0.2 // wheelchair negative rotations
#define ClockWise -1
#define AntiClockWise 1
#include "map.h"
#include "callbacks.h"
#include "interface.h"
#include "support.h"
SDL_Surface *screen;
FILE * file;
GTimer *timer2,*delta_timer;
double last_time;
double estimate_x,estimate_y,estimate_theta,velocity,delta_t;
class Point
	{
	public : 
		double x,y;
		Point(double x, double y);
		Point();
	};
Point::Point()
	{
	this->x = 0;
	this->y = 0;
	};
Point::Point(double x,double y)
	{
	this->x = x;
	this->y = y;
	};
class Robot
	{
	public :
		//Point * check_points;
		vector<Point> check_points;
		void SetCheckPoints(int,Point *);
		Robot();
		~Robot();	
	};
void Robot::SetCheckPoints(int number_of_points,Point * a) 
	{
	//this->check_points = new Point[number_of_points];
	for(int i = 0; i < number_of_points; i++)
		{
		check_points.push_back(a[i]);
		//cout << "\n New Robot i="<<i<<" X=" <<this->check_points[i].x<<" Y="<<this->check_points[i].y;
		}
	};
Robot::Robot() 
	{
	};
Robot::~Robot() 
	{
	//delete [] this->check_points;
	//cout << "\n Robot Freed ";
	};
class Node
	{
	public :
		int id, depth,state;
		double nearest_obstacle,g_value,h_value,f_value,angle;
		Node  * parent, * next, * prev;
		Point   NodeInfo;
		Robot  wheelchair; // saving the location of the Robot edges on the planned path
		Node ();
		~Node();	
	};
Node :: Node ()
	{
	parent = next = prev = NULL;
	};
Node :: ~Node ()
	{
	//cout << "\n Node Freed ";
	parent = next = prev = NULL;
	};
class SearchSpaceNode
	{
	public :
		Point location;
		SearchSpaceNode * parent, * next;
		double obstacle_cost;
		vector <SearchSpaceNode *>  children;
		 SearchSpaceNode ();
		~SearchSpaceNode ();
	};
SearchSpaceNode::SearchSpaceNode()
	{
	parent = next = NULL;
	};
SearchSpaceNode::~SearchSpaceNode()
	{
	parent = next = NULL;
	};
class AstarPlanner 
	{
	public :
		Robot * wheelchair; // Location of the wheelchair's points to check in the wheelchair coordinate system
		int number_of_point_to_check,map_height,map_width,MAXNODES;
		double pixel_size,initial_angle,obstacle_radius,node_gap,bridge_length,final_angle;
		bool simulate;
		Point start, end, * points_to_check,local_edge_points[4],translated_edge_points[4];
		const char * MapFilename;
		GtkWidget * widget;
		GdkPixbuf * pixbuf;
		MapInfo mapinfo;	
		Node * test,*root,*path, *p;
		SearchSpaceNode * search_space,* temp;
		Node *openList, *closedList, *current, *childList, *curChild, *q;
		MapFile * Map;
	public :
		double Calculate_g   (Node *); // Distance from Start
		double Calculate_h   (Node *); // Herustic Distance to Goal
		bool   GoalReached   (Node *); 
		Node  *MakeChildrenNodes(Node *parent) ;
		void   FreeNode      (Node *);
		int    NodeEquality  (Node *, Node *);
		void   PrintNodeList ();
		int    StartSearch   (Point start, Point end,double initial_angle,double final_angle);
		void   Translate(Point  , double);
		void   Translate_edges(Point  , double);
		int    check_line (Point start,Point end);
		double DistToLineSegment(Point LineStart, Point LineEnd, Point p);
		int    Obstacle   (double x, double y, double angle);
		void   ConvertPixel   (Point *p);
		void   ConvertToPixel (Point *p);
		double Absolute(double );
		double DistanceToLine(Point p1,Point p2, Point p3);
		void   ReadMap(); // Reads the Map file and sets the attributes
		void   ExpandObstacles();
		void   AddCostToNodes(double distance);
		void   BridgeTest(double length,double gap);
		bool   CheckShortestDistance(double i,double j,double neigbhour_pixel_distance);
		void   GenerateRegularGrid(double gap);
		void   ConnectNodes(double distance);
		void   ShowConnections();
		void   SaveSearchSpace();
		void   DetermineCheckPoints();
		void   AddText ( char const * text);
		void   FindRoot();
		void   draw_path();
		void   Print();
		void   FreePath();
		void   PathFollow(Node *,double kd, double kt,double ko,double tracking_distance);
		AstarPlanner(Point start,Point end,double initial_angle,double final_angle,double pixel_size,double radius,GtkWidget*,const char * MapFilename); 
		AstarPlanner();
		~AstarPlanner();
	};
AstarPlanner :: AstarPlanner()
	{
	};
void AstarPlanner::FreePath()
	{
	while(path != NULL) 
		{
		p = path->next;
		delete path;
		path = p;
		}
	};
void AstarPlanner::DetermineCheckPoints() // This Determines the locations of the points to be checked in the Vehicle Coordinates, should be rotated at each node
	{
	//Point edges[4]={{32.5,93},{-32.5,93},{-32.5,-22},{32.5,-22}}; this is the exact dimentions
	int point_index=0,points_per_height,points_per_width;
	double i,j;
	double l = 1 , w = 0.6;   // length and width of the wheelchair, can be modefied at any time.
	double startx,starty;       // The edges of the robot in -ve quadrant
	Point center_of_rotation;   // center of rotation in respect to the center of Area
	center_of_rotation.x =-0.3; // it's 30 cm on the x-axis
	center_of_rotation.y = 0;   // it's on the mid of the Wheels Axes so i assume that y = 0;
	//  TO BE DONE in a smarter and GENERAL WAY / not a priority
	startx = -l/2 - center_of_rotation.x; // am determining here the location of the edges in the robot coordinate system
	starty = -w/2 - center_of_rotation.y; // 
	local_edge_points[0].x = startx; 		local_edge_points[0].y = starty;
	local_edge_points[1].x = startx;		local_edge_points[1].y = w + starty;
	local_edge_points[2].x = l + startx;	local_edge_points[2].y = w + starty;
	local_edge_points[3].x = l + startx; 	local_edge_points[3].y = starty;
	for (int i=0 ;i < 4; i++)
		cout<<"\nEdge->"<< i<<" X="<<local_edge_points[i].x<<" Y="<<local_edge_points[i].y;
	points_per_height = (int)(ceil((l) / (2.0*this->obstacle_radius)));
	points_per_width  = (int)(ceil((w) / (2.0*this->obstacle_radius)));
	this->number_of_point_to_check = points_per_height*points_per_width;
	cout<<"\nPer H ="<<points_per_height<<" Per W="<<points_per_width<<" Total ="<<this->number_of_point_to_check;
	cout<<"\n Obstacle Radius="<<this->obstacle_radius;
	fflush(stdout);
	// The location of the current edges at each NODE
	this->points_to_check = new Point[this->number_of_point_to_check];
	i =(startx + this->obstacle_radius);
	//cout<<"L+startx = "<<l+startx<<"W+starty="<<w+starty;
	for(int r =0; r < points_per_height ; r++ )
		{
			j=(starty + this->obstacle_radius);
			for (int s=0;s < points_per_width;s++)
			{
				this->points_to_check[point_index].x = i;// Angle zero is when robot heading points to the right (right had rule)
				this->points_to_check[point_index].y = j;// 
				point_index++;
				//cout<<"\n I="<<i<<" J="<<j;
				if ( (j+2*this->obstacle_radius) >= (w + starty) ) 
					j += (w + starty - j)/2;
				else 
					j += (2*this->obstacle_radius);
			}
			double m = i + 2*this->obstacle_radius,n=(l + startx);
			//cout<<"\n	m ="<<m<<" n="<<n;
			if (  m >= n  ) 
				{
				i += (l + startx - i)/2;
				//cout<<"\n It happened i="<<i;
				//fflush(stdout);
				}
			else 
				i += (2*this->obstacle_radius);
		}
	cout<<"\n	Last index is="<<point_index;
	this->wheelchair = new Robot();
	this->wheelchair->SetCheckPoints(this->number_of_point_to_check,this->points_to_check);
	for (int k=0;k<this->number_of_point_to_check;k++)
		{
		cout << "\nRobot Edge '"<<k<<"'---> X="<<this->wheelchair->check_points[k].x<<" Y="<<this->wheelchair->check_points[k].y;
		fflush(stdout);
		}
	};
AstarPlanner :: AstarPlanner(Point start, Point point,double initial_angle,double final_an,double pixel_size,double radius,GtkWidget *w,const char * MapFilename)
	{
	this->ConvertPixel(&start);
	this->ConvertPixel(&end);
	this->start.x = start.x;
	this->start.y = start.y;
	this->end.x = end.x;
	this->end.y = end.y;
	this->MapFilename = MapFilename;
	this->initial_angle= initial_angle;
	this->final_angle = final_an;
	this->pixel_size = pixel_size;
	this->obstacle_radius = radius;
	path=p=root=test=NULL;
	this->search_space=this->temp = NULL;
	this->widget = w;
	this->simulate = FALSE;
	this->DetermineCheckPoints();// Determine the Points that should be checked for obstacle avoidance on the wheelchair
	};
AstarPlanner :: ~AstarPlanner()
	{
	delete wheelchair;
	delete [] points_to_check;
	delete Map;
	path=p=root=test=NULL;
	while (this->search_space != NULL)
		{
		temp = this->search_space;
		this->search_space = this->search_space->next;
		delete temp;
		};
	this->FreePath();
	this->AddText("\n	--->>> Allocated Memory FREED <<<---");
	};
void AstarPlanner::AddText ( char const * text)
	{
	GtkWidget * view;
	GtkTextBuffer *text_buffer;
	GtkTextIter     startv, endv;
	view = lookup_widget (GTK_WIDGET(widget),"textview1");
  	text_buffer = gtk_text_view_get_buffer (GTK_TEXT_VIEW (view));
	gtk_text_buffer_insert_at_cursor(text_buffer,text,-1);
      	gtk_text_buffer_get_bounds(text_buffer, &startv, &endv);
        gtk_text_view_scroll_to_iter(GTK_TEXT_VIEW(view), &endv, 0.0, FALSE, 0.0,0.0);
	}
void AstarPlanner::ShowConnections()
	{
	Point loc1,loc2;
	GtkWidget * view;
	view = lookup_widget (GTK_WIDGET(widget),"drawingarea1");
	temp = this->search_space;
	int m=0,n=0;
	while (temp != NULL)
		{
		for (unsigned int i=0; i < temp->children.size();i++)
			{
			loc1 = temp->location;
			ConvertToPixel(&loc1);
			loc2 = temp->children[i]->location;
			ConvertToPixel(&loc2);
	//gdk_draw_line(view->window,(GdkGC*)(view)->style->white_gc,(int)loc1.x,(int)loc1.y,(int)loc2.x,(int)loc2.y);
			m++;
			}
		temp = temp->next;
		n++;
		}
	cout<<"\n---->>> TOTAL NUMBER OF CONNECTIONS ="<<m<<"\n---->>> Total Nodes in search Space ="<<n;
	fflush(stdout);
	this->MAXNODES = m;
	}
void AstarPlanner::ConnectNodes(double allowed_distance)
	{
	SearchSpaceNode * S;
	double distance,angle;
	if (!this->search_space) // Do nothing if Search Space is not Yet Created
		return;
	temp = this->search_space;
	while (temp!=NULL)
		{
		S = this->search_space;
		while (S!=NULL)
			{
			distance = sqrt(pow(S->location.x - temp->location.x,2) + pow(S->location.y - temp->location.y,2));
			if (distance <= allowed_distance && distance !=0 && distance >= 0.2)
				{
					angle = atan2(S->location.y - temp->location.y ,S->location.x - temp->location.x);
					if(!Obstacle(S->location.x,S->location.y,angle))
						temp->children.push_back(S);
				}
			S = S->next;
			}
		temp = temp->next;
		}
	this->AddText(g_strdup_printf("\n	--->>> NODES CONNECTED <<<---	"));
	cout<<"\n	--->>> NODES CONNECTED <<<---	";
	};
void AstarPlanner::AddCostToNodes(double r)
	{
	SearchSpaceNode * S;
	Point  point;
	double number_of_pixels,radius,nearest_obstacle;
	int i,j;
	//int n=0;
	if (!this->search_space) // Do nothing if Search Space is not Yet Created
		return;
	S = this->search_space;
	number_of_pixels = r / this->pixel_size;
	while (S!=NULL)
		{
		//cout<<"\n Node= "<<++n;
		//fflush(stdout);
		point.x = S->location.x;
		point.y = S->location.y;
		this->ConvertToPixel(&point);
		nearest_obstacle = 0;
		for(radius = (int)number_of_pixels ; radius >0 ; radius --)
			{
				for( i= (int)(point.x - radius) ; i < (int)(point.x + radius); i+=5)
					{
						if (i < 0) i = 0;
						if (i >= this->map_width)  break;
						j = (int)(- sqrt(radius*radius - (i - point.x)*(i - point.x)) + point.y);
						if (j < 0) j = 0;
						if (j >= this->map_height) j = this->map_height - 1;
						if(this->Map->mapdata[i][j])
								 nearest_obstacle = radius;
						j = (int)(+ sqrt(radius*radius - (i - point.x)*(i - point.x)) + point.y);
						if (j < 0) j = 0;
						if (j >= this->map_height) j = this->map_height - 1;

						if(this->Map->mapdata[i][j])
								 nearest_obstacle = radius;
					}
			//cout<<"\n R="<<radius;
			//fflush(stdout);
			}
		S->obstacle_cost =  (r - nearest_obstacle*this->pixel_size)/r; // this is a normalized cost, it will make more sense later on 
		//cout<<"\n Obstacle Cost ="<<S->obstacle_cost;
		//getchar();
		S = S->next;
		}
	this->AddText(g_strdup_printf("\n	--->>> NODES CONNECTED <<<---	"));
	cout<<"\n	--->>> NODES CONNECTED <<<---	";
	};
bool AstarPlanner::CheckShortestDistance(double x,double y,double neigbhour_distance)
	{
	SearchSpaceNode * S;
	double distance,shortest_distance = 10000000;

	S = this->search_space;
	while (S!=NULL)
		{
		distance = sqrt(pow(S->location.x - x,2)+pow(S->location.y - y,2));
		if (distance < shortest_distance)
			shortest_distance = distance;
		S = S->next;
		}
	if( shortest_distance > neigbhour_distance )
		return 1;
	else
		return 0;
	};
void AstarPlanner::ExpandObstacles() // Nifty way of doing an obstacle expansion, not finalized, ----->>>>> @ TO DO @ <<<<<------
	{
	if(this->Map->mapdata == NULL )
		{
		this->AddText(g_strdup_printf("\n	--->>> You have to read the Map before Expanding the Obstacles <<<---	"));
		return;
		}
	int thickness;
	int m,n,x,y,radius;
	thickness = int(this->obstacle_radius/this->pixel_size);
	radius =2; // This covers vertical, horizontal and diagonal cells
	for(int i=0;i<this->map_width - 1;i++)
		for(int j=0;j<this->map_height - 1 ;j++)
			{
		for(x = (i - radius) ; x <= (i + radius) ; x ++)
				{
						if (x < 0) x = 0;
						if (x >= this->map_width)  break;
						y = (int)(- sqrt(radius*radius - (x - i)*(x - i)) + j);
						if (y < 0) y = 0;
						if (y >= this->map_height) y = this->map_height - 1;
						if (this->Map->mapdata[i][j] && !this->Map->mapdata[x][y])
							{
							for (int r = 1; r <(int)(thickness);r++)
								for( m = (int)(i - r); m <= (int)(i + r);m++)
									{
										if (m < 0) m = 0;
										if (m >= this->map_width)  break;
										n = (int)(- sqrt(r*r - (m - i)*(m - i)) + j);
										if (n < 0) n = 0;
										if (n >= this->map_height) n = this->map_height - 1;
										this->Map->DrawPixel(0xff,0xff,0xff,m,n);
										n = (int)(+ sqrt(r*r - (m - i)*(m - i)) + j);
										if (n < 0) n = 0;
										if (n >= this->map_height) n = this->map_height - 1;
										this->Map->DrawPixel(0xff,0xff,0xff,m,n);
									}
							break;
							}
						y = (int)(+ sqrt(radius*radius - (x - i)*(x - i)) + j);
						if (y < 0) y = 0;
						if (y >= this->map_height) y = this->map_height - 1;
						if (this->Map->mapdata[i][j] && !this->Map->mapdata[x][y])
							{
							for (int r = 1; r <(int)(thickness);r++)
								for( m = (int)(i - r); m <= (int)(i + r);m++)
									{
										if (m < 0) m = 0;
										if (m >= this->map_width)  break;
										n = (int)(- sqrt(r*r - (m - i)*(m - i)) + j);
										if (n < 0) n = 0;
										if (n >= this->map_height) n = this->map_height - 1;
										this->Map->DrawPixel(0xff,0xff,0xff,m,n);
										n = (int)(+ sqrt(r*r - (m - i)*(m - i)) + j);
										if (n < 0) n = 0;
										if (n >= this->map_height) n = this->map_height - 1;
										this->Map->DrawPixel(0xff,0xff,0xff,m,n);
									}
							break;
							}
				}
			}
	cout<<"\n	--->>> OBSTACLES EXPANDED SUCCESSFULLY <<<---	";
	this->Map->ReadMap(); // we changed the pixel buffer, so reading the image again will regenerate the free space / MapData ;)
	this->Map->SavePixelBufferToFile(); // saving the newly generated space, default file is mapname_FreeSpace.jpeg
	};
void AstarPlanner::SaveSearchSpace()
	{
	Point p;
	temp = this->search_space;
	while (temp != NULL)
		{
		p = temp->location;
		this->ConvertToPixel(&p);
		this->Map->DrawPixel(0x00,0xff,0x00,int(p.x),(int)p.y);// Drawing the new nodes into the Pixel Buffer
		temp =temp->next;
		}
	this->Map->SavePixelBufferToFile(); // Saving the newly generated Pixel Buffer into the file
	}
void AstarPlanner::GenerateRegularGrid(double distance)
	{
	if(this->Map->mapdata == NULL )
		return;
	Point p;
	for(int i=0; i<this->map_width; i++)
		{
		for(int j=0;j<this->map_height;j++)
			{
			if (!this->Map->mapdata[i][j]) //Free Pixel
				{
				if (this->search_space == NULL ) // Constructing the ROOT NODE
					{
					temp = new SearchSpaceNode;
					temp->location.x = i;
					temp->location.y = j;
					this->ConvertPixel(&temp->location);
					temp->parent   = NULL;
					temp->next     = NULL;
					search_space = temp;
					}
				else
					{
					p.x = i;
					p.y = j;
					this->ConvertPixel(&p);
						if (CheckShortestDistance(p.x,p.y,distance))
							{
							temp = new SearchSpaceNode;
							temp->location.x = p.x;
							temp->location.y = p.y;
							temp->parent = NULL; 
							temp->next   = search_space;
							search_space = temp;
							}
					}
				}
			}
		}
	cout<<"\n	--->>> REGULAR GRID GENERATED SUCCESSFULLY <<<---	";
	this->SaveSearchSpace();
	};
void AstarPlanner::BridgeTest(double length, double neigbhour_distance)
	{
	int radius,pixels_per_bridge;
	Point p;
	double x,y,x2,y2;
	pixels_per_bridge = (int) (length/this->pixel_size);
	radius = (int) (length/(this->pixel_size*2.0));
	for(int i=0; i < this->map_width - pixels_per_bridge; i++)
		{
		for(int j=0;j<this->map_height - pixels_per_bridge ;j++)
			{
			int ab=0;
					for( x = (i - radius) ; x < (i + radius) ; x+=(radius/2.0))//x+=(radius))
						{
							// The circle is of Center (i,j) and radius R=radius
							ab++;
							if (x < 0) x = 0;
							if (x > this->map_width) break;						
							y = (int)(+ sqrt(radius*radius -  ( x - i)*( x - i)) + j);
							if (y < 0) y = 0;
							if (y >= this->map_height) y = this->map_height - 1;

							x2 = i + (i - x);
							if (x2 < 0) x2 = 0;
							if (x2 > this->map_width) break;	
							y2 = (int)(- sqrt(radius*radius - (x2 - i)*(x2 - i)) + j);
							if (y2 < 0) y2 = 0;
							if (y2 >= this->map_height) y2 = this->map_height - 1;
						//cout<<"\n x="<<x<<" y="<<y<<" x2="<<x2<<" y2="<<y2<<" i="<<i<<" j="<<j<<" R="<<radius;
						//fflush(stdout);
						if (!this->Map->mapdata[i][j]&&this->Map->mapdata[(int)x][(int)y]&&this->Map->mapdata[(int)x2][(int)y2])
							{
								p.x = i;
								p.y = j;
								this->ConvertPixel(&p);
								if (this->search_space == NULL ) // Constructing the ROOT NODE
									{
									temp = new SearchSpaceNode;
									temp->location.x = p.x;
									temp->location.y = p.y;
									temp->parent   = NULL;
									temp->next     = NULL;
									search_space = temp;
									}
								else
									{
										if (CheckShortestDistance(p.x,p.y,neigbhour_distance))
											{
											temp = new SearchSpaceNode;
											temp->location.x = p.x;
											temp->location.y = p.y;
											temp->parent = NULL;
											temp->next   = search_space;
											search_space = temp;
											}
									}
							}
						}
			//cout<<"\n I have iterated = "<<ab;
			//fflush(stdout);
			}
		}
	cout<< "\n	--->>> BRIDGE TEST FINISHED SUCCESSFULLY <<<---";
	this->SaveSearchSpace();
	};

void AstarPlanner :: ReadMap()
	{
	Map = new MapFile(this->MapFilename,pixel_size);
	this->pixbuf = Map->ReadMap();
	this->mapinfo = Map->GetMapInfo();
	this->map_width=mapinfo.width;
	this->map_height=mapinfo.height;
	this->MAXNODES=map_height*map_width;
	};
double AstarPlanner :: Absolute(double x)
	{
	if (x>=0)
		return x;
	else
		return -x;
	};
void AstarPlanner :: ConvertPixel(Point  *p) // transfers from pixel coordinate to the main coordinate system
	{
	p->x= p->x*this->pixel_size - this->pixel_size*this->map_width/2;
	p->y=-p->y*this->pixel_size + this->pixel_size*this->map_height/2;
	return ;
	};
void AstarPlanner ::ConvertToPixel (Point *p)
	{
	p->x=( p->x + this->pixel_size*this->map_width/2)/this->pixel_size;
	p->y=(-p->y + this->pixel_size*this->map_height/2)/this->pixel_size;
	}
void AstarPlanner :: PrintNodeList()
{
	int step=1;
	Point  pixel;
	if(!(p = this->path))
		return ;
	cout <<"\n  --------------------   START OF LIST ---------------------- \n";
	while(p !=NULL)
		{
		pixel =  p->NodeInfo;
		//ConvertToPixel(&pixel);
		cout << "\nStep [" << step++ <<"] corresponding to Pixel x["<< pixel.x<<"]y["<<pixel.y<<"]"; 
		for(int i=0;i<this->number_of_point_to_check;i++)
			{
			if (p->wheelchair.check_points.size() == 0)
				{
				cout<<"\n FOR some FUCKING REASON AM EMPTY :| !!!!!"; //SHOULD NEVER HAPPEN :@
				break;
				}
			}
		fflush(stdout);
		p = p->next;
		}
	cout <<"\n\n  --------------------   END OF LIST ---------------------- \n";
}
void AstarPlanner::FindRoot() // find the nearest node to the start
	{
	if(!this->search_space)
		return;
	double distance,shortest_distance = 100000;
	root = new Node;	// allocate and setup the root node
	temp = this->search_space;
	while(temp!=NULL)
		{
		distance = sqrt(pow(temp->location.x - start.x,2) + pow(temp->location.y - start.y,2));
		if (distance < shortest_distance) // Initialize the root node information and put it in the open list
			{
			shortest_distance = distance;
			root->NodeInfo.x = temp->location.x;
			root->NodeInfo.y = temp->location.y;
			}
		temp = temp->next;
		}
	root->parent = NULL;
	root->next = NULL;
	root->prev = NULL;
	root->g_value = 0;;
	root->h_value = Calculate_h(root);
	root->f_value = root->g_value + root->h_value;
	root->id = 0;
	root->depth = 0;
	root->state = OPEN;
	root->angle=this->initial_angle;
	Translate(root->NodeInfo,this->initial_angle);
	root->wheelchair.SetCheckPoints(this->number_of_point_to_check,this->points_to_check);
	cout<<"\n---->>>Root is Set to be X="<<root->NodeInfo.x<<" Y="<<root->NodeInfo.y;
	fflush(stdout);
	};
int AstarPlanner :: StartSearch(Point start_search, Point end_search,double initial_angle, double final_angle)
{
	int      gblID = 1;
  	int      gblExpand = 0;
	this->initial_angle = initial_angle;
	this->final_angle = final_angle;
	cout <<"\n	--->>> 1- Initial angle="<<RTOD(this->initial_angle)<<" Final Angle="<<RTOD(this->final_angle);
	fflush(stdout);
	if (!this->Map) // Make sure that we have a map to search
		{
		this->Map->ReadMap();
		cout <<"\nRead the Map File First !!!"<<endl;
		fflush(stdout);
		this->AddText("\nRead the Map File First !!!");
		return -1;
		}
	if (!this->search_space)
		{
		cout <<"\n	--->>> Generate Search Space First !!!"<<endl;
		fflush(stdout);
		this->AddText("\n	--->>> Generate Search Space First !!!");
		return -1;
		}
	this->ConvertPixel(&start_search);
	this->ConvertPixel(&end_search);
	this->start.x = start_search.x;
	this->start.y = start_search.y;
	this->end.x   = end_search.x;
	this->end.y   = end_search.y;
	cout <<"\n	--->>> Search Started <<<---";
	fflush(stdout);
	FindRoot();
	cout <<"\n---->>>Target is Set to be X="<<end.x<<" Y="<<end.y<<" <<<---";
	fflush(stdout);
  	openList = NULL;  // generate the open list
  	closedList = NULL;// generate the closed list
  	openList = root;
	while (openList != NULL) 
	{
    		current = openList;	                      // remove the first node from the open list, as it will always be sorted
    		openList = openList->next;
    		if(openList != NULL)
    			openList->prev = NULL;
    		gblExpand++;
    		if (GoalReached(current))                     // We reached the target location, so build the path and return it.
		{
			// build the complete path to return
      			current->next = NULL;
      			path = current;
      			p = current->parent;
    			printf("\n	--->>> Goal state reached with %d nodes created and %d nodes expanded <<<---\n", gblID, gblExpand);
			fflush(stdout);
			cout<<"\n	--->>> General Clean UP <<<---";
			fflush(stdout);
			int m=0;
      			p = current;
   			while (p != NULL) 
				{
				cout<<"\n	--->>> Step["<<++m<<"] X="<<p->NodeInfo.x<<" Y="<<p->NodeInfo.y;
				//cout<<"\n	--->>> Angle is="<<RTOD(p->angle);
				fflush(stdout);
				p = p->parent;
				}
      			p = current->parent;
   			while (p != NULL) 
				{
				//cout<<"\n Am Still HERE Step["<<++m<<"] X="<<p->NodeInfo.x<<" Y="<<p->NodeInfo.y;
				//fflush(stdout);
				if (p->prev != NULL) 			// remove the parent node from the closed list (where it has to be)
					(p->prev)->next = p->next;
				if (p->next != NULL)
					(p->next)->prev = p->prev;
				if (p == closedList)			// check if we're removing the top of the list
		  			closedList = p->next;
				p->next = path;				// set it up in the path
				path = p;
				p = p->parent;
				}
        		// now delete all nodes on OPEN
			cout<<"\n	--->>> Freeing open list <<<---";
			fflush(stdout);
			while (openList != NULL) 
				{
				p = openList->next;
				FreeNode(openList);
				openList = p;
      				}
      			// now delete all nodes on CLOSED
			cout<<"\n	--->>> Freeing closed list <<<---";
			fflush(stdout);
      			while (closedList != NULL) 
				{
				p = closedList->next;
				FreeNode(closedList);
				closedList = p;
      				}
      			return 1; // Path Found Successfully
    		}
		if(!(childList = MakeChildrenNodes(current))) // No more Children => Search Ended Unsuccessfully at this path Branch
		{
			cout<<"\n	--->>> Search Ended On this Branch / We Reached a DEAD END <<<---";
			fflush(stdout);
		}
    		while (childList != NULL)                     // insert the children into the OPEN list according to their f values
		{
      			curChild  = childList;
      			childList = childList->next;
			// set up the rest of the child node details
      			curChild->parent = current;
      			curChild->state = OPEN;
      			curChild->depth = current->depth + 1;
      			curChild->id = gblID++;
      			curChild->next = NULL;
      			curChild->prev = NULL;
      			curChild->g_value = Calculate_g(curChild);
	      		curChild->h_value = Calculate_h(curChild);
      			curChild->f_value = curChild->g_value + curChild->h_value;
   			if (closedList) // test whether the child is in the closed list (already been there)
			{
				p = closedList;
				while (p) 
				{
	  			if (NodeEquality(p, curChild)) 	       		   // if so, check this child already exists in the closed list
					{
	  				if (p->f_value <= curChild->f_value)       // if the f value of the older node is lower, delete the new child
						{
							//cout<<"\n	--->>> Closed list -- Current Child deleted cause it has a longer path<<<---";
							//fflush(stdout);
			        			FreeNode(curChild);
	      						curChild = NULL;
	      						break;
	    					}
	    				else // the child is a shorter path to this point, delete p from  the closed list
						{
						// This is the tricky part, it rarely happens, but in my case it happenes all the time :s
						// Anyways, we are here cause we found a better path to a node that we already visited, we will have to
						// Update the cost of that node and ALL ITS DESCENDENTS because their cost is parent dependent ;)
						// Another Solution is simply to comment everything and do nothing, doing this, the child will be added to the
						// Open List and it will be investigated further more later on.
/*							cout<<"\n	--->>> Closed list -- Node is deleted, current child X="<<curChild->NodeInfo.x<<" Y="<<curChild->NodeInfo.y<<" has shorter path<<<---";
							fflush(stdout);
							if (p->prev != NULL)	        
					      			(p->prev)->next = p->next;
			      				if (p->next != NULL)	        
								(p->next)->prev = p->prev;								
	      						if (p == closedList)
								closedList     = p->next;
							p->f_value = curChild->f_value;
							//p->parent = curChild->parent;
							//curChild->parent = p->parent;
							q = openList;
							while(q)	// Update the DESCENDENTS with the new cost
								{
									if(NodeEquality(p,q->parent))
										{
										//q->parent = curChild;
      										q->g_value = Calculate_g(q);
	      									q->h_value = Calculate_h(q);	
      										q->f_value = q->g_value + q->h_value;
										}
								q = q->next;
								}
							bool sorted = FALSE;
							Node * tmp;
							while (!sorted) // Bubble sorting the open list
								{
									q = openList;
									while (q && q->next)
									{
										sorted = TRUE;
										if(q->next->f_value < q->f_value)
											{
												tmp = q->next;
												q->next = tmp->next;
												tmp->prev = q->prev;	
												q->prev = tmp;
												tmp->next = q;
												if(q->prev) 
													q->prev->next = tmp;
												if(tmp->next)
													tmp->next->prev = q;
												if (q == openList)
													openList = tmp;
												sorted = FALSE;
											}
									q = q->next;
									}
								}
			        			FreeNode(curChild);
	      						curChild = NULL;
	      						//FreeNode(p);*/
	      						break;
	    					}
	  				}
	  			p = p->next;
				}
      			}	
			if (curChild) 		       // check if the child is already in the open list
			{
				p = openList;
				while (p)
				{
				if (NodeEquality(p, curChild)) // child is on the OPEN list
					{
					if (p->f_value <= curChild->f_value)  // child is a longer path to the same place so delete it
						{
							//cout<<"\n	--->>> Open list -- Current Child Deleted cause it exist in the  with longer path<<<---";
							//fflush(stdout);
							FreeNode(curChild);
							curChild = NULL;
							break;
						}
					else 			              // child is a shorter path to the same place so remove the duplicate node
						{
							//cout<<"\n	--->>> Open list -- Duplicated node deleted cause current child has shorter path <<<---";
							//fflush(stdout);
							if (p->prev != NULL)
								(p->prev)->next = p->next;
							if (p->next != NULL)
								(p->next)->prev = p->prev;
							if (p == openList)
								openList = p->next;
							FreeNode(p);
							break;
						}
					}
				p = p->next;
				}
			if (curChild)
				{
					p = openList;
					q = p;
					while (p) // now insert the child into the open list according to the f value
					{
	    				if (p->f_value >= curChild->f_value) 	       // insert before p
						{
							//cout<<"\n	--->>> Child added to the Open List <<<---";
							//fflush(stdout);
	      						if (p == openList)// test head of the list case
								openList = curChild;	
		      					curChild->next = p;
	      						curChild->prev = p->prev;
	      						p->prev = curChild;
	      						if (curChild->prev)
								(curChild->prev)->next = curChild;
		      					break;
	    					}
	    				q = p;
	    				p = p->next;
	  				}		
	  				if (p == NULL)       
					{
						//cout<<"\n	--->>> Inserting Child at Start or End of the Open List<<<---";
						//fflush(stdout);
						if (q != NULL) // insert at the end
						{
				      			q->next = curChild;
		      					curChild->prev = q;
	    					}
	    					else	      // insert at the beginning
							openList = curChild;
	  				}
				}       
			} 	 		
    		} 	 	// Children Node loop end       
		// put the current node onto the closed list, ==>> already visited List
    		current->next = closedList;
    		if (closedList != NULL)
      			closedList->prev = current;
    		closedList = current;
		current->prev = NULL;
    		current->state = CLOSED;
    		if (current->id > this->MAXNODES)     	// Test to see if we have expanded too many nodes without a solution
		{
	    		cout<<"\n	--->>>	Expanded more than the maximum allowable nodes of MAXNODE="<<this->MAXNODES<<" Terminating ...";
			fflush(stdout);
			while (openList != NULL)     	// delete all nodes on OPEN
			{
				p = openList->next;
				FreeNode(openList);
				openList = p;
      			}
      			while (closedList != NULL)      // delete all nodes on CLOSED
			{
				p = closedList->next;
				FreeNode(closedList);
				closedList = p;
      			}
			return 0; // Expanded more than the maximium nodes state
    		}
// This part is for testing reasons ONLY
/*		p = openList;
		int r =0,o;
		while (p) // Printing Open List   -->> for debugging reasons
		{
			cout<<"\n--->>> ["<<++r<<"]- Open List X="<<p->NodeInfo.x<<" Y="<<p->NodeInfo.y<<" Cost ="<<p->f_value;
			q = p->parent;
			o = 0;
			while (q)
				{
				cout<<"\n	--->>> Parent ["<<++o<<"] X="<<q->NodeInfo.x<<" Y="<<q->NodeInfo.y<<" Cost ="<<q->f_value;
				//if (o >10) break;
				q = q->parent;
				if (!q) cout <<"\n	--->>>ROOT<<<---";
				}
			fflush(stdout);
			p = p->next;
		}
		p = closedList;
		r =0;
		while (p) // Printing Closed List -->> for debugging reasons
		{
			cout<<"\n--->>> ["<<++r<<"]- Closed List X="<<p->NodeInfo.x<<" Y="<<p->NodeInfo.y<<" Cost="<<p->f_value;
			fflush(stdout);
			q = p->parent;
			o = 0;
			while (q)
				{
				cout<<"\n	--->>> Parent ["<<++o<<"] X="<<q->NodeInfo.x<<" Y="<<q->NodeInfo.y<<" Cost ="<<q->f_value;
				//if (o >10) break;
				q = q->parent;
				if (!q) cout <<"\n	--->>>ROOT<<<---";
				}
			p = p->next;
		}
		getchar();*/
 	}					       // end of OPEN loop
  	// if we got here, then there is no path to the goal
  	// delete all nodes on CLOSED since OPEN is now empty
  	while (closedList != NULL) 
	{
    		p = closedList->next;
    		FreeNode(closedList);
    		closedList = p;
  	}
  	return -1; // No Path Found
};
double AstarPlanner:: Calculate_g(Node *n) // define the g function as parent plus 1 step
{
	double cost;
	if(n == NULL)
		return 0.0;
	if(n->parent == NULL)
		return 0.0;
	cost = n->parent->g_value + sqrt(pow(n->NodeInfo.x - n->parent->NodeInfo.x,2) + pow(n->NodeInfo.y - n->parent->NodeInfo.y,2) );
	return cost;
};
double AstarPlanner::Calculate_h(Node *n) //define the h function as the Euclidean distance to the goal + turning penalty
{
	double h,angle_cost,obstacle_penalty;
	if(n == NULL)
		return(1e+5);
	// Using the Euclidean distance
	h = sqrt(pow(this->end.x - n->NodeInfo.x,2) + pow(this->end.y - n->NodeInfo.y,2));
	if (n->parent != NULL) // Adding the Angle cost, we have to uniform the angle representation to the +ve rep or we well get a non sense result
		{
		if (n->parent->angle < 0 )	
			if (n->angle <0)
				angle_cost = (DTOR(360) + n->parent->angle) - (n->angle + DTOR(360)); // in Radians
			else 
				angle_cost = (DTOR(360) + n->parent->angle) - n->angle; // in Radians
		else
			if (n->angle <0)
				angle_cost = n->parent->angle - (n->angle + DTOR(360)); // in Radians
			else
				angle_cost = n->parent->angle - n->angle; // in Radians
		}
	obstacle_penalty = n->nearest_obstacle; 
	// this has a maximium value of 1 because it's normalized 
	// Now the trick is to wisely distribute the weights of the costs
	// My priority is to have a path that is smooth and goes in the middle of the free space
	// That's why i will make the obstacle cost propotional to the angle cost
	// 0.555 is the AXLE Length , we don't care which direction we are turning, it's a turn anyways
	return ( h + 0.555 * Absolute(angle_cost) + obstacle_penalty );
}

// define the goalNode function

bool AstarPlanner :: GoalReached (Node *n) 
{
	double a,b;
	if ((a = RTOD(n->angle))          < 0 ) a +=360;
	if ((b = RTOD(this->final_angle)) < 0 ) b +=360;
	if (sqrt(pow(n->NodeInfo.x - this->end.x,2)+pow(n->NodeInfo.y - this->end.y,2)) <= 0.3 && Absolute(a - b)<=60)  
		return 1;
	return 0;
};
void AstarPlanner::Translate(Point  P, double theta) // Rotates and Translates the check points according to the vehicle position and orientation
	{
	for(int i=0;i<this->number_of_point_to_check;i++)
		{
		this->points_to_check[i].x = (this->wheelchair->check_points[i].x*cos(theta) - this->wheelchair->check_points[i].y*sin(theta) + P.x);
		this->points_to_check[i].y = (this->wheelchair->check_points[i].x*sin(theta) + this->wheelchair->check_points[i].y*cos(theta) + P.y);
		//cout<<" After Conversion X="<<this->points_to_check[i].x<<" Y="<<this->points_to_check[i].y;
		}
	};
void AstarPlanner::Translate_edges(Point  P, double theta) // Rotates and Translates the check points according to the vehicle position and orientation
	{
	for(int i=0;i<4;i++)
		{
		this->translated_edge_points[i].x = (this->local_edge_points[i].x*cos(theta) - this->local_edge_points[i].y*sin(theta) + P.x);
		this->translated_edge_points[i].y = (this->local_edge_points[i].x*sin(theta) + this->local_edge_points[i].y*cos(theta) + P.y);
		cout<<" \nAfter Conversion X="<<this->translated_edge_points[i].x<<" Y="<<this->translated_edge_points[i].y;
		}
	};
// Test for whether a point is in an obstacle or not
int AstarPlanner :: Obstacle(double x, double y, double angle) 
{
	Point P,s;
	int m,n;
	P.x = x;
	P.y = y;
	Translate(P,angle); // Rotates and Translates the check points according to the vehicle position and orientation
	for (int i=0;i<this->number_of_point_to_check;i++)
	{
		this->ConvertToPixel(&this->points_to_check[i]);
		m = (int)this->points_to_check[i].x;
		n = (int)this->points_to_check[i].y;
		//cout <<"\nx ="<<m<<" y="<<n;
		//fflush(stdout);
		if (m <= 0 || n <= 0 || m >=this->map_width || n >=this->map_height)
			return 1;
		if (this->Map->mapdata[m][n]) return 1;
	};	
	return 0;
};
void AstarPlanner::draw_path()
{
  if (!this->path)
	{
		cout<<"\n->NO Path Generated Yet, plan a Path First";
		return;
	}
	GtkWidget * temp;
	double angle;
	temp = lookup_widget (GTK_WIDGET(widget),"drawingarea1");
	Point l_start,l_end,E,S;
  	Node *p;
  	p = this->path;
	while(p != NULL && p->next!=NULL)
	{
		S = l_start =  p->NodeInfo;
		ConvertToPixel(&l_start);
		E = l_end   =  p->next->NodeInfo;
		ConvertToPixel(&l_end);
  		temp = lookup_widget (widget,"drawingarea1");
		if(p->parent!=NULL)
		{
			angle = atan2(E.y - S.y,E.x - S.x); // Angle in Radians
			Translate_edges(S,angle);
			for (int i=0 ;i < 4; i++)
			{
				this->ConvertToPixel(&translated_edge_points[i]);
				cout<<"\nEdge "<< i+1<<" X="<<translated_edge_points[i].x<<" Y="<<translated_edge_points[i].y;
			}
			cout<<"\n";
 		gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)translated_edge_points[0].x,(int)translated_edge_points[0].y,
		(int)translated_edge_points[1].x,(int)translated_edge_points[1].y);
  		gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)translated_edge_points[1].x,(int)translated_edge_points[1].y,
		(int)translated_edge_points[2].x,(int)translated_edge_points[2].y);
  		gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)translated_edge_points[2].x,(int)translated_edge_points[2].y,
		(int)translated_edge_points[3].x,(int)translated_edge_points[3].y);
  		gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)translated_edge_points[3].x,(int)translated_edge_points[3].y,
		(int)translated_edge_points[0].x,(int)translated_edge_points[0].y);

  		gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)l_start.x,(int)l_start.y,(int)l_end.x,(int)l_end.y);
		}
		p = p->next;
	} 
};
Node *AstarPlanner :: MakeChildrenNodes(Node *parent) 
{
	Point P; // grand parents  and parent node information
	Node  *p, *q;
	double angle,angle_difference,discrete_angle,parent_angle,child_angle;//,angle_resolution = DTOR(20);
	bool collides = FALSE;
	P.x = parent->NodeInfo.x;
	P.y = parent->NodeInfo.y;
	if(!this->search_space)
		return NULL;
	temp = this->search_space;
	while(temp!=NULL)
		{
		if (temp->location.x == P.x && temp->location.y == P.y)
			break;
		temp = temp->next;
		}
	if (!temp) 
		{
	    	cout<<"\n	--->>>	Node not found in the search Space :)";
		fflush(stdout);
		return NULL;
		}
	q = NULL;
	for (unsigned int i=0;i<temp->children.size();i++) // Check Each neighbour
		{
		angle = atan2(temp->children[i]->location.y - P.y,temp->children[i]->location.x - P.x); // Angle in Radians
		if ((parent_angle = parent->angle) < 0) parent_angle +=DTOR(360);
		if ((child_angle = angle) < 0) 	        child_angle  +=DTOR(360);
		angle_difference = Absolute(parent_angle - child_angle);
		if (angle_difference > DTOR(120))
			continue;
		discrete_angle =  parent_angle;
		//cout<<"\n I="<<i<<" Diff Angle="<<RTOD(angle_difference);
		//fflush(stdout);
		collides =FALSE;
/*		for (int s=0 ; s <= ceil(angle_difference/angle_resolution); s++)
			{
				if (parent_angle > child_angle)
					{
						discrete_angle -= angle_resolution;
						if (discrete_angle < child_angle)
							discrete_angle = child_angle;
					}
				else
					{
						discrete_angle += angle_resolution;
						if (discrete_angle > child_angle)
							discrete_angle = child_angle;
					}
				if (Obstacle(temp->children[i]->location.x,temp->children[i]->location.y,discrete_angle))
					{
					collides= TRUE;
					break;
					}
			}
*/
		if (!collides)
					{
						p = new Node;
						p->NodeInfo.x = temp->children[i]->location.x;
						p->NodeInfo.y = temp->children[i]->location.y;
						p->nearest_obstacle = temp->obstacle_cost;
						p->parent = parent;
						p->angle= angle;
						p->wheelchair.SetCheckPoints(this->number_of_point_to_check,this->points_to_check);
						p->next = q;
						q = p;
					}
		//cout<<"\n I="<<i<<" Dis Angle="<<RTOD(discrete_angle);
		//fflush(stdout);
		}
	return q;
}
// Free node function
void AstarPlanner :: FreeNode(Node *n) 
{
	delete n;
}

int AstarPlanner :: NodeEquality(Node *a, Node *b) // Test for node equality
{
	if (a->NodeInfo.x == b->NodeInfo.x && a->NodeInfo.y == b->NodeInfo.y)
		return 1;
	return 0;
}
double Magnitude( Point p1, Point p2 )
{
    Point Vector;
    Vector.x = p2.x - p1.x;
    Vector.y = p2.y - p1.y;
    return sqrt( Vector.x * Vector.x + Vector.y * Vector.y);
}
double AstarPlanner :: DistanceToLine( Point LineStart, Point LineEnd, Point P)
{
	double LineMag,distance;
	LineMag = Magnitude(LineStart,LineEnd);
	distance = (P.x*(LineStart.y - LineEnd.y) + P.y*(LineEnd.x - LineStart.x)+(LineStart.x*LineEnd.y - LineEnd.x*LineStart.y))/LineMag ;
    	return distance;
};
double AstarPlanner :: DistToLineSegment(Point LineStart, Point LineEnd, Point p)
{
	Vector2D A(LineStart.x,LineStart.y),B(LineEnd.x,LineEnd.y),P(p.x,p.y);
  	//if the angle between PA and AB is obtuse then the closest vertex must be A
  	double dotA = (P.x - A.x)*(B.x - A.x) + (P.y - A.y)*(B.y - A.y);
  	if (dotA <= 0) 
		return Vec2DDistance(A, P);
	//if the angle between PB and AB is obtuse then the closest vertex must be B
  	double dotB = (P.x - B.x)*(A.x - B.x) + (P.y - B.y)*(A.y - B.y);
   	if (dotB <= 0) 
		return Vec2DDistance(B, P);
   	//calculate the point along AB that is the closest to P
  	//Vector2D Point = A + ((B - A) * dotA)/(dotA + dotB);
	//calculate the distance P-Point
  	//return Vec2DDistance(P,Point);
	return DistanceToLine(LineStart,LineEnd, p); 
}
void  AstarPlanner :: PathFollow(Node * p,double kd, double kt,double ko,double tracking_distance)
{
	/**********************Variable Decleration************************/
  	/* init threads */    
	int direction;
	Point begin,amcl_location,tracking_point,ni,SegmentStart,SegmentEnd,EstimatedPos;
	double angle,prev_angle,theta,error_orientation,displacement,wdem,distance,distance_to_next,obstacle_avoidance_force;
	bool position_found=FALSE,end_reached=FALSE,segment_navigated=FALSE;
	Node  *last,*first;
	double pose[3], pose_var[3][3],minR,minL,minAhead,avoidance_distance_left,avoidance_distance_right,avoidance_distance_ahead;
	Point old_amcl;
    file=fopen("timing.txt","wb");
	PlayerClient robot("127.0.0.1", 6665);
	PositionProxy pp(&robot,0,'a');
	LaserProxy laser(&robot,0,'r');
	LocalizeProxy localizer(&robot,0,'r');
	//robot.SetFrequency(100);
	avoidance_distance_left  = 0.2 + 0.5;
	avoidance_distance_right = 0.3;
	avoidance_distance_ahead = 0.2;
  	SDL_Event event ;
  	if (SDL_Init(SDL_INIT_VIDEO) != 0)
  		{
  		  printf("Unable to initialize SDL: %s\n", SDL_GetError());
  		  return ;
  		}
	//When this program exits, SDL_Quit must be called
  	atexit(SDL_Quit);
  	//Set the video mode to anything, just need a window
  	screen = SDL_SetVideoMode(320, 240, 0, SDL_ANYFORMAT);
  	if (screen == NULL)
   		{
    		printf("Unable to set video mode: %s\n", SDL_GetError());
    		return ;
  		}
	/*****************************************************************/
 	if(localizer.access == 'e')
    		{
  		this->AddText( "\n	--->>> Can't read from localizer <<<---" );
  		exit(-1);
   		} 
 	if(laser.access != 'r')
    		{
      		this->AddText( "	--->>> Can't read from laser" );
      		exit(-1);
    		}  
  	if(pp.GetAccess() == 'e') 
	  	{
    		this->AddText("\n	--->>> Error getting position device access! <<<---");
    		exit(1);
  		}
//	if(WCp.GetAccess() == 'e')
//		{
//		this->AddText("\n	--->>> Error getting wheelchair device atracking_distanceccess! <<<---");
//			exit(1);
//		}		
	printf("%s\n",robot.conn.banner);
	/************** Wheelchair ON Automatic Mode **********************/
//	WCp.SetPower(OFF);
//	usleep(200000);
//	WCp.SetPower(ON);
//	usleep(200000);
//	WCp.SetMode(AUTO);
//	usleep(200000);
//	this->AddText("\n	--->>> WheelChair is ON and CONTROL Mode is AUTO <<<---\n");
//	fflush(stdout);
	/*****************************************************************/
	if (!p) // path not passed throught the arguments 
		return;
	this->ConvertPixel(&this->start);
	//pose[0]= begin.x;
	//pose[1]= begin.y;
	//pose[2]=DTOR(this->initial_angle);
	pose[0]= p->NodeInfo.x;
	pose[1]= p->NodeInfo.y;
	pose[2]=p->angle;
	cout << "\n Default Pose given to the Localizer X="<<p->NodeInfo.x<<" Y="<<p->NodeInfo.y<<" Theta="<<p->angle;
	cout << "\n Tracking Distance="<<tracking_distance<<" Kd="<<kd<<" KTheta="<<kt;
	pose_var[0][0]=0.5;
	pose_var[0][1]=0.5;
	pose_var[0][2]=0.5;
	pose_var[1][0]=0.5;
	pose_var[1][1]=0.5;
	pose_var[1][2]=0.5;
	pose_var[2][0]=0.5;
	pose_var[2][1]=0.5;
	pose_var[2][2]=DTOR(10);
	localizer.SetPose(pose,pose_var);
	/***********************Finding Out where we are from the Localizer with initial estimation ************************/
	while(!position_found) //wait until we have 99% accurate assumption / hypothesis
	{
		if(robot.Read()) 
			exit(1);
		printf("%d hypotheses\n", localizer.hypoth_count); // Assumed number of Hypothesis
    	printf("%d (weight %f): [ %f %f %f ]\n",0,localizer.hypoths[0].weight,localizer.hypoths[0].mean[0],localizer.hypoths[0].mean[1], 	localizer.hypoths[0].mean[2]);
		if(localizer.hypoths[0].weight>=0.9) // Since the hypothesis are sorted, we assume the first one to be the one with the most weight
			{
			position_found=1; // Accurate Hypothesis found, so update current location and move next
			old_amcl.x = EstimatedPos.x=amcl_location.x= localizer.hypoths[0].mean[0];
			old_amcl.y = EstimatedPos.y=amcl_location.y= localizer.hypoths[0].mean[1];
			estimate_theta=localizer.hypoths[0].mean[2];
			localizer.SetPose(pose,pose_var);
			}
	usleep(100000);
	}
	usleep(10000000);
	/********************** Start from the root and move forward ************************/
	first = p;
	prev_angle=p->angle;
	p = p->next; // The first angle is always zero
	/************************************************************************************/
	timer2 = g_timer_new();
	delta_timer = g_timer_new();
	last_time=0;
	delta_t=0;
	velocity=0;
while(!end_reached) 
	{
	g_timer_start(timer2);
	g_timer_start(delta_timer);
	if(robot.Read()) 
		exit(1);
	last = p;
	ni = first->NodeInfo;
	SegmentStart.x = ni.x;
	SegmentStart.y = ni.y;
	printf("\n	--->>>NEW SEGMENT Initial Location of  straight line Path is x[%.3f]y[%.3f] <<<---",SegmentStart.x,SegmentStart.y);
	fflush(stdout);
	ni = last->NodeInfo;
	SegmentEnd.x = ni.x;
	SegmentEnd.y = ni.y;	
	printf("\n	--->>>NEW SEGMENT Last  Location of this straight line Path is x[%.3f]y[%.3f] <<<---",SegmentEnd.x,SegmentEnd.y);
	fflush(stdout);
	direction = -1;
	angle = atan2(SegmentEnd.y - SegmentStart.y,SegmentEnd.x - SegmentStart.x);
	printf("\n	--->>> Angle is:=%.3f <<<---",RTOD(angle));
	//getchar();
	segment_navigated = FALSE;
	while (!segment_navigated) // Loop for each path segment
	{
	while (gtk_events_pending())
    	gtk_main_iteration();
	SDL_PollEvent(&event);
		if(event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_q)
			{
				segment_navigated = TRUE;
				end_reached = TRUE;
				cout<<"\n\n Navigation Ended by USER \n\n";
				break;
			}
			//if(robot.Peek(0))
			if(robot.Read()) 
					exit(1);
			if (velocity!=pp.Speed())
				velocity = pp.Speed();
		obstacle_avoidance_force = 0;
		delta_t=g_timer_elapsed(delta_timer, NULL );
		estimate_theta += pp.SideSpeed()*delta_t;
		EstimatedPos.x += velocity*cos(estimate_theta)*delta_t;
		EstimatedPos.y += velocity*sin(estimate_theta)*delta_t;
		printf("\n--->>> Velocity =%.3f m/sev EstimateX=[%.3f] EstimateY=[%.3f] Estimated Theta=[%.3f] time=%g",pp.Speed(),EstimatedPos.x,EstimatedPos.y,RTOD(estimate_theta),delta_t);
		//if (g_timer_elapsed(timer2, NULL ) >=10.0)
		for(int i=0;i<localizer.hypoth_count;i++)
			{
			if(localizer.hypoths[i].weight>=0.8)
				{
				amcl_location.x= localizer.hypoths[i].mean[0];
				amcl_location.y= localizer.hypoths[i].mean[1];
				theta = localizer.hypoths[i].mean[2];
				if(old_amcl.x !=amcl_location.x || old_amcl.y !=amcl_location.y )
					{
						last_time = g_timer_elapsed(timer2, NULL );// Recording the last time Data changed
						g_timer_start(timer2); // resetting the timer
						old_amcl.x=EstimatedPos.x = amcl_location.x;
						old_amcl.y=EstimatedPos.y = amcl_location.y;
						estimate_theta= theta;
					}
				}
			}
		tracking_point.x= EstimatedPos.x + tracking_distance*cos(estimate_theta) - 0*sin(estimate_theta);
		tracking_point.y= EstimatedPos.y + tracking_distance*sin(estimate_theta) + 0*cos(estimate_theta); 
		distance=sqrt(pow(SegmentEnd.x-tracking_point.x,2)+pow(SegmentEnd.y-tracking_point.y,2));
		displacement = DistToLineSegment(SegmentStart,SegmentEnd,tracking_point);
		if (p->next)
			{
				Point n;
				n.x = p->next->NodeInfo.x;
				n.y = p->next->NodeInfo.y;
				distance_to_next = DistToLineSegment(SegmentEnd,n,tracking_point);
			}
		else // This is the last segment
			{
				distance_to_next = 100;
				if (distance <=0.1)
					segment_navigated = TRUE;
			}
		cout <<"\n	--->>>Current disp=["<<displacement<<"] Next =["<<distance_to_next<<"] ";
		if (displacement > distance_to_next) // we are closer to the next segment
			segment_navigated = TRUE;
		if (estimate_theta < 0 )	
			if (angle <0)
				error_orientation = (DTOR(360) + angle) - (estimate_theta + DTOR(360)); // in Radians
			else 
				error_orientation = angle - (estimate_theta + DTOR(360)); // in Radians
		else
			if (angle <0)
				error_orientation = (DTOR(360) + angle) - estimate_theta; // in Radians
			else
				error_orientation = angle - estimate_theta; // in Radians
		if (Absolute(error_orientation) > DTOR(180) ) error_orientation*=-1;
    	minR = minL = minAhead = 1000;
		// Obstacle Avoidance STUFF
/*		for (int j=0; j<laser.scan_count/5; j++) // Right Section
			{
      				if (laser[j] < minR)       		
						minR = laser[j];	
    			}
		for (int j=2*(laser.scan_count/5); j<3*(laser.scan_count/5); j++) // Ahead Section
			{
      				if (laser[j] < minAhead)       		
						minAhead = laser[j];	
    			}
    		for (int j=4*(laser.scan_count/5); j < laser.scan_count; j++) // Left Section
			{
	        		if (laser[j] < minL)
        					minL = laser[j];
    			}
		if (minR < avoidance_distance_right )
			{
			minR = (avoidance_distance_right - minR)/avoidance_distance_right;// inversely propotional to the distance and normalized
			obstacle_avoidance_force += minR;
			}
		if (minAhead < avoidance_distance_ahead)
			{
			minAhead = (avoidance_distance_ahead - minAhead)/avoidance_distance_ahead;// inversely propotional to the distance and normalized
			obstacle_avoidance_force += minAhead;
			//end_reached = TRUE;
			//WCp.SetSpeed(0.0,0.0); 
			pp.SetSpeed(0.0,0.0);
			continue;
			}
		if (minL < avoidance_distance_left )
			{
			minL = (avoidance_distance_left - minL)/avoidance_distance_left;// inversely propotional to the distance and normalized
			obstacle_avoidance_force -= minL;
			}
		cout<<"\n		--->>> Min Distance Left="<<minL<<" Min Distance Right="<<minR;
		fflush(stdout);*/
		wdem = direction*kd*displacement + kt*error_orientation + ko*obstacle_avoidance_force;
		//wdem = ko*obstacle_avoidance_force;
		printf("\n--->>> First X[%.3f]Y[%.3f] Last=X[%.3f]Y[%.3f] Target Angle =[%.3f] Displ=[%.3f] Distance to end=[%.3f] Orient Error=[%.3f] Wdem=[%.3f]", SegmentStart.x,SegmentStart.y ,SegmentEnd.x,SegmentEnd.y ,RTOD(angle),displacement ,distance, RTOD(error_orientation), wdem);
		fprintf(file,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %g %g\n",EstimatedPos.x,EstimatedPos.y,amcl_location.x, amcl_location.y, displacement ,error_orientation ,wdem,SegmentStart.x,SegmentStart.y,SegmentEnd.x,SegmentEnd.y,g_timer_elapsed(delta_timer, NULL ),last_time);
		//pp.SetSpeed(0.1,wdem); 
		if( Absolute(RTOD(error_orientation)) > 60)
			pp.SetSpeed(0.0,wdem);
		else
			pp.SetSpeed(0.1,wdem); 
		g_timer_start(delta_timer);
	}
	this->AddText("\n\n	--->>>Finished Navigating this section, Moving to NEXT --->>>\n");
	first = last;
	if (!p->next)
		{
		printf("\n--->>> Destination Reached !!!");
		end_reached=TRUE;
		fflush(stdout);
		}
	else
		p = p->next;
	}
	pp.SetSpeed(0,0); // Stop The motors
   	g_timer_destroy( timer2 );
   	g_timer_destroy( delta_timer );
	fclose(file);
}



