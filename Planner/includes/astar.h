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
#include <list>
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
#define FORWARD 1
#define BACKWARD -1
#include "map.h"
#include "callbacks.h"
#include "interface.h"
#include "support.h"
enum {WHEELCHAIR,STAGE};
GTimer *timer2,*delta_timer;
double last_time;
double estimate_x,estimate_y,estimate_theta,velocity,delta_t;
/*
 * Max
 * Return the maximum of two numbers.
 */
#define Max(x, y) ((x) > (y) ? (x) : (y))
/*
 * Min
 * Return the minimum of two numbers.
 */
#define Min(x, y) ((x) < (y) ? (x) : (y))
/*
 * Abs
 * Return the absolute value of the argument.
 */
#define Abs(x) ((x) >= 0 ? (x) : -(x))
/* This function takes two angles in radians
 * and returns the smallest angle between them in radians
 */
double anglediff(double alfa, double beta) 
{
	double diff;
	if( alfa < 0 ) alfa+= 2*M_PI; 	if( alfa > 2*M_PI) alfa-= 2*M_PI;
	if( beta < 0 ) beta+= 2*M_PI;	if( beta > 2*M_PI) beta-= 2*M_PI;		
	diff = alfa - beta;
	if ( diff >  M_PI) diff=( 2*M_PI  - diff);
	if ( diff < -M_PI) diff=(-2*M_PI - diff);
	return Abs(diff);
};
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
		int id, depth,direction;
		double nearest_obstacle,g_value,h_value,f_value,angle;
		Node  * parent, * next, * prev;
		Point   location;
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
int NodeEquality(Node *a, Node *b) // Test for node equality
{
	if (a->location.x == b->location.x && a->location.y == b->location.y)
		return 1;
	return 0;
}
class LList
	{
		public :
			Node * 	Start;
		public :
			     	LList();
			     	~LList();
			void 	Add(Node *);
			bool 	Remove(Node *);
			void 	Print();
			void	Free();
			Node   *Find(Node *);
			void   	Next();
			void    Prev();
			Node *  GetHead();
	};

Node * LList::GetHead()
	{
		return this->Start;
	};
void LList::Next()
	{
		Start = Start->next;
		if(Start != NULL)
			Start->prev = NULL;
	};
void LList::Prev()
	{
		Start = Start->prev;
	};
Node * LList::Find(Node * q)
	{
		Node *p = Start;
		while (p)
		{
			if (NodeEquality(q,p))
				return p;
			p = p->next;				
		}
		return NULL;
	};
LList::LList()
	{
		Start = NULL;
	};
LList::~LList()
	{
		Node *p;
		while (Start != NULL)
		{
			p = Start->next;
			delete Start;
			Start = p;
  		}		
	};
void LList::Free()
	{
		Node *p;
		while (Start != NULL)
		{
			//cout <<"\n	--->>> Freeing Node <<<---"; fflush(stdout);
			p = Start->next;
			delete Start;
			Start = p;
  		}		
	};
void LList::Add(Node * curChild)
	{
		Node * p,* q ;
		p = this->Start;
		q = p;
		// now insert the child into the open list according to the f value
		while (p) 
		{
			// insert before p, sorted ascending
			if (p->f_value >= curChild->f_value) 	       
			{
				// test head of the list case
				if (p == Start)
					Start = curChild;
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
			if (q != NULL) // insert at the end
			{
	      		q->next = curChild;
  				curChild->prev = q;
  				curChild->next = NULL;
			}
			else	      // insert at the beginning
			{
				Start = curChild;
				curChild->prev = NULL;
			}
		}			
		//cout <<"\n	--->>> Node Added Successfully <<<---"; fflush(stdout);
	};
bool LList::Remove(Node *q)
	{
		Node *p;
		p = Start;
		while (p)
		{
			if(NodeEquality(p,q))
				{
					if (p->prev != NULL)	        
			      		(p->prev)->next = p->next;
					if (p->next != NULL)	        
						(p->next)->prev = p->prev;								
					if (p == Start)
						Start = p->next;
					delete p;
					return 1;
				}
			p = p->next;
		}
		return 0;	
	};
void LList::Print()
	{
		Node *p;
		int i=0;
		p = Start;
		while(p)
		{
			cout<<"\n Node["<<++i<<"] X="<<p->location.x<<" Y="<<p->location.y;
			p = p->next;
		}
	};
typedef struct _tree
	{
		Point location;
		vector <Point> children;
	}Tree;
typedef struct _control_action
	{
		double linear_velocity;
		double angular_velocity;
	} ControlAction;
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
		Node *current, *childList, *curChild, *q;
		LList *openList,*closedList;
		//list <Node> openList,closedList;
		MapFile * Map;
		//PathFollower follower;
		vector <Tree> tree;
	public :
		double Calculate_g   (Node *); // Distance from Start
		double Calculate_h   (Node *); // Herustic Distance to Goal
		bool   GoalReached   (Node *); 
		Node  *MakeChildrenNodes(Node *parent) ;
		void   FreeNode      (Node *);
		void   PrintNodeList ();
		int    StartSearch   (Point start, Point end,double initial_angle,double final_angle);
		void   Translate(Point  , double);
		void   Translate_edges(Point  , double);
		int    check_line (Point start,Point end);
		int    Obstacle   (Point p, double angle);
		void   ConvertPixel   (Point *p);
		void   ConvertToPixel (Point *p);
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
		void   draw_tree();
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
	double l = 1.2 , w = 0.7;   // length and width of the wheelchair, can be modefied at any time.
	double startx,starty;       // The edges of the robot in -ve quadrant
	Point center_of_rotation;   // center of rotation in respect to the center of Area
	center_of_rotation.x =-0.3; // it's 30 cm on the x-axis
	center_of_rotation.y = 0;   // it's on the mid of the Wheels Axes so i assume that y = 0;
	startx = -l/2 - center_of_rotation.x; // am determining here the location of the edges in the robot coordinate system
	starty = -w/2 - center_of_rotation.y; // 
	local_edge_points[0].x = startx; 		local_edge_points[0].y = starty;
	local_edge_points[1].x = startx;		local_edge_points[1].y = w + starty;
	local_edge_points[2].x = l + startx;	local_edge_points[2].y = w + starty;
	local_edge_points[3].x = l + startx; 	local_edge_points[3].y = starty;
	// These Points are used for drawing the Robot rectangle
	for (int i=0 ;i < 4; i++)
		cout<<"\nEdge->"<< i<<" X="<<local_edge_points[i].x<<" Y="<<local_edge_points[i].y;
	// Create a Matrix of the points to check for collision detection
	points_per_height = (int)(ceil((l) / (2.0*this->obstacle_radius)));
	points_per_width  = (int)(ceil((w) / (2.0*this->obstacle_radius)));
	this->number_of_point_to_check = points_per_height*points_per_width;
	cout<<"\nPer H ="<<points_per_height<<" Per W="<<points_per_width<<" Total ="<<this->number_of_point_to_check;
	cout<<"\n Obstacle Radius="<<this->obstacle_radius; fflush(stdout);

	// The location of the current edges at each NODE
	this->points_to_check = new Point[this->number_of_point_to_check];
	i =(startx + this->obstacle_radius);
	for(int r =0; r < points_per_height ; r++ )
	{
		j=(starty + this->obstacle_radius);
		for (int s=0;s < points_per_width;s++)
		{
			// Angle zero is when robot heading points to the right (right had rule)
			this->points_to_check[point_index].x = i;
			this->points_to_check[point_index].y = j;
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
	this->openList   = new LList;
	this->closedList = new LList;
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
			gdk_draw_line(view->window,(GdkGC*)(view)->style->white_gc,(int)loc1.x,(int)loc1.y,(int)loc2.x,(int)loc2.y);
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
			if (distance <= allowed_distance && distance !=0)
			{
				angle = atan2(S->location.y - temp->location.y ,S->location.x - temp->location.x);
				if(!Obstacle(temp->location,angle))
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
	this->Map->ReadMap(); 				// we changed the pixel buffer, so reading the image again will regenerate the free space / MapData ;)
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
				for( x = (i - radius) ; x < (i + radius) ; x+=(radius/4.0))//x+=(radius))
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
void AstarPlanner :: ConvertPixel(Point  *p) // transfers from pixel coordinate to the main coordinate system
{
	p->x= p->x*this->pixel_size - this->pixel_size*this->map_width/2;
	p->y=-p->y*this->pixel_size + this->pixel_size*this->map_height/2;
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
		pixel =  p->location;
		cout <<"\nStep [" << step++ <<"] x["<< pixel.x<<"]y["<<pixel.y<<"]"<<" Direction="<<p->direction; 
		cout <<"\tG cost="<<p->g_value<<"\tH cost="<<p->h_value<<"\tFcost="<<p->f_value;
		cout<<"\tStored Angle = "<<RTOD(p->angle);
		if (p->next !=NULL)
		{
			cout<<"\tNext Angle = "<<RTOD(atan2(p->next->location.y - p->location.y, p->next->location.x - p->location.x));
			cout<<"\tAngle Diff ="<<RTOD(anglediff(p->angle,atan2(p->next->location.y - p->location.y, p->next->location.x - p->location.x)));
		}
		for (int i=0;i<this->number_of_point_to_check;i++)
		{
			if(p->wheelchair.check_points.size() == 0)
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
		// Initialize the root node information and put it in the open list
		if (distance < shortest_distance) 
		{
			shortest_distance = distance;
			root->location.x = temp->location.x;
			root->location.y = temp->location.y;
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
	root->angle=this->initial_angle;
	root->direction = FORWARD;
	Translate(root->location,this->initial_angle);
	root->wheelchair.SetCheckPoints(this->number_of_point_to_check,this->points_to_check);
	cout<<"\n---->>>Root is Set to be X="<<root->location.x<<" Y="<<root->location.y;
	fflush(stdout);
	};
int AstarPlanner :: StartSearch(Point start_search, Point end_search,double initial_angle, double final_angle)
{
	int      ID = 1;
  	int      NodesExpanded = 0;
	this->initial_angle = initial_angle;
	this->final_angle   = final_angle;
	if(this->tree.size() > 0)		this->tree.clear();
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
	cout <<"\n	--->>> Search Started <<<---"; fflush(stdout);
	FindRoot();
	cout <<"\n---->>>Target is Set to be X="<<end.x<<" Y="<<end.y<<" <<<---"; fflush(stdout);
  	openList->Add(root);				// Add the root to OpenList
	cout <<"\n	--->>> Root Added <<<---"; fflush(stdout);
//  while openList is not empty 
	while (openList->Start != NULL) 
	{
		current = openList->GetHead(); 	// Get the node with the cheapest cost, first node
		openList->Next();				// Move to the next Node
    	NodesExpanded++;
    	if (GoalReached(current))                     // We reached the target location, so build the path and return it.
		{
			// build the complete path to return
      		current->next = NULL;
      		path = current;
      		p = current->parent;
    		cout<<"\n	--->>> Goal state reached with :"<<ID<<" nodes created and :"<<NodesExpanded<<" nodes expanded <<<---\n";
			fflush(stdout);
			cout<<"\n	--->>> General Clean UP <<<---";
			fflush(stdout);
//			int m=0;
      		p = current;
//   			while (p != NULL) 
//			{
//				//cout<<"\n	--->>> Step["<<++m<<"] X="<<p->location.x<<" Y="<<p->location.y;
//				//cout<<"\n	--->>> Angle is="<<RTOD(p->angle);
//				fflush(stdout);
//				p = p->parent;
//			}
      		// Going up to the Root
      		p = current->parent;
   			while (p != NULL)
			{
//				cout<<"\n Am Still HERE Step["<<++m<<"] X="<<p->location.x<<" Y="<<p->location.y;
//				fflush(stdout);
				// remove the parent node from the closed list (where it has to be)
				if(p->prev != NULL)
					(p->prev)->next = p->next;
				if(p->next != NULL)
					(p->next)->prev = p->prev;
				// check if we're removing the top of the list
				if(p == closedList->Start)
		  			closedList->Next();
				// set it up in the path
				p->next = path;				
				path = p;
				p = p->parent;
			}
        	// now delete all nodes on OPEN and Closed Lists
			cout<<"\n	--->>> Freeing open list <<<---";   fflush(stdout);
			openList->Free();
			cout<<"\n	--->>> DONE  <<<---";	fflush(stdout);
			cout<<"\n	--->>> Freeing closed list <<<---";	fflush(stdout);
			closedList->Free();
			cout<<"\n	--->>> DONE  <<<---";	fflush(stdout);
      		return 1; 	// Path Found Successfully
    	}
    	
    	// Create List of Children for the current NODE
		if(!(childList = MakeChildrenNodes(current))) // No more Children => Search Ended Unsuccessfully at this path Branch
		{
			cout<<"\n	--->>> Search Ended On this Branch / We Reached a DEAD END <<<---";
			fflush(stdout);
		}
		// insert the children into the OPEN list according to their f values
    	while (childList != NULL)                     
		{
  		    curChild  = childList;
  			childList = childList->next;
		    // set up the rest of the child node details
  			curChild->parent = current;
  			curChild->depth = current->depth + 1;
  			curChild->id = ID++;
  			curChild->next = NULL;
  			curChild->prev = NULL;
  			curChild->g_value = Calculate_g(curChild);
      		curChild->h_value = Calculate_h(curChild);
  			curChild->f_value = curChild->g_value + curChild->h_value;
			Node * p;
			// check if the child is already in the open list
			if( (p = openList->Find(curChild)))
			{
  				if (p->f_value <= curChild->f_value && (p->direction == curChild->direction))       
				{
	        		FreeNode(curChild);
  					curChild = NULL;
				}
				// the child is a shorter path to this point, delete p from  the closed list
				else 
  				if (p->f_value > curChild->f_value && (p->direction == curChild->direction))
				{
					openList->Remove(p);
				 	//cout<<"\n	--->>> Opened list -- Node is deleted, current child X="<<curChild->location.x<<" Y="<<curChild->location.y<<" has shorter path<<<---";
					fflush(stdout);

				}					
			}
			// test whether the child is in the closed list (already been there)			
			if (curChild)
			if((p = closedList->Find(curChild)))
			{
  				if (p->f_value <= curChild->f_value && p->direction == curChild->direction)       
				{
	        		FreeNode(curChild);
  					curChild = NULL;
				}
				// the child is a shorter path to this point, delete p from  the closed list
				else 
				{
					/* This is the tricky part, it rarely happens, but in my case it happenes all the time :s
					 * Anyways, we are here cause we found a better path to a node that we already visited, we will have to
					 * Update the cost of that node and ALL ITS DESCENDENTS because their cost is parent dependent ;)
					 * Another Solution is simply to comment everything and do nothing, doing this, the child will be added to the
					 * Open List and it will be investigated further later on.
					 */
					//closedList->Remove(p);
				 	//cout<<"\n	--->>> Closed list -- Node is deleted, current child X="<<curChild->location.x<<" Y="<<curChild->location.y<<" has shorter path<<<---";
					fflush(stdout);

				}					
			}
			// ADD the child to the OPEN List
			if (curChild)
			{
				openList->Add(curChild);
			}	 		
    	}
		// put the current node onto the closed list, ==>> already visited List
		closedList->Add(current);
		// Test to see if we have expanded too many nodes without a solution
    	if (current->id > this->MAXNODES*2)     	
		{
	    	cout<<"\n	--->>>	Expanded more than the maximum allowable nodes of MAXNODE="<<this->MAXNODES<<" Terminating ...";
			fflush(stdout);
			//Delete Nodes in Open and Closed Lists
			closedList->Free();
			openList->Free();
			return 0; // Expanded more than the maximium nodes state
    	}
 	}					       // end of OPEN loop
  	// if we got here, then there is no path to the goal
  	// delete all nodes on CLOSED since OPEN is now empty
	closedList->Free();
  	cout<<"\n	--->>>No Path Found<<<---";
  	return -1; // No Path Found
};
double AstarPlanner:: Calculate_g(Node *n) // define the g function as parent plus 1 step
{
	double cost;
	if(n == NULL || n->parent==NULL)
		return 0.0;
	cost = n->parent->g_value + sqrt(pow(n->location.x - n->parent->location.x,2) + pow(n->location.y - n->parent->location.y,2) );
	return cost;
};
double AstarPlanner::Calculate_h(Node *n) //define the h function as the Euclidean distance to the goal + turning penalty
{
	double h=0,angle_cost=0,obstacle_penalty=0,reverse_penalty=0,delta_d=0;
	if(n == NULL)
		return(0);
	// Using the Euclidean distance
	h = sqrt(pow(this->end.x - n->location.x,2) + pow(this->end.y - n->location.y,2));
	//h = 0;
	if (n->parent != NULL) // Adding the Angle cost, we have to uniform the angle representation to the +ve rep or we well get a non sense result
	{
		angle_cost = anglediff(n->angle,n->parent->angle); // in radians
		delta_d = sqrt(pow(n->location.x - n->parent->location.x,2) + pow(n->location.y - n->parent->location.y,2) );
	}
	obstacle_penalty = n->nearest_obstacle;
	if(n->direction == BACKWARD) 
		reverse_penalty = 3;
	else
		reverse_penalty = 0;
	// this has a maximium value of 1 because it's normalized 
	// Now the trick is to wisely distribute the weights of the costs
	// My priority is to have a path that is smooth and goes in the middle of the free space
	// That's why i will make the obstacle cost propotional to the angle cost
	// 0.555 is the AXLE Length , we don't care which direction we are turning, it's a turn anyways
	return ( h + 0.555 * angle_cost + obstacle_penalty*delta_d + reverse_penalty );
};
// define the goalNode function

bool AstarPlanner :: GoalReached (Node *n) 
{
	double angle_diff, delta_d;
	delta_d = sqrt(pow(n->location.x - this->end.x,2)+pow(n->location.y - this->end.y,2));
	if (n->direction == FORWARD)
		angle_diff =	anglediff(this->final_angle,n->angle);
	else
	{
		angle_diff =	anglediff(this->final_angle,n->angle + M_PI);
	}
	if ( delta_d <= 0.3  && angle_diff <= DTOR(30))
	{
		cout<<" \n Desired Final Orientation ="<<RTOD(this->final_angle)<<" Current="<<RTOD(n->angle);
		cout<<"\n Reached Destination with Diff Orientation="<< RTOD(angle_diff);
		return 1;
	}
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
		//cout<<" \nAfter Conversion X="<<this->translated_edge_points[i].x<<" Y="<<this->translated_edge_points[i].y;
		}
	};
// Test for whether a point is in an obstacle or not
int AstarPlanner :: Obstacle(Point P, double angle) 
{
	Point s;
	int m,n;
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
void AstarPlanner::draw_tree()
{
	GtkWidget * temp;
	temp = lookup_widget (GTK_WIDGET(widget),"drawingarea1");
	Point l_start,l_end;
	for (unsigned int i =0; i<tree.size();i++)
		{
			l_start = tree[i].location;
			ConvertToPixel(&l_start);
			//cout<<"\n Main X="<<tree[i].location.x<<" Y="<<tree[i].location.y;
			for(unsigned int j=0;j<tree[i].children.size();j++)
			{
				l_end = tree[i].children[j];
				ConvertToPixel(&l_end);
				gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)l_start.x,(int)l_start.y,
				(int)l_end.x,(int)l_end.y);
				//cout<<"\n	--->>>Child X="<<tree[i].children[j].x<<" Y="<<tree[i].children[j].y;
			}
		}
}
void AstarPlanner::draw_path()
{
	if (!this->path)
	{
		cout<<"\n->NO Path Generated Yet, plan a Path First";
		return;
	}
	double angle;
	GtkWidget * temp;
	temp = lookup_widget (GTK_WIDGET(widget),"drawingarea1");
	Point l_start,l_end,E,S;
  	Node *p;
  	p = this->path;
	while(p != NULL && p->next!=NULL)
	{
		S =  p->location;
  		temp = lookup_widget (widget,"drawingarea1");
		if (p->direction == FORWARD)
			angle = p->angle;
		else
			angle = p->angle + M_PI;
		Translate_edges(S,angle);
		for (int i=0 ;i < 4; i++)
		{
			this->ConvertToPixel(&translated_edge_points[i]);
			//cout<<"\nEdge "<< i+1<<" X="<<translated_edge_points[i].x<<" Y="<<translated_edge_points[i].y;
		}
				//cout<<"\n";
// 		gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)translated_edge_points[0].x,(int)translated_edge_points[0].y,
//		(int)translated_edge_points[1].x,(int)translated_edge_points[1].y);
//  		gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)translated_edge_points[1].x,(int)translated_edge_points[1].y,
//		(int)translated_edge_points[2].x,(int)translated_edge_points[2].y);
//  		gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)translated_edge_points[2].x,(int)translated_edge_points[2].y,
//		(int)translated_edge_points[3].x,(int)translated_edge_points[3].y);
//  		gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)translated_edge_points[3].x,(int)translated_edge_points[3].y,
//		(int)translated_edge_points[0].x,(int)translated_edge_points[0].y);
		if (p->next)
		{
			l_start = p->location; l_end = p->next->location;
			ConvertToPixel(&l_start); ConvertToPixel(&l_end);
  			gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)l_start.x,(int)l_start.y,(int)l_end.x,(int)l_end.y);
		}
		p = p->next;
	} 
};
Node *AstarPlanner :: MakeChildrenNodes(Node *parent) 
{
	Point P; // grand parents  and parent node information
	Node  *p, *q;
	double angle,angle_difference,discrete_angle,parent_angle,child_angle,angle_resolution = DTOR(20);
	bool collides = FALSE;
	int direction;
	P.x = parent->location.x;
	P.y = parent->location.y;
	Tree t;
	t.location = P;
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
		angle_difference = anglediff(angle,parent->angle);	
		if (angle_difference > DTOR(90)) // Change direction of Motion
		{
			//cout<<"\n Angle difference ="<<RTOD(angle_difference)<<" parent angle="<<RTOD(parent->angle)<<" destination angle="<<RTOD(angle);
			direction = parent->direction * -1;
			//angle += M_PI;
			//if (angle > 2*M_PI ) angle-=2*M_PI;
		}
		else
		{
			direction = parent->direction;
		}
		collides =FALSE;
		parent_angle = parent->angle; if(parent_angle < 0 ) parent_angle += 2*M_PI;
		(direction == FORWARD)?(child_angle = angle):(child_angle = angle + M_PI);
		if(child_angle  < 0 ) child_angle  += 2*M_PI;
		discrete_angle =  parent_angle;
		double start_angle,end_angle;
		start_angle  = (parent_angle > child_angle)?parent_angle:child_angle;
		end_angle	 = (parent_angle < child_angle)?parent_angle:child_angle;
    	discrete_angle =  start_angle;
		//cout<<"\n Start is"<<RTOD(start_angle)<<" End angle="<<RTOD(end_angle);
		for (int s=0 ; s <= ceil(angle_difference/angle_resolution); s++)
		{
			if(Abs(start_angle - end_angle) > DTOR(180))
			{
				discrete_angle += angle_resolution;
				if ( ((discrete_angle>360)?discrete_angle-=360:discrete_angle) > end_angle)
					discrete_angle = end_angle;							
			}
			else
			{
				discrete_angle -= angle_resolution;
				if (discrete_angle < end_angle)
					discrete_angle = end_angle;
			}
			if (Obstacle(temp->children[i]->location,discrete_angle))
				{
				collides= TRUE;
				break;
				}
		}
		if (!collides) // if after discretization the child still doens't collide then add it
		{
			p = new Node;
			p->location.x = temp->children[i]->location.x;
			p->location.y = temp->children[i]->location.y;
			p->direction  =	direction ;
			t.children.push_back(p->location);
			p->nearest_obstacle = temp->children[i]->obstacle_cost;
			p->parent = parent;
			p->angle= angle;
			p->wheelchair.SetCheckPoints(this->number_of_point_to_check,this->points_to_check);
			p->next = q;
			q = p;
	    	//cout<<"\n NEW CHILD ADDED";
		}
	}
	// Save the search tree so that it can be displayed later
	if (t.children.size() > 0)
		tree.push_back(t);
	return q;
}
// Free node function
void AstarPlanner :: FreeNode(Node *n) 
{
	delete n;
}


double Magnitude( Point p1, Point p2 )
{
    Point Vector;
    Vector.x = p2.x - p1.x;
    Vector.y = p2.y - p1.y;
    return sqrt( Vector.x * Vector.x + Vector.y * Vector.y);
}
double DistanceToLine( Point LineStart, Point LineEnd, Point P)
{
	double LineMag,distance;
	LineMag = Magnitude(LineStart,LineEnd);
	distance = (P.x*(LineStart.y - LineEnd.y) + P.y*(LineEnd.x - LineStart.x)+(LineStart.x*LineEnd.y - LineEnd.x*LineStart.y))/LineMag ;
    	return distance;
};
double DistToLineSegment(Point LineStart, Point LineEnd, Point p)
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
};
class PathFollower
{
	public :
		Node 	*path,*last,*first;
		GdkPixbuf *pixbuf;
		AstarPlanner * Planner;
		double 	kd,kt,ko,tracking_distance,angle,prev_angle,theta,error_orientation,
				displacement,wdem,distance,distance_to_next,obstacle_avoidance_force,
				pose[3], pose_var[3][3],minR,minL,minAhead,avoidance_distance_left,
				avoidance_distance_right,avoidance_distance_ahead,speed;
		bool	log,position_found,end_reached,segment_navigated;
		int		platform,direction;
		Point 	old_amcl,begin,amcl_location,tracking_point,ni,SegmentStart,SegmentEnd,
				EstimatedPos;
		GtkWidget 		*widget;
		PlayerClient 	*robot;
		WheelChairProxy *WCp;
		PositionProxy 	*pp;
		LaserProxy 		*laser;
		LocalizeProxy 	*localizer;
		FILE 			* file;
	public:
		bool stop;
		PathFollower(); // Start with no initialization
		PathFollower(Node * p,double kd, double kt,double ko,double tracking_distance,GtkWidget *widget,bool log,AstarPlanner *Planner,GdkPixbuf *pixbuf);
		~PathFollower(); 
		void Connect(int);
		void RenderGui(Point,double);
		void FollowPath(Node *path);
		void AddText( char const * text);
		void ResetWheelchair();
		void Localize();
		ControlAction Controller(double angle_ref,double angle_currnet,double displacement,int direction);
		
};
PathFollower::PathFollower()
{
	// Do nothing, Empty Constructor
}
PathFollower::PathFollower(Node * p,double kd, double kt,double ko,double tracking_distance,GtkWidget * widget,bool log, AstarPlanner *Planner,GdkPixbuf *pixbuf )
{
	this->path = p;
	this->kd = kd;
	this->kt = kt;
	this->ko = ko;
	this->tracking_distance = tracking_distance;
	this->widget = widget;
	this->speed = 0.1;
	this->Planner = Planner;
	this->pixbuf = pixbuf;
	robot = NULL; WCp = NULL; pp = NULL;  laser = NULL; localizer = NULL;
	avoidance_distance_left  = 0.2 + 0.5;
	avoidance_distance_right = 0.3;
	avoidance_distance_ahead = 0.2;
	stop = FALSE;
	this->log = log;
	if (log) 
		file = fopen("pathfollowing.txt","wb");
}
PathFollower::~PathFollower()
{
	if(platform == WHEELCHAIR)
		WCp->SetSpeed(0,0); // Stop The motors
	else
		pp->SetSpeed(0,0);
	if (robot)
		delete robot;
	if (WCp)
		delete WCp;
	if (pp)
		delete pp;
	if (laser)
		delete laser;
	if (localizer)
		delete localizer;
   	g_timer_destroy( timer2 );
   	g_timer_destroy( delta_timer );
	if(log)	
		fclose(file);
};
void PathFollower::AddText ( char const * text)
	{
	GtkWidget * view;
	GtkTextBuffer *text_buffer;
	GtkTextIter     start, end;
	GtkTextMark* mark;
	view = lookup_widget (GTK_WIDGET(widget),"textview1");
  	text_buffer = gtk_text_view_get_buffer (GTK_TEXT_VIEW (view));
	gtk_text_buffer_insert_at_cursor(text_buffer,text,-1);
	// Scrolling to end
	text_buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(view));
    gtk_text_buffer_get_bounds (text_buffer, &start, &end);
   	mark = gtk_text_buffer_create_mark    (text_buffer, NULL, &end, 1);
   	gtk_text_view_scroll_to_mark(GTK_TEXT_VIEW(view), mark, 0.0, 0, 0.0, 1.0);
   	gtk_text_buffer_delete_mark (text_buffer, mark);
	}
void PathFollower::ResetWheelchair()
{
	if(platform == WHEELCHAIR && WCp)
	{
		/************** Wheelchair ON Automatic Mode **********************/
		WCp->SetPower(OFF);
		usleep(200000);
		WCp->SetPower(ON);
		usleep(200000);
		WCp->SetMode(AUTO);
		usleep(200000);
		AddText("\n	--->>> WheelChair is ON and CONTROL Mode is AUTO <<<---\n");
		fflush(stdout);
		/*****************************************************************/
	}	
}
void PathFollower::Connect(int platform)
{
	this->platform = platform;
	if(platform == WHEELCHAIR)
	{
		robot = new PlayerClient("192.168.0.101", 6665);
		WCp   = new WheelChairProxy(robot,0,'a');
		if(WCp->GetAccess() == 'e')
		{
			AddText("\n	--->>> Error getting wheelchair device atracking_distanceccess! <<<---");
			return;
		}		
	}
	else 
		robot = new PlayerClient("127.0.0.1", 6665);
	pp  		= new PositionProxy(robot,0,'a');
	laser 		= new LaserProxy(robot,0,'r');
	localizer 	= new LocalizeProxy(robot,0,'r');
 	if(localizer->access == 'e')
    {
  		AddText( "\n	--->>> Can't read from localizer <<<---" );
  		return;
   	} 
 	if(laser->access != 'r')
    {
      	AddText( "	--->>> Can't read from laser" );
      	return;
    }  
  	if(pp->GetAccess() == 'e') 
	{
    	AddText("\n	--->>> Error getting position device access! <<<---");
    	return;
  	}
	AddText(robot->conn.banner);
	ResetWheelchair();
}
void PathFollower::Localize()
{
	if(!path)
	{
		AddText("\n --->>> NO PATH to Localize<<<--- ");
		return;
	}
	pose[0]= path->location.x;
	pose[1]= path->location.y;
	pose[2]= path->angle;
	cout << "\n Default Pose given to the Localizer X="<<path->location.x<<" Y="<<path->location.y<<" Theta="<<path->angle;
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
	localizer->SetPose(pose,pose_var);
	/***********************Finding Out where we are from the Localizer with initial estimation ************************/
	while(!position_found) //wait until we have 90% accurate assumption / hypothesis
	{
		if(robot->Read()) 
			exit(1);
		printf("%d hypotheses\n", localizer->hypoth_count); // Assumed number of Hypothesis
    	printf("%d (weight %f): [ %f %f %f ]\n",0,localizer->hypoths[0].weight,localizer->hypoths[0].mean[0],localizer->hypoths[0].mean[1], localizer->hypoths[0].mean[2]);
		if(localizer->hypoths[0].weight>=0.9) // Since the hypothesis are sorted, we assume the first one to be the one with the most weight
		{
			position_found=1; // Accurate Hypothesis found, so update current location and move next
			old_amcl.x = EstimatedPos.x=amcl_location.x= localizer->hypoths[0].mean[0];
			old_amcl.y = EstimatedPos.y=amcl_location.y= localizer->hypoths[0].mean[1];
			estimate_theta=localizer->hypoths[0].mean[2];
			localizer->SetPose(pose,pose_var);
		}
	usleep(100000);
	}
	usleep(10000000);
}
void PathFollower::RenderGui(Point amcl,double angle)
{
	GtkWidget * temp;
	Point p=amcl;
	Planner->ConvertToPixel(&p);
	temp = lookup_widget (GTK_WIDGET(widget),"drawingarea1");
	p.x = p.x - 25;
	p.y = p.y - 25;
	if (p.x < 0) p.x = 0;
	if (p.y < 0) p.y = 0;
	if (p.x > Planner->map_width)  p.x = Planner->map_width;
	if (p.y > Planner->map_height) p.y = Planner->map_height;
  	gdk_draw_pixbuf(temp->window, NULL, pixbuf,
                    (int)p.x, (int)p.y,
                    (int)p.x, (int)p.y,
                    50, 50,
                    GDK_RGB_DITHER_NORMAL, 0, 0);
    Planner->draw_path();
	Planner->Translate_edges(amcl,angle);
	
	for (int i=0 ;i < 4; i++)
		Planner->ConvertToPixel(&Planner->translated_edge_points[i]);

	gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)Planner->translated_edge_points[0].x,(int)Planner->translated_edge_points[0].y,
	(int)Planner->translated_edge_points[1].x,(int)Planner->translated_edge_points[1].y);
	gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)Planner->translated_edge_points[1].x,(int)Planner->translated_edge_points[1].y,
	(int)Planner->translated_edge_points[2].x,(int)Planner->translated_edge_points[2].y);
	gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)Planner->translated_edge_points[2].x,(int)Planner->translated_edge_points[2].y,
	(int)Planner->translated_edge_points[3].x,(int)Planner->translated_edge_points[3].y);
	gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)Planner->translated_edge_points[3].x,(int)Planner->translated_edge_points[3].y,
	(int)Planner->translated_edge_points[0].x,(int)Planner->translated_edge_points[0].y);
	while (gtk_events_pending())
    	gtk_main_iteration();
}
ControlAction PathFollower::Controller(double angle_current,double angle_ref,double displacement,int direction)
{
	ControlAction cntrl;
	if(direction == -1)	  angle_current += M_PI;
	if(angle_ref     < 0) angle_ref    += 2*M_PI;
	if(angle_current < 0) angle_current+= 2*M_PI;
	double orientation_error = angle_current - angle_ref;
	if ( orientation_error >  M_PI) orientation_error= (-2*M_PI + orientation_error);
	if ( orientation_error < -M_PI) orientation_error= ( 2*M_PI + orientation_error);
	cntrl.angular_velocity = (-kd*speed*displacement - kt*speed*orientation_error);
	if (Abs(orientation_error)>DTOR(10))
	{
		cntrl.linear_velocity = 0;
		//cntrl.angular_velocity *=2;
	}
	else
		cntrl.linear_velocity = speed;
	AddText(g_strdup_printf("\n->Ori-Err=[%.3f] Dist-Err=[%.3f] Wdem=[%.3f]",RTOD(orientation_error),displacement,cntrl.angular_velocity));
	return cntrl;
};
void PathFollower::FollowPath(Node *path)
{
	ControlAction cntrl;
	if(!path)
	{
		AddText("\n --->>> No PATH TO FOLLOW <<<---");
		return ;
	}
	if(robot && pp && WCp && laser && localizer)
		Localize();
	else
	{
		AddText("\n No connected to Server , Reconnect and try again");
	}
	/********************** Start from the root and move forward ************************/
	first 		= path;
	prev_angle	= path->angle;
	path 		= path->next;
	/************************************************************************************/
	timer2 		= g_timer_new();
	delta_timer = g_timer_new();
	//Reset Times
	last_time=0; delta_t=0;	velocity=0;
	while(!end_reached) 
	{
		g_timer_start(timer2);
		g_timer_start(delta_timer);
		if(robot->Read()) 
			exit(1);
		last = path;
		ni = first->location; SegmentStart.x = ni.x; SegmentStart.y = ni.y;
		AddText(g_strdup_printf("\n	--->>>NEW Line SEGMENT Starts at x[%.3f]y[%.3f] <<<---",SegmentStart.x,SegmentStart.y));
		fflush(stdout);
		ni = last->location;  SegmentEnd.x = ni.x;  SegmentEnd.y = ni.y;	
		AddText(g_strdup_printf("\n	--->>>NEW Line SEGMENT Ends at   x[%.3f]y[%.3f] <<<---",SegmentEnd.x,SegmentEnd.y));
		direction = -1;
		angle = atan2(SegmentEnd.y - SegmentStart.y,SegmentEnd.x - SegmentStart.x);
		AddText(g_strdup_printf("\n	--->>> Angle is:=%.3f <<<---",RTOD(angle)));
		segment_navigated = FALSE;
		while (!segment_navigated && !stop) // Loop for each path segment
		{
			RenderGui(EstimatedPos,estimate_theta);
			//if(robot.Peek(0))
			if(robot->Read()) 
					exit(1);
			if (velocity!= pp->Speed())
					velocity = pp->Speed();
			obstacle_avoidance_force = 0;
			delta_t=g_timer_elapsed(delta_timer, NULL );
			estimate_theta += pp->SideSpeed()*delta_t;
			EstimatedPos.x += velocity*cos(estimate_theta)*delta_t;
			EstimatedPos.y += velocity*sin(estimate_theta)*delta_t;
			AddText(g_strdup_printf("\n->Vel =%.3f m/sev X=[%.3f] Y=[%.3f] Theta=[%.3f] time=%g",pp->Speed(),EstimatedPos.x,EstimatedPos.y,RTOD(estimate_theta),delta_t));
			for(int i=0;i<localizer->hypoth_count;i++)
			{
				if(localizer->hypoths[i].weight>=0.8)
				{
					amcl_location.x= localizer->hypoths[i].mean[0];
					amcl_location.y= localizer->hypoths[i].mean[1];
					theta = localizer->hypoths[i].mean[2];
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
			distance = sqrt(pow(SegmentEnd.x-tracking_point.x,2)+pow(SegmentEnd.y-tracking_point.y,2));
			displacement = DistToLineSegment(SegmentStart,SegmentEnd,tracking_point);
			if (path->next)
			{
				Point n;
				n.x = path->next->location.x;
				n.y = path->next->location.y;
				distance_to_next = DistToLineSegment(SegmentEnd,n,tracking_point);
			}
			else // This is the last segment
			{
				distance_to_next = 100;
				if (distance <= 0.3)
					segment_navigated = TRUE;
			}
			if (displacement > distance_to_next) // we are closer to the next segment
				segment_navigated = TRUE;
			AddText(g_strdup_printf("\n->First X[%.3f]Y[%.3f] Last=X[%.3f]Y[%.3f] Target Angle =[%.3f] Cur_Ang =[%.3f]", SegmentStart.x,SegmentStart.y ,SegmentEnd.x,SegmentEnd.y ,RTOD(angle),RTOD(estimate_theta)));
			AddText(g_strdup_printf("\n->Displ=[%.3f] Dist to Segend=[%.3f] D-Next=[%.3f]",displacement ,distance,distance_to_next));
			cntrl = Controller(estimate_theta,angle,displacement,path->direction);
			if(log)
				fprintf(file,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %g %g\n",EstimatedPos.x,EstimatedPos.y,amcl_location.x, amcl_location.y, displacement ,error_orientation ,cntrl.angular_velocity,SegmentStart.x,SegmentStart.y,SegmentEnd.x,SegmentEnd.y,g_timer_elapsed(delta_timer, NULL ),last_time);
			if(platform == WHEELCHAIR)
				WCp->SetSpeed(path->direction*cntrl.linear_velocity,cntrl.angular_velocity);
			else
				pp->SetSpeed(path->direction*cntrl.linear_velocity,cntrl.angular_velocity);
			g_timer_start(delta_timer);
		}
		AddText("\n--->>>Finished Navigating this section, Moving to NEXT --->>>\n");
		first = last;
		if (!path->next)
		{
			AddText("\n--->>> Destination Reached !!!");
			end_reached=TRUE;
			fflush(stdout);
		}
		else
			path = path->next;
	}	
	if(platform == WHEELCHAIR)
		WCp->SetSpeed(0,0); // Stop The motors
	else
		pp->SetSpeed(0,0);
}

