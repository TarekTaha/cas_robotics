#include "PathPlanner.h"
#include<Node.h>
#include<Point.h>
namespace CasPlanner
{
// Test for node equality
int NodeEquality(Node *a, Node *b) 
{
	if (a->location.x == b->location.x && a->location.y == b->location.y)
		return 1;
	return 0;
}
PathPlanner :: PathPlanner()
	{
	};
void PathPlanner::FreePath()
{
	while(path != NULL) 
	{
		p = path->next;
		delete path;
		path = p;
	}
};
// This Determines the locations of the points to be checked in the Vehicle Coordinates, should be rotated at each node
void PathPlanner::DetermineCheckPoints() 
{
	//Point edges[4]={{32.5,93},{-32.5,93},{-32.5,-22},{32.5,-22}}; this is the exact dimentions
	int point_index=0,points_per_height,points_per_width;
	double i,j;
	double l = 1.2 , w = 0.65;   // length and width of the wheelchair, can be modefied at any time.
	double startx,starty;       // The edges of the robot in -ve quadrant
	Point center_of_rotation;   // center of rotation in respect to the center of Area
	center_of_rotation.x =-0.3; // it's 30 cm on the x-axis
	center_of_rotation.y = 0;   // it's on the mid of the Wheels Axes so i assume that y = 0;
	startx = -l/2 - center_of_rotation.x; // am determining here the location of the edges in the robot coordinate system
	starty = -w/2 - center_of_rotation.y; // 
	// These Points are used for drawing the Robot rectangle
	local_edge_points[0].x = startx; 		local_edge_points[0].y = starty;
	local_edge_points[1].x = startx;		local_edge_points[1].y = w + starty;
	local_edge_points[2].x = l + startx;	local_edge_points[2].y = w + starty;
	local_edge_points[3].x = l + startx; 	local_edge_points[3].y = starty;
	for (int i=0 ;i < 4; i++)
		cout<<"\nEdge->"<< i<<" X="<<local_edge_points[i].x<<" Y="<<local_edge_points[i].y;
	// Create a Matrix of the points to check for collision detection
	points_per_height = (int)(ceil(l/(double)(2*this->obstacle_radius)));
	points_per_width  = (int)(ceil(w/(double)(2*this->obstacle_radius)));
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
			/* Determining the next center it should be 2*r away from the previous
			 * and it's circle should not exceed the boundaries
			 */
			if ( (j+2*this->obstacle_radius + this->obstacle_radius) >= (w + starty) ) 
				j = (w + starty - this->obstacle_radius);// Allow overlap
			else 
				j += (2*this->obstacle_radius);
		}
		// Same as Above
		if ((i+2*this->obstacle_radius + this->obstacle_radius) >= (l + startx)) 
		{
			// Alow overlap in this case, this is the last center
			i = (l + startx - this->obstacle_radius); 
		}
		else 
			i += (2*this->obstacle_radius);
	}
	this->wheelchair = new Robot();
	this->wheelchair->SetCheckPoints(this->number_of_point_to_check,this->points_to_check);
	for (int k=0;k<this->number_of_point_to_check;k++)
	{
		cout << "\nPoint to check "<<k<<"'---> X="<<this->wheelchair->check_points[k].x<<" Y="<<this->wheelchair->check_points[k].y;
		fflush(stdout);
	}
};
PathPlanner :: PathPlanner(Point start, Point point,double initial_angle,double final_an,double pixel_size,double radius,GtkWidget *w,const char * MapFilename)
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
PathPlanner :: ~PathPlanner()
	{
	delete wheelchair;
	delete [] points_to_check;
	delete map;
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
void PathPlanner::AddText ( char const * text)
	{
	GtkWidget * view;
	GtkTextBuffer *text_buffer;
	GtkTextIter     startv, endv;
	//view = lookup_widget (GTK_WIDGET(widget),"textview1");
  	text_buffer = gtk_text_view_get_buffer (GTK_TEXT_VIEW (view));
	gtk_text_buffer_insert_at_cursor(text_buffer,text,-1);
    gtk_text_buffer_get_bounds(text_buffer, &startv, &endv);
    gtk_text_view_scroll_to_iter(GTK_TEXT_VIEW(view), &endv, 0.0, FALSE, 0.0,0.0);
	}
void PathPlanner::ShowConnections()
	{
	Point loc1,loc2;
	GtkWidget * view;
	//view = lookup_widget (GTK_WIDGET(widget),"drawingarea1");
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
void PathPlanner::ConnectNodes(double allowed_distance)
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
void PathPlanner::AddCostToNodes(double r)
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
				for( i= (int)(point.x - radius) ; i < (int)(point.x + radius); i+=1)
					{
						if (i < 0) i = 0;
						if (i >= this->map_width)  break;
						j = (int)(- sqrt(radius*radius - (i - point.x)*(i - point.x)) + point.y);
						if (j < 0) j = 0;
						if (j >= this->map_height) j = this->map_height - 1;
						if(this->map->mapdata[i][j])
								 nearest_obstacle = radius;
						j = (int)(+ sqrt(radius*radius - (i - point.x)*(i - point.x)) + point.y);
						if (j < 0) j = 0;
						if (j >= this->map_height) j = this->map_height - 1;
	
						if(this->map->mapdata[i][j])
								 nearest_obstacle = radius;
					}
			//cout<<"\n R="<<radius;
			//fflush(stdout);
			}
		// this is a normalized cost, it will make more sense later on 
		S->obstacle_cost =  (r - nearest_obstacle*this->pixel_size)/r; 
		//cout<<"\n Obstacle Cost ="<<S->obstacle_cost;
		//getchar();
		S = S->next;
	}
	this->AddText(g_strdup_printf("\n	--->>> NODES CONNECTED <<<---	"));
	cout<<"\n	--->>> NODES CONNECTED <<<---	";
	};
bool PathPlanner::CheckShortestDistance(double x,double y,double neigbhour_distance)
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
void PathPlanner::ExpandObstacles() // Nifty way of doing an obstacle expansion, not finalized, ----->>>>> @ TO DO @ <<<<<------
	{
	if(this->map->mapdata == NULL )
	{
		this->AddText(g_strdup_printf("\n	--->>> You have to read the map before Expanding the Obstacles <<<---	"));
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
				if (this->map->mapdata[i][j] && !this->map->mapdata[x][y])
				{
				for (int r = 1; r <(int)(thickness);r++)
					for( m = (int)(i - r); m <= (int)(i + r);m++)
					{
						if (m < 0) m = 0;
						if (m >= this->map_width)  break;
						n = (int)(- sqrt(r*r - (m - i)*(m - i)) + j);
						if (n < 0) n = 0;
						if (n >= this->map_height) n = this->map_height - 1;
						this->map->DrawPixel(0xff,0xff,0xff,m,n);
						n = (int)(+ sqrt(r*r - (m - i)*(m - i)) + j);
						if (n < 0) n = 0;
						if (n >= this->map_height) n = this->map_height - 1;
						this->map->DrawPixel(0xff,0xff,0xff,m,n);
					}
				break;
				}
				y = (int)(+ sqrt(radius*radius - (x - i)*(x - i)) + j);
				if (y < 0) y = 0;
				if (y >= this->map_height) y = this->map_height - 1;
				if (this->map->mapdata[i][j] && !this->map->mapdata[x][y])
				{
				for (int r = 1; r <(int)(thickness);r++)
					for( m = (int)(i - r); m <= (int)(i + r);m++)
					{
						if (m < 0) m = 0;
						if (m >= this->map_width)  break;
						n = (int)(- sqrt(r*r - (m - i)*(m - i)) + j);
						if (n < 0) n = 0;
						if (n >= this->map_height) n = this->map_height - 1;
						this->map->DrawPixel(0xff,0xff,0xff,m,n);
						n = (int)(+ sqrt(r*r - (m - i)*(m - i)) + j);
						if (n < 0) n = 0;
						if (n >= this->map_height) n = this->map_height - 1;
						this->map->DrawPixel(0xff,0xff,0xff,m,n);
					}
				break;
				}
			}
		}
	cout<<"\n	--->>> OBSTACLES EXPANDED SUCCESSFULLY <<<---	";
	this->map->ReadMap(); 				// we changed the pixel buffer, so reading the image again will regenerate the free space / MapData ;)
	this->map->SavePixelBufferToFile(); // saving the newly generated space, default file is mapname_FreeSpace.jpeg
	};
void PathPlanner::SaveSearchSpace()
	{
	Point p;
	temp = this->search_space;
	while (temp != NULL)
		{
		p = temp->location;
		this->ConvertToPixel(&p);
		this->map->DrawPixel(0x00,0xff,0x00,int(p.x),(int)p.y);// Drawing the new nodes into the Pixel Buffer
		temp =temp->next;
		}
	this->map->SavePixelBufferToFile(); // Saving the newly generated Pixel Buffer into the file
	}
void PathPlanner::GenerateRegularGrid(double distance)
	{
	if(this->map->mapdata == NULL )
		return;
	Point p;
	for(int i=0; i<this->map_width; i++)
		{
		for(int j=0;j<this->map_height;j++)
			{
			if (!this->map->mapdata[i][j]) //Free Pixel
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
void PathPlanner::BridgeTest(double length, double neigbhour_distance)
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
					if (!this->map->mapdata[i][j]&&this->map->mapdata[(int)x][(int)y]&&this->map->mapdata[(int)x2][(int)y2])
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

void PathPlanner :: ReadMap()
	{
	map = new Map(this->MapFilename,pixel_size);
	this->pixbuf = map->ReadMap();
	this->mapinfo = map->GetMapInfo();
	this->map_width=mapinfo.width;
	this->map_height=mapinfo.height;
	this->MAXNODES=map_height*map_width;
	};
void PathPlanner :: ConvertPixel(Point  *p) // transfers from pixel coordinate to the main coordinate system
{
	p->x= p->x*this->pixel_size - this->pixel_size*this->map_width/2;
	p->y=-p->y*this->pixel_size + this->pixel_size*this->map_height/2;
};
void PathPlanner ::ConvertToPixel (Point *p)
{
	p->x=( p->x + this->pixel_size*this->map_width/2)/this->pixel_size;
	p->y=(-p->y + this->pixel_size*this->map_height/2)/this->pixel_size;
}
double PathPlanner::anglediff(double alfa, double beta) 
{
	double diff;
	if( alfa < 0 ) alfa+= 2*M_PI; 	if( alfa > 2*M_PI) alfa-= 2*M_PI;
	if( beta < 0 ) beta+= 2*M_PI;	if( beta > 2*M_PI) beta-= 2*M_PI;		
	diff = alfa - beta;
	if ( diff >  M_PI) diff=( 2*M_PI  - diff);
	if ( diff < -M_PI) diff=(-2*M_PI - diff);
	return Abs(diff);
};
void PathPlanner :: PrintNodeList()
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
		//cout<<"\tStored Angle = "<< setiosflags(ios::fixed) << setprecision(2)<<RTOD(p->angle);
		if (p->next !=NULL)
		{
			//cout<<"\tNext Angle = "<< setiosflags(ios::fixed) << setprecision(2)<<RTOD(atan2(p->next->location.y - p->location.y, p->next->location.x - p->location.x));
			//cout<<"\tAngle Diff ="<< setiosflags(ios::fixed) << setprecision(2)<<RTOD(anglediff(p->angle,atan2(p->next->location.y - p->location.y, p->next->location.x - p->location.x)));
		}
		for (int i=0;i<this->number_of_point_to_check;i++)
		{
//			if(p->wheelchair.check_points.size() == 0)
//			{
//				cout<<"\n FOR some FUCKING REASON AM EMPTY :| !!!!!"; //SHOULD NEVER HAPPEN :@
//				break;
//			}
		}
		fflush(stdout);
		p = p->next;
	}
	cout <<"\n\n  --------------------   END OF LIST ---------------------- \n";
}
void PathPlanner::FindRoot() // find the nearest node to the start
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
	//root->wheelchair.SetCheckPoints(this->number_of_point_to_check,this->points_to_check);
	cout<<"\n---->>>Root is Set to be X="<<root->location.x<<" Y="<<root->location.y;
	fflush(stdout);
	};
int PathPlanner :: StartSearch(Point start_search, Point end_search,double initial_angle, double final_angle)
{
	int      ID = 1;
  	int      NodesExpanded = 0;
	this->initial_angle = initial_angle;
	this->final_angle   = final_angle;
	if(this->tree.size() > 0)		this->tree.clear();
	//cout <<"\n	--->>> 1- Initial angle="<<RTOD(this->initial_angle)<<" Final Angle="<<RTOD(this->final_angle);
	fflush(stdout);
	if (!this->map) // Make sure that we have a map to search
	{
		this->map->ReadMap();
		cout <<"\nRead the map File First !!!"<<endl;
		fflush(stdout);
		this->AddText("\nRead the map File First !!!");
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
double PathPlanner:: Calculate_g(Node *n) // define the g function as parent plus 1 step
{
	double cost;
	if(n == NULL || n->parent==NULL)
		return 0.0;
	cost = n->parent->g_value + sqrt(pow(n->location.x - n->parent->location.x,2) + pow(n->location.y - n->parent->location.y,2) );
	return cost;
};
double PathPlanner::Calculate_h(Node *n) //define the h function as the Euclidean distance to the goal + turning penalty
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
		reverse_penalty = delta_d;
	else
		reverse_penalty = 0;
	// 0.555 is the AXLE Length 
	return ( h*(1 + reverse_penalty ) + 0.555 * angle_cost + obstacle_penalty*delta_d);
};
// define the goalNode function
bool PathPlanner :: GoalReached (Node *n) 
{
	double angle_diff, delta_d;
	delta_d = sqrt(pow(n->location.x - this->end.x,2)+pow(n->location.y - this->end.y,2));
	if (n->direction == FORWARD)
		angle_diff =	anglediff(this->final_angle,n->angle);
	else
	{
		angle_diff =	anglediff(this->final_angle,n->angle + M_PI);
	}
	if ( delta_d <= 0.7  && angle_diff <= DTOR(30))
	{
		cout<<" \n Desired Final Orientation ="<<RTOD(this->final_angle)<<" Current="<<RTOD(n->angle);
		cout<<"\n Reached Destination with Diff Orientation="<< RTOD(angle_diff);
		return 1;
	}
	return 0;
};
void PathPlanner::Translate(Point  P, double theta) // Rotates and Translates the check points according to the vehicle position and orientation
{
	for(int i=0;i<this->number_of_point_to_check;i++)
	{
		this->points_to_check[i].x = (this->wheelchair->check_points[i].x*cos(theta) - this->wheelchair->check_points[i].y*sin(theta) + P.x);
		this->points_to_check[i].y = (this->wheelchair->check_points[i].x*sin(theta) + this->wheelchair->check_points[i].y*cos(theta) + P.y);
		//cout<<" After Conversion X="<<this->points_to_check[i].x<<" Y="<<this->points_to_check[i].y;
	}
};
// Rotates and Translates the check points according to the vehicle position and orientation
void PathPlanner::Translate_edges(Point  P, double theta) 
{
	for(int i=0;i<4;i++)
	{
		this->translated_edge_points[i].x = (this->local_edge_points[i].x*cos(theta) - this->local_edge_points[i].y*sin(theta) + P.x);
		this->translated_edge_points[i].y = (this->local_edge_points[i].x*sin(theta) + this->local_edge_points[i].y*cos(theta) + P.y);
		//cout<<" \nAfter Conversion X="<<this->translated_edge_points[i].x<<" Y="<<this->translated_edge_points[i].y;
	}
};
// Test for whether a point is in an obstacle or not
int PathPlanner :: Obstacle(Point P, double angle) 
{
	int m,n;
	// Rotates and Translates the check points according to the vehicle position and orientation
	Translate(P,angle); 
	for (int i=0;i<this->number_of_point_to_check;i++)
	{
		this->ConvertToPixel(&this->points_to_check[i]);
		m = (int)this->points_to_check[i].x;
		n = (int)this->points_to_check[i].y;
		//cout <<"\nx ="<<m<<" y="<<n;
		//fflush(stdout);
		if (m <= 0 || n <= 0 || m >=this->map_width || n >=this->map_height)
			return 1;
		if (this->map->mapdata[m][n]) return 1;
	}
	return 0;
};
void PathPlanner::draw_tree()
{
	GtkWidget * temp;
	//temp = lookup_widget (GTK_WIDGET(widget),"drawingarea1");
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
void PathPlanner::draw_path()
{
	if (!this->path)
	{
		cout<<"\n->NO Path Generated Yet, plan a Path First";
		return;
	}
	double angle;
	GtkWidget * temp;
	//temp = lookup_widget (GTK_WIDGET(widget),"drawingarea1");
	Point l_start,l_end,E,S;
  	Node *p;
  	p = this->path;
	while(p != NULL && p->next!=NULL)
	{
		S =  p->location;
  		//temp = lookup_widget (widget,"drawingarea1");
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
Node *PathPlanner :: MakeChildrenNodes(Node *parent)
{
	Point P; 
	Node  *p, *q;
	double start_angle,end_angle,angle,angle_difference,discrete_angle,robot_angle,child_angle,angle_resolution = DTOR(10);
	bool collides = FALSE;
	int direction;
	P.x = parent->location.x;
	P.y = parent->location.y;
	Tree t;
	t.location = P;
	if(!this->search_space)
		return NULL;
	temp = this->search_space;
	// Locate the Cell in the Search Space, necessary to determine the neighbours
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
	// Check Each neighbour
	for (unsigned int i=0;i<temp->children.size();i++) 
	{
		// What will be our orientation when we go to this child node ?
		angle = atan2(temp->children[i]->location.y - P.y,temp->children[i]->location.x - P.x); // Angle in Radians
		// How much it differs from our current orientations ?
		angle_difference = anglediff(angle,parent->angle);
		// Are we gonna turn too much ? if yes then why not go backwards ?
		if (angle_difference > DTOR(90))
		{
			//cout<<"\n Angle difference ="<<RTOD(angle_difference)<<" parent angle="<<RTOD(parent->angle)<<" destination angle="<<RTOD(angle);
			direction = parent->direction * -1;
		}
		else
		{
			direction = parent->direction;
		}
		collides = FALSE;
		/* Discreatize the turning space and check for collison
		 * 1- Angle stored in the node is the direction of the PATH
		 * 2- If we were moving Forward then the Robot direction is the same as the Path
		 * 3- If we were moving BackWard then the Robot direction is Path + M_PI
		 * 4- Determine what will the Robot orientation will be at this step
		 * 5- Check for collision detection with a certain resolution
		 */

		if (parent->direction == FORWARD)
			robot_angle = parent->angle;
		else
			robot_angle = parent->angle + M_PI;
		if (direction == FORWARD)
			child_angle = angle;
		else
			child_angle = angle + M_PI;
		if(robot_angle  < 0 ) robot_angle  += 2*M_PI;
		if(child_angle  < 0 ) child_angle  += 2*M_PI;
		// Start from the largest angle and go down
		if (robot_angle > child_angle)
		{
			start_angle  = robot_angle;
			end_angle    = child_angle;
		}
		else
		{
			start_angle  = child_angle;
			end_angle    = robot_angle;
		}
    	discrete_angle =  start_angle;
		//cout<<"\n Start is"<<RTOD(start_angle)<<" End angle="<<RTOD(end_angle);
		for (int s=0 ; s <= ceil(angle_difference/angle_resolution); s++)
		{
			if(Abs(start_angle - end_angle) >= DTOR(180))
			{
				discrete_angle += angle_resolution;
				if (discrete_angle > 2*M_PI)
					discrete_angle-= 2*M_PI;
				if(discrete_angle > end_angle)
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
				collides= true;
				continue;
			}
		}
//		if (Obstacle(temp->children[i]->location,angle))
//		{
//			collides= true;
//			continue;
//		}
//		else
//			collides = false;
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
			//p->wheelchair.SetCheckPoints(this->number_of_point_to_check,this->points_to_check);
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
void PathPlanner :: FreeNode(Node *n) 
{
	delete n;
}

}
