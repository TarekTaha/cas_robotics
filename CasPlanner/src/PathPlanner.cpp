#include "PathPlanner.h"
namespace CasPlanner
{
// Test for node equality
int NodeEquality(Node *a, Node *b) 
{
	if (a->location.x() == b->location.x() && a->location.y() == b->location.y())
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
PathPlanner ::PathPlanner(double r_l ,double r_w , QString r_m , QPointF r_c,double pixel_res,double bridge_len,
			double bridge_r,double reg_g,double obst_exp,double conn_rad,double obst_pen):
			Astar(r_l,r_w,obst_exp,pixel_res,r_m,r_c),
			obstacle_radius(obst_exp),
			bridge_length(bridge_len),
			bridge_res(bridge_r),
			reg_grid(reg_g),
			conn_radius(conn_rad),
			obst_dist(obst_pen)
{

};
PathPlanner :: ~PathPlanner()
{
	path=p=root=test=NULL;
	while (search_space != NULL)
		{
		temp = search_space;
		search_space = search_space->next;
		delete temp;
		};
	this->FreePath();
	cout<<"\n	--->>> Allocated Memory FREED <<<---";
};

void PathPlanner::ShowConnections()
{
	QPointF loc1,loc2;
	//GtkWidget * view;
	temp = search_space;
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
void PathPlanner::ConnectNodes()
{
	SearchSpaceNode * S;
	double distance,angle;
	if (!search_space) // Do nothing if Search Space is not Yet Created
		return;
	temp = search_space;
	while (temp!=NULL)
	{
		S = search_space;
		while (S!=NULL)
		{
			distance = Dist(S->location,temp->location);
			if (distance <= conn_radius && distance !=0)
			{
				angle = atan2(S->location.y() - temp->location.y() ,S->location.x() - temp->location.x());
				if(!Obstacle(temp->location,angle))
					temp->children.push_back(S);
			}
			S = S->next;
		}
		temp = temp->next;
	}
	cout<<"\n	--->>> NODES CONNECTED <<<---	";
};
void PathPlanner::AddCostToNodes()
{
	SearchSpaceNode * S;
	QPointF  point;
	double number_of_pixels,radius,nearest_obstacle;
	int i,j;
	//int n=0;
	if (!search_space) // Do nothing if Search Space is not Yet Created
		return;
	S = search_space;
	number_of_pixels = obst_dist / this->pixel_size;
	while (S!=NULL)
	{
		//cout<<"\n Node= "<<++n;
		//fflush(stdout);
		point.setX(S->location.x());
		point.setY(S->location.y());
		this->ConvertToPixel(&point);
		nearest_obstacle = 0;
		for(radius = (int)number_of_pixels ; radius >0 ; radius --)
			{
				for( i= (int)(point.x() - radius) ; i < (int)(point.x() + radius); i+=1)
					{
						if (i < 0) i = 0;
						if (i >= this->map_width)  break;
						j = (int)(- sqrt(radius*radius - (i - point.x())*(i - point.x())) + point.y());
						if (j < 0) j = 0;
						if (j >= this->map_height) j = this->map_height - 1;
						if(this->map[i][j])
								 nearest_obstacle = radius;
						j = (int)(+ sqrt(radius*radius - (i - point.x())*(i - point.x())) + point.y());
						if (j < 0) j = 0;
						if (j >= this->map_height) j = this->map_height - 1;
	
						if(this->map[i][j])
								 nearest_obstacle = radius;
					}
			//cout<<"\n R="<<radius;
			//fflush(stdout);
			}
		// this is a normalized cost, it will make more sense later on 
		S->obstacle_cost =  (obst_dist - nearest_obstacle*this->pixel_size)/obst_dist; 
		//cout<<"\n Obstacle Cost ="<<S->obstacle_cost;
		//getchar();
		S = S->next;
	}
	cout<<"\n	--->>> NODES CONNECTED <<<---	";
};
bool PathPlanner::CheckShortestDistance(double x,double y,double neigbhour_distance)
{
	SearchSpaceNode * S;
	double distance,shortest_distance = 10000000;

	S = search_space;
	while (S!=NULL)
	{
		distance = sqrt(pow(S->location.x() - x,2)+pow(S->location.y() - y,2));
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
	if(!map_initialized)
	{
		qDebug("\n	--->>> You have to read the map before Expanding the Obstacles <<<---	");
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
				if (this->map[i][j] && !this->map[x][y])
				{
				for (int r = 1; r <(int)(thickness);r++)
					for( m = (int)(i - r); m <= (int)(i + r);m++)
					{
						if (m < 0) m = 0;
						if (m >= this->map_width)  break;
						n = (int)(- sqrt(r*r - (m - i)*(m - i)) + j);
						if (n < 0) n = 0;
						if (n >= this->map_height) n = this->map_height - 1;
						this->map[m][n] = true;
						n = (int)(+ sqrt(r*r - (m - i)*(m - i)) + j);
						if (n < 0) n = 0;
						if (n >= this->map_height) n = this->map_height - 1;
						this->map[m][n] = true;
					}
				break;
				}
				y = (int)(+ sqrt(radius*radius - (x - i)*(x - i)) + j);
				if (y < 0) y = 0;
				if (y >= this->map_height) y = this->map_height - 1;
				if (this->map[i][j] && !this->map[x][y])
				{
				for (int r = 1; r <(int)(thickness);r++)
					for( m = (int)(i - r); m <= (int)(i + r);m++)
					{
						if (m < 0) m = 0;
						if (m >= this->map_width)  break;
						n = (int)(- sqrt(r*r - (m - i)*(m - i)) + j);
						if (n < 0) n = 0;
						if (n >= this->map_height) n = this->map_height - 1;
						this->map[m][n] = true;
						n = (int)(+ sqrt(r*r - (m - i)*(m - i)) + j);
						if (n < 0) n = 0;
						if (n >= this->map_height) n = this->map_height - 1;
						this->map[m][n] = true;
					}
				break;
				}
			}
		}
	cout<<"\n	--->>> OBSTACLES EXPANDED SUCCESSFULLY <<<---	";
};
void PathPlanner::SaveSearchSpace()
{
	QPointF p;
	temp = search_space;
	while (temp != NULL)
	{
		p = temp->location;
		this->ConvertToPixel(&p);
		this->map[int(p.x())][(int)p.y()]= true ;
		temp =temp->next;
	}
}
void PathPlanner::GenerateRegularGrid()
{
	if(!map_initialized)
	{
		qDebug("\n	--->>> You have to read the map First <<<---	");
		return;
	}
	QPointF p;
	for(int i=0; i<this->map_width; i++)
	{
		for(int j=0;j<this->map_height;j++)
		{
			if (!this->map[i][j]) //Free Pixel
			{
				if (search_space == NULL ) // Constructing the ROOT NODE
				{
					temp = new SearchSpaceNode;
					temp->location.setX(i);
					temp->location.setY(j);
					this->ConvertPixel(&temp->location);
					temp->parent   = NULL;
					temp->next     = NULL;
					search_space = temp;
				}
				else
				{
					p.setX(i);
					p.setY(j);
					this->ConvertPixel(&p);
						if (CheckShortestDistance(p.x(),p.y(),reg_grid))
						{
							temp = new SearchSpaceNode;
							temp->location.setX(p.x());
							temp->location.setY(p.y());
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
void PathPlanner::BridgeTest()
{
	int radius,pixels_per_bridge;
	QPointF p;
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
					if (!this->map[i][j]&&this->map[(int)x][(int)y]&&this->map[(int)x2][(int)y2])
					{
						p.setX(i);
						p.setY(j);
						this->ConvertPixel(&p);
						if (search_space == NULL ) // Constructing the ROOT NODE
						{
							temp = new SearchSpaceNode;
							temp->location.setX(p.x());
							temp->location.setY(p.y());
							temp->parent   = NULL;
							temp->next     = NULL;
							search_space = temp;
						}
						else
						{
							if (CheckShortestDistance(p.x(),p.y(),bridge_res))
							{
								temp = new SearchSpaceNode;
								temp->location.setX(p.x());
								temp->location.setY(p.y());
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

void PathPlanner :: SetMap(QVector <QBitArray> map)
{
	this->map = map;
	this->map_width = map[0].size();
	this->map_height = map.size();
	this->MAXNODES=map_height*map_width;
	map_initialized = true;
};
void PathPlanner :: PrintNodeList()
{
	int step=1;
	QPointF  pixel;
	if(!(p = this->path))
		return ;
	cout <<"\n  --------------------   START OF LIST ---------------------- \n";
	while(p !=NULL)
	{
		pixel =  p->location;
		cout <<"\nStep [" << step++ <<"] x["<< pixel.x()<<"]y["<<pixel.y()<<"]"<<" Direction="<<p->direction; 
		cout <<"\tG cost="<<p->g_value<<"\tH cost="<<p->h_value<<"\tFcost="<<p->f_value;
		//cout<<"\tStored Angle = "<< setiosflags(ios::fixed) << setprecision(2)<<RTOD(p->angle);
		if (p->next !=NULL)
		{
			//cout<<"\tNext Angle = "<< setiosflags(ios::fixed) << setprecision(2)<<RTOD(atan2(p->next->location.y - p->location.y, p->next->location.x - p->location.x));
			//cout<<"\tAngle Diff ="<< setiosflags(ios::fixed) << setprecision(2)<<RTOD(anglediff(p->angle,atan2(p->next->location.y - p->location.y, p->next->location.x - p->location.x)));
		}
		for (int i=0;i<this->check_points.size();i++)
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

void PathPlanner::draw_tree()
{
	//GtkWidget * temp;
	//temp = lookup_widget (GTK_WIDGET(widget),"drawingarea1");
	QPointF l_start,l_end;
	for (unsigned int i =0; i<tree.size();i++)
		{
			l_start = tree[i].location;
			ConvertToPixel(&l_start);
			//cout<<"\n Main X="<<tree[i].location.x<<" Y="<<tree[i].location.y;
			for(int j=0;j<tree[i].children.size();j++)
			{
				l_end = tree[i].children[j];
				ConvertToPixel(&l_end);
				//gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)l_start.x,(int)l_start.y,
				//(int)l_end.x,(int)l_end.y);
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
	//GtkWidget * temp;
	//temp = lookup_widget (GTK_WIDGET(widget),"drawingarea1");
	QPointF l_start,l_end,E,S;
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
		//Translate_edges(S,angle);
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
  			//gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)l_start.x,(int)l_start.y,(int)l_end.x,(int)l_end.y);
		}
		p = p->next;
	} 
};

}
