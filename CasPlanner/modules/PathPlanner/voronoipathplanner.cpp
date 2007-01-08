//
// C++ Implementation: voronoiVoronoiPathPlanner
//
// Description:
//
//
// Author: Tarek Taha <tataha@eng.uts.edu.au>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "voronoipathplanner.h"

namespace CasPlanner
{

VoronoiPathPlanner::VoronoiPathPlanner(SSkelPtr & sskel):sskel(sskel)
{
}


VoronoiPathPlanner :: ~VoronoiPathPlanner()
{
	freeResources();
	cout<<"\n	--->>> Allocated Memory FREED <<<---";
};

void VoronoiPathPlanner::freeResources()
{
	freeSearchSpace();
	freePath();
	p=root=test=NULL;
}

void VoronoiPathPlanner::freePath()
{
	while(path != NULL)
	{
		p = path->next;
		delete path;
		path = p;
	}
};

void VoronoiPathPlanner::setStart(Pose start)
{
	this->start = start;
};

void VoronoiPathPlanner::setEnd(Pose end)
{
	this->end= end;
};
bool VoronoiPathPlanner::readSpaceFromFile(const char *filename)
{
  	double locationx,locationy,obstacle_cost;
  	SearchSpaceNode *temp;
  	assert(filename != NULL);
  	filename = strdup(filename);
  	FILE *file = fopen(filename, "r");
  	if (!file)
  	{
  		qDebug("Error Opening File");
    	fclose(file);
    	return false;
  	}
  	while (!feof(file))
  	{
  		fscanf(file,"%lf %lf %lf\n",&locationx,&locationy,&obstacle_cost);
		if (search_space == NULL ) // Constructing the ROOT NODE
		{
			temp = new SearchSpaceNode;
			temp->location.setX(locationx);
			temp->location.setY(locationy);
			temp->obstacle_cost = obstacle_cost;
			temp->parent   = NULL;
			temp->next     = NULL;
			search_space = temp;
		}
		else
		{
			temp = new SearchSpaceNode;
			temp->location.setX(locationx);
			temp->location.setY(locationy);
			temp->parent = NULL;
			temp->next   = search_space;
			search_space = temp;
		}
  	}
  	fclose(file);
  	return true;
}

bool VoronoiPathPlanner::saveSpace2File(const char *filename)
{
  	assert(filename != NULL);
  	filename = strdup(filename);
  	FILE *file = fopen(filename, "wb");
  	if (!file)
  	{
  		qDebug("Error Opening File");
    	fclose(file);
    	return false;
  	}
  	SearchSpaceNode *temp=search_space;
  	while (temp)
  	{
  		fprintf(file,"%f %f %f\n",temp->location.x(),temp->location.y(),temp->obstacle_cost);
		temp = temp->next;
  	}
  	fclose(file);
  	return true;
}

void VoronoiPathPlanner :: printNodeList()
{
	int step=1;
	QPointF  location;
	if(!(p = this->path))
		return ;
	qDebug("\n  --------------------   START OF LIST ----------------------");
	while(p !=NULL)
	{
		location =  p->pose.p;
		cout <<"\nStep [" << step++ <<"] x["<< location.x()<<"]y["<<location.y()<<"]"<<" Direction="<<p->direction;
		cout <<"\tG cost="<<p->g_value<<"\tH cost="<<p->h_value<<"\tFcost="<<p->f_value;
		fflush(stdout);
		//cout<<"\tStored Angle = "<< setiosflags(ios::fixed) << setprecision(2)<<RTOD(p->angle);
		if (p->next !=NULL)
		{
			//cout<<"\tNext Angle = "<< setiosflags(ios::fixed) << setprecision(2)<<RTOD(atan2(p->next->location.y - p->location.y, p->next->location.x - p->location.x));
			//cout<<"\tAngle Diff ="<< setiosflags(ios::fixed) << setprecision(2)<<RTOD(anglediff(p->angle,atan2(p->next->location.y - p->location.y, p->next->location.x - p->location.x)));
		}
		p = p->next;
	}
	qDebug("\n --------------------   END OF LIST ---------------------- ");fflush(stdout);
}

void VoronoiPathPlanner::showConnections()
{
	QPointF loc1,loc2;
	SearchSpaceNode *temp = search_space;
	int m=0,n=0;
	while (temp != NULL)
	{
		for(int i=0; i < temp->children.size();i++)
		{
			loc1 = temp->location;
			convert2Pix(&loc1);
			loc2 = temp->children[i]->location;
			convert2Pix(&loc2);
			m++;
		}
		temp = temp->next;
		n++;
	}
	qDebug("\n---->>> TOTAL NUMBER OF CONNECTIONS =%d\n---->>> Total Nodes in search Space =%d",m,n);
	this->MAXNODES = 2*m;
}

void VoronoiPathPlanner::buildSpace()
{
  	SearchSpaceNode *temp,*child;
  	const Halfedge_const_handle null_halfedge ;
  	const Vertex_const_handle   null_vertex ;
    if ( !this->sskel )
	{
		qDebug("\n Skeleton not assigned YET !!!");
		fflush(stdout);
		return ;
	}

    int watchdog_limit = sskel->size_of_halfedges();
    for ( Face_const_iterator fit = sskel->faces_begin(), efit = sskel->faces_end(); fit != efit ; ++ fit)
	{
    	Halfedge_const_handle hstart = fit->halfedge();
     	Halfedge_const_handle he     = hstart ;
      	int watchdog = watchdog_limit ;

		do
      	{
        	if ( he == null_halfedge )
          		break ;
        	if ( he->is_bisector() )
        	{
          		bool lVertexOK      = he->vertex() != null_vertex ;
          		bool lOppositeOK    = he->opposite() != null_halfedge ;
          		bool lOppVertexOK   = lOppositeOK && he->opposite()->vertex() != null_vertex ;
				bool lVertexHeOK    = lVertexOK && he->vertex()->halfedge() != null_halfedge ;
          		bool lOppVertexHeOK = lOppVertexOK && he->opposite()->vertex()->halfedge() != null_halfedge ;

          		if ( lVertexOK && lOppVertexOK && lVertexHeOK && lOppVertexHeOK )
				{
			    	//he->is_inner_bisector()? glColor4f(0,0,1,1) : glColor4f(1,0,0,1);
			    	if(!(temp = nodeExists(QPointF(he->vertex()->point().x(),he->vertex()->point().y()))))
			    		temp  = insertNode(QPointF(he->vertex()->point().x(),he->vertex()->point().y()));
					if(!(child = nodeExists(QPointF(he->opposite()->vertex()->point().x(),he->opposite()->vertex()->point().y()))))
						child = insertNode(QPointF(he->opposite()->vertex()->point().x(),he->opposite()->vertex()->point().y()));
					temp->children.push_back(child);
          		}
        	}
        	he = he->next();
      	}
		while ( -- watchdog > 0 && he != hstart ) ;
    }
    MAXNODES = 500;
}


void VoronoiPathPlanner::saveSearchSpace()
{
	QPointF p;
	SearchSpaceNode *temp = search_space;
	while (temp != NULL)
	{
		p = temp->location;
		this->convert2Pix(&p);
		this->map->grid[int(p.x())][(int)p.y()]= true ;
		temp =temp->next;
	}
}

/*! Sets the converted QImage or the Laser Scan Map to
 * the Current Planner
 */
void VoronoiPathPlanner :: setSkeleton(SSkelPtr & ssk)
{
	if(!ssk)
		qDebug("Why the hell ur giving me an empty Skeleton ???");
	this->sskel = ssk;
	skelInitialize = true;
};

}
