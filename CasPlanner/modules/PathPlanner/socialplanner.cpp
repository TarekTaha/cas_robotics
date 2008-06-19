/***************************************************************************
 *   Copyright (C) 2006 - 2007 by                                          *
 *      Tarek Taha, CAS-UTS  <tataha@tarektaha.com>                        *
 *                                                                         *
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
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/
#include "socialplanner.h"

namespace CasPlanner
{

SocialPlanner::SocialPlanner(Map *m, Robot *r,MapSkeleton *mapS):Astar(r,0.2,"Social"),mapSkeleton(mapS)
{
	map = m;
	robot =  r;
}

SocialPlanner::SocialPlanner():mapSkeleton(NULL)
{

}

SocialPlanner :: ~SocialPlanner()
{
	freeResources();
	cout<<"\n	--->>> Allocated Memory FREED <<<---";
}

void SocialPlanner::freeResources()
{
	freeSearchSpace();
	freePath();
	p=root=test=NULL;
}

void SocialPlanner::freePath()
{
	while(path != NULL)
	{
		p = path->next;
		delete path;
		path = p;
	}
}

void SocialPlanner::setStart(Pose start)
{
	this->start = start;
}

void SocialPlanner::setEnd(Pose end)
{
	this->end= end;
}

bool SocialPlanner::loadActivities(const char *filename)
{
  	int i,id,prev;
  	assert(filename != NULL);
  	filename = strdup(filename);
  	FILE *file = fopen(filename, "r");
  	char * pch;
  	char line[250],s[10];  	
  	if (!file)
  	{
  		qDebug("Error Opening File");
    	fclose(file);
    	return false;
  	}
  	while (!feof(file))
  	{
  		fgets(line,sizeof(line),file);
		pch = strtok (line, " ");
		i=0;
  		while (pch!=NULL)
  		{
  			sscanf(pch,"%d",&id);
  			pch = strtok (NULL, " "); if(pch==NULL) break;
  			sscanf(pch,"%s",s);
  			pch = strtok (NULL, " ");
  			if(++i==1)
  			{
  				prev = id;
  			}
  			else
  			{		
  				QString s = QString("%1-%2").arg(prev).arg(id);
  				if(socialRewards.contains(s))
  					socialRewards[s] = socialRewards[s] + 1 ;
  				else
  					socialRewards[s] = 1;
  				prev = id;
  			}
  		}
  	}
  	fclose(file);
  	
//    QHash<QString, int>::const_iterator j = socialRewards.constBegin();
//    while (j != socialRewards.constEnd())
//    {
//    	printf("\n Edge:=%s Reward:=%d",qPrintable(j.key()),j.value());
//        ++j;
//    }
    
    setSocialReward(&socialRewards);
  	return true;
}

QHash<QString, int> SocialPlanner::getSocialRewards()
{
	return this->socialRewards;
}

bool SocialPlanner::readSpaceFromFile(const char *filename)
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

bool SocialPlanner::saveSpace2File(const char *filename)
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

void SocialPlanner :: printNodeList()
{
	int step=1;
	QPointF  location;
	if(!(p = this->path))
		return ;
	qDebug("\n  --------------------   START OF LIST ----------------------");
	while(p !=NULL)
	{
		location =  p->pose.p;
		cout <<"\nStep [" << step++ <<"] state ["<<mapSkeleton->getCurrentSpatialState(p->pose)<<"] x["<< location.x()<<"]y["<<location.y()<<"]"<<" Direction="<<p->direction;
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

void SocialPlanner::setMapSkeleton(MapSkeleton *mapSke)
{
	this->mapSkeleton = mapSke;
}

void SocialPlanner::showConnections()
{
	QPointF loc1,loc2;
	SearchSpaceNode *temp = search_space;
	int m=0,n=0;
	while (temp != NULL)
	{
		for(int i=0; i < temp->children.size();i++)
		{
			loc1 = temp->location;
			loc2 = temp->children[i]->location;
			m++;
		}
		temp = temp->next;
		n++;
	}
	qDebug("\n---->>> TOTAL NUMBER OF CONNECTIONS =%d\n---->>> Total Nodes in search Space =%d",m,n);
	//this->MAXNODES = 2*m;
}

void SocialPlanner::buildSpace()
{
  	SearchSpaceNode *temp=NULL,*child;
	for(int i=0; i < mapSkeleton->verticies.size(); i++)
	{
		temp  = insertNode(mapSkeleton->verticies[i].getLocation(),i+1);
		for(int j=0;j<mapSkeleton->verticies[i].connections.size();j++)
		{
			QPointF s = mapSkeleton->verticies[mapSkeleton->verticies[i].connections[j].nodeIndex].getLocation();
			child = insertNode(s,mapSkeleton->verticies[i].connections[j].nodeIndex+1);
			temp->children.push_back(child);	
		}
	}
    MAXNODES = 3000;
}

}
