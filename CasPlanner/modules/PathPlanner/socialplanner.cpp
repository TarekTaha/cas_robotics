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


SocialPlanner::SocialPlanner(PlayGround *playGround,RobotManager * robotManager)
{
	robot 		= robotManager->robot;
	astar 		= new CasPlanner::Astar(robot,0.2,"Social");
	if(playGround->mapManager)
	{
		mapSkeleton = &playGround->mapManager->mapSkeleton;
		astar->map 	= playGround->mapManager->globalMap;	
	}
}

SocialPlanner :: ~SocialPlanner()
{
	freeResources();
	cout<<"\n	--->>> Allocated Memory FREED <<<---";
}

void SocialPlanner::freeResources()
{
	astar->freeSearchSpace();
	freePath();
	astar->p=astar->root=astar->test=NULL;
}

void SocialPlanner::freePath()
{
	while(astar->path != NULL)
	{
		astar->p = astar->path->next;
		delete astar->path;
		astar->path = astar->p;
	}
}

void SocialPlanner::setStart(Pose start)
{
	astar->start = start;
}

void SocialPlanner::setEnd(Pose end)
{
	astar->end= end;
}

bool SocialPlanner::loadActivities(const char *filename)
{
  	int i,id;
  	assert(filename != NULL);
  	filename = strdup(filename);
  	FILE *file = fopen(filename, "r");
  	char * pch;
  	char line[250],obs[10];  	
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
		QVector <int> task;
  		while (pch!=NULL)
  		{
  			sscanf(pch,"%d",&id);
  			task.push_back(id);
  			pch = strtok (NULL, " "); if(pch==NULL) break;
  			sscanf(pch,"%s",obs);
  			pch = strtok (NULL, " ");
  			/*
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
  			}*/
  		}
  		int dest = task[task.size()-1];
  		for(int j=0;j<task.size()-1;j++)
  		{
  			QString s = QString("%1-%2-%3").arg(task[j]).arg(task[j+1]).arg(dest);
			if(socialRewards.contains(s))
				socialRewards[s] = socialRewards[s] + 1 ;
			else
				socialRewards[s] = 1;  			
  		}
  	}
  	fclose(file);
  	
    QHash<QString, int>::const_iterator j = socialRewards.constBegin();
    while (j != socialRewards.constEnd())
    {
    	printf("\n Edge:=%s Reward:=%d",qPrintable(j.key()),j.value());
        ++j;
    }
    
    astar->setSocialReward(&socialRewards);
  	return true;
}

QHash<QString, int> SocialPlanner::getSocialRewards()
{
	return this->socialRewards;
}

void SocialPlanner::setMapSkeleton(MapSkeleton *mapSke)
{
	this->mapSkeleton = mapSke;
}

void SocialPlanner::setMap(Map * mapData)
{
	astar->map = mapData;
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
		if (astar->search_space == NULL ) // Constructing the ROOT NODE
		{
			temp = new SearchSpaceNode;
			temp->location.setX(locationx);
			temp->location.setY(locationy);
			temp->obstacle_cost = obstacle_cost;
			temp->parent   = NULL;
			temp->next     = NULL;
			astar->search_space = temp;
		}
		else
		{
			temp = new SearchSpaceNode;
			temp->location.setX(locationx);
			temp->location.setY(locationy);
			temp->parent = NULL;
			temp->next   = astar->search_space;
			astar->search_space = temp;
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
  	SearchSpaceNode *temp=astar->search_space;
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
	if(!(astar->p = astar->path))
		return ;
	qDebug("\n  --------------------   START OF LIST ----------------------");
	while(astar->p !=NULL)
	{
		location =  astar->p->pose.p;
		cout <<"\nStep [" << step++ <<"] state ["<<mapSkeleton->getCurrentSpatialState(astar->p->pose)<<"] x["<< location.x()<<"]y["<<location.y()<<"]"<<" Direction="<<astar->p->direction;
		cout <<"\tG cost="<<astar->p->g_value<<"\tH cost="<<astar->p->h_value<<"\tFcost="<<astar->p->f_value;
		fflush(stdout);
		//cout<<"\tStored Angle = "<< setiosflags(ios::fixed) << setprecision(2)<<RTOD(p->angle);
		if (astar->p->next !=NULL)
		{
			//cout<<"\tNext Angle = "<< setiosflags(ios::fixed) << setprecision(2)<<RTOD(atan2(p->next->location.y - p->location.y, p->next->location.x - p->location.x));
			//cout<<"\tAngle Diff ="<< setiosflags(ios::fixed) << setprecision(2)<<RTOD(anglediff(p->angle,atan2(p->next->location.y - p->location.y, p->next->location.x - p->location.x)));
		}
		astar->p = astar->p->next;
	}
	qDebug("\n --------------------   END OF LIST ---------------------- ");fflush(stdout);
}

void SocialPlanner::showConnections()
{
	QPointF loc1,loc2;
	SearchSpaceNode *temp = astar->search_space;
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

Node * SocialPlanner::getPath()
{
	if(astar)
		return astar->path;
	return NULL;
}

SearchSpaceNode * SocialPlanner::getSearchSpace()
{
	if(astar)
		return astar->search_space;
	return NULL;	
}

vector <Tree> SocialPlanner::getTree()
{
	if(astar)
		return astar->tree;
	//TODO: This is stupid, i will worry about it later (quick fix)
	vector <Tree> tree;
	return tree;
}

void SocialPlanner::buildSpace()
{
  	SearchSpaceNode *temp=NULL,*child;
	for(int i=0; i < mapSkeleton->verticies.size(); i++)
	{
		temp  = astar->insertNode(mapSkeleton->verticies[i].getLocation(),i+1);
		for(int j=0;j<mapSkeleton->verticies[i].connections.size();j++)
		{
			QPointF s = mapSkeleton->verticies[mapSkeleton->verticies[i].connections[j].nodeIndex].getLocation();
			child = astar->insertNode(s,mapSkeleton->verticies[i].connections[j].nodeIndex+1);
			temp->children.push_back(child);	
		}
	}
	astar->MAXNODES = 3000;
}

