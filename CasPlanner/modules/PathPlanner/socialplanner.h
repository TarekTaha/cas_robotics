#ifndef SOCIALPLANNER_H_
#define SOCIALPLANNER_H_

#include "playground.h"
#include <QHash>
#include "astar.h"
#include "mapskeleton.h"
#include "map.h"


class PlayGround;
class RobotManager;

class SocialPlanner
{
	private :
		Robot *robot;
		MapSkeleton  *mapSkeleton;
		PlayGround   *playGround;
		RobotManager *robotManager;
		QHash<QString, int> socialRewards;
	public :
		CasPlanner::Astar *astar;		
		void freeResources();
		void freePath();
		void printNodeList ();
		void buildSpace();
		void showConnections();
		Node * getPath();
		SearchSpaceNode * getSearchSpace();
		vector <Tree> getTree();
		bool loadActivities(const char *filename);
		bool readSpaceFromFile(const char *filename);
		bool saveSpace2File(const char *filename);
		void setStart(Pose start);
		void setEnd(Pose start);
		QHash<QString, int> getSocialRewards();
		void setMapSkeleton(MapSkeleton *mapSke);
		void setMap(Map*);
		SocialPlanner(PlayGround *playGround,RobotManager *robotManager);
   	 	~SocialPlanner(); 	 	
   	 	SocialPlanner(){};
};

#endif /*SOCIALPLANNER_H_*/
