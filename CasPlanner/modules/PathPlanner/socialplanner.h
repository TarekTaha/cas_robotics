#ifndef SOCIALPLANNER_H_
#define SOCIALPLANNER_H_

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include <QHash>
#include "astar.h"
#include "mapskeleton.h"
#include "map.h"

namespace CasPlanner
{

class SocialPlanner : public Astar
{
	private :
		Robot *robot;
		MapSkeleton *mapSkeleton;
		QHash<QString, int> socialRewards;
	public :
		void freeResources();
		void freePath();
		void printNodeList ();
		void buildSpace();
		void showConnections();
		void saveSearchSpace();
		bool loadActivities(const char *filename);
		bool readSpaceFromFile(const char *filename);
		bool saveSpace2File(const char *filename);
		void setStart(Pose start);
		void setEnd(Pose start);
		QHash<QString, int> getSocialRewards();
		void setMapSkeleton(MapSkeleton *mapSke);
		SocialPlanner();
		SocialPlanner(Map *m, Robot *r,MapSkeleton *mapS);
   	 	~SocialPlanner();

};

}
#endif /*SOCIALPLANNER_H_*/
