#ifndef SOCIALPLANNER_H_
#define SOCIALPLANNER_H_

#include "astar.h"
#include "mapskeleton.h"
#include "map.h"

namespace CasPlanner
{

class SocialPlanner : public Astar
{
	private :
		Map *map;
		Robot *robot;
		MapSkeleton *mapSkeleton;
	public :
		void freeResources();
		void freePath();
		void printNodeList ();
		void buildSpace();
		void showConnections();
		void saveSearchSpace();
		bool readSpaceFromFile(const char *filename);
		bool saveSpace2File(const char *filename);
		void setStart(Pose start);
		void setEnd(Pose start);
		void setMapSkeleton(MapSkeleton *mapSke);
		SocialPlanner();
		SocialPlanner(Map *m, Robot *r,MapSkeleton *mapS);
   	 	~SocialPlanner();

};

}
#endif /*SOCIALPLANNER_H_*/
