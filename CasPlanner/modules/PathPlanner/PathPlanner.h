#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include <Astar.h>
#include "interfaceprovider.h"
namespace CasPlanner
{
class PathPlanner : public Astar
{
	public :
		bool map_initialized;
		double  obstacle_radius,bridge_length,bridge_res,reg_grid,conn_radius,obst_dist;
	public :
		void   setExpRad(double);
		void   setBridgeLen(double);
		void   setBridgeRes(double);
		void   setRegGrid(double);
		void   setConRad(double);
		void   setObstDist(double);
		void   FreeResources();
		void   PrintNodeList ();
		void   SetMap(Map *); // Reads the map file and sets the attributes
		void   ExpandObstacles();
		void   AddCostToNodes();
		void   BridgeTest();
		void   GenerateRegularGrid();
		void   ConnectNodes();
		void   ShowConnections();
		void   SaveSearchSpace();
		void   DetermineCheckPoints();
		void   FindRoot();
		void   FreePath();
		void   updateMap(Map *mapPatch);
		bool   CheckShortestDistance(double i,double j,double neigbhour_pixel_distance);
		bool   readSpaceFromFile(const char *filename);
		bool   saveSpace2File(const char *filename);
		PathPlanner(Robot *,double dG,double bridge_len,
					double bridge_res,double reg_grid,double obst_exp,double conn_rad,double obst_pen);
		PathPlanner();
		~PathPlanner();
};

}

#endif /*PATHPLANNER_H_*/