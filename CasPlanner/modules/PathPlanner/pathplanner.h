#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include <astar.h>
#include "interfaceprovider.h"
namespace CasPlanner
{
class PathPlanner : public Astar
{
	public :
		bool map_initialized;
		double  obstacle_radius,bridge_length,bridge_res,regGridDist,reg_grid_conn_rad,obst_dist,bridge_conn_rad;
	public :
		void   setExpRad(double);
		void   setBridgeLen(double);
		void   setBridgeRes(double);
		void   setRegGrid(double);
		void   setConRad(double);
		void   setObstDist(double);
		void   freeResources();
		void   printNodeList ();
		void   setMap(Map *); // Reads the map file and sets the attributes
		void   expandObstacles();
		void   addCostToNodes();
		void   bridgeTest();
		void   generateRegularGrid();
		void   connectNodes();
		void   showConnections();
		void   saveSearchSpace();
		void   determineCheckPoints();
		void   findRoot();
		void   freePath();
		void   updateMap(Map *mapPatch);
		bool   checkShortestDistance(double i,double j,double neigbhour_pixel_distance);
		bool   readSpaceFromFile(const char *filename);
		bool   saveSpace2File(const char *filename);
		PathPlanner(Robot *,double dG,double bridge_len,
					double bridge_res,double regGridDist,double obst_exp,double reg_grid_conn_rad,double obst_pen,double bridge_conn_rad);
		PathPlanner();
		~PathPlanner();
};

}

#endif /*PATHPLANNER_H_*/
