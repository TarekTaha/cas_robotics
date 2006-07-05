#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include <Astar.h>

namespace CasPlanner
{
class PathPlanner : public Astar
{
	public :
		bool map_initialized;
		double  obstacle_radius,bridge_length,bridge_res,reg_grid,conn_radius,obst_dist;
		QPointF start, end;
	public :
		void   setExpRad(double);
		void   setBridgeLen(double);
		void   setBridgeRes(double);
		void   setRegGrid(double);
		void   setConRad(double);
		void   setObstDist(double);
		void   FreeSearchSpace();
		void   PrintNodeList ();
		void   SetMap(QVector <QBitArray>); // Reads the map file and sets the attributes
		void   ExpandObstacles();
		void   AddCostToNodes();
		void   BridgeTest();
		bool   CheckShortestDistance(double i,double j,double neigbhour_pixel_distance);
		void   GenerateRegularGrid();
		void   ConnectNodes();
		void   ShowConnections();
		void   SaveSearchSpace();
		void   DetermineCheckPoints();
		void   FindRoot();
		void   FreePath();
		PathPlanner(double r_l ,double r_w , QString r_m ,QPointF r_c,double pixel_res,double bridge_len,
					double bridge_res,double reg_grid,double obst_exp,double conn_rad,double obst_pen);
		PathPlanner();
		~PathPlanner();
};

}

#endif /*PATHPLANNER_H_*/