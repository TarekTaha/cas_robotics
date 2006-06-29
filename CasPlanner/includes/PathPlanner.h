#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_
#include <defs.h>
#include <QPointF>
#include <QList>
#include <QImage>
#include <Robot.h>
#include <Point.h>
#include <LList.h>
#include <Map.h>
#include <Node.h>
#include <SearchSpaceNode.h>
#include <math.h>
#include <player/driver.h>
#define FORWARD 1
#define BACKWARD -1

namespace CasPlanner
{
typedef struct _tree
	{
		QPointF location;
		vector <QPointF> children;
	}Tree;
class PathPlanner , public Robot
	{
	public :
		// Location of the wheelchair's points to check in the wheelchair coordinate system
		//Robot * wheelchair; 
		int number_of_point_to_check,map_height,map_width,MAXNODES;
		double pixel_size,initial_angle,obstacle_radius,node_gap,bridge_length,final_angle;
		QPointF start, end, * points_to_check,local_edge_points[4],translated_edge_points[4];
		const char * MapFilename;
		MapInfo mapinfo;	
		Node * test,*root,*path, *p;
		SearchSpaceNode * search_space,* temp;
		Node *current, *childList, *curChild, *q;
		//LList *openList,*closedList;
		QList <Node> openList,closedList;
		//list <Node> openList,closedList;
		QImage map;
		//PathFollower follower;
		vector <Tree> tree;
	public :
		double anglediff(double alfa, double beta);
		Node  *MakeChildrenNodes(Node *parent) ;
		void   FreeNode      (Node *);
		void   PrintNodeList ();
		void   Translate(QPointF  , double);
		void   Translate_edges(QPointF  , double);
		int    check_line (QPointF start,QPointF end);
		int    Obstacle   (QPointF p, double angle);
		void   ConvertPixel   (QPointF *p);
		void   ConvertToPixel (QPointF *p);
		void   ReadMap(); // Reads the map file and sets the attributes
		void   ExpandObstacles();
		void   AddCostToNodes(double distance);
		void   BridgeTest(double length,double gap);
		bool   CheckShortestDistance(double i,double j,double neigbhour_pixel_distance);
		void   GenerateRegularGrid(double gap);
		void   ConnectNodes(double distance);
		void   ShowConnections();
		void   SaveSearchSpace();
		void   DetermineCheckPoints();
		void   FindRoot();
		void   draw_path();
		void   draw_tree();
		void   Print();
		void   FreePath();
		void   PathFollow(Node *,double kd, double kt,double ko,double tracking_distance);
		PathPlanner(double r_l ,double r_w , double r_m ,double pixel_res,double bridge_len,
					double bridge_res,double reg_grid,double obst_exp,double conn_rad,double obst_pen);
		PathPlanner();
		~PathPlanner();
	};

}

#endif /*PATHPLANNER_H_*/
