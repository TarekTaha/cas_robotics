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
		//PathPlanner();
		~PathPlanner();
};

}

#endif /*PATHPLANNER_H_*/
