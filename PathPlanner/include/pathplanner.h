/***************************************************************************
 *   Copyright (C) 2007 by Tarek Taha                                      *
 *   tataha@eng.uts.edu.au                                                 *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include "robot.h"
#include "map.h"
#include "astar.h"
#include "timer.h"
#include <fstream>
#include <errno.h>

namespace CasPlanner
{
class PathPlanner : public Astar
{
	private :
		bool renderTree,mapInitialized,bridgeTestEnabled,regGridEnabled,obstPenEnabled,expObstEnabled,showTreeEnabled,negate;		
		double  obstacle_radius,bridge_length,bridge_res,regGridDist,reg_grid_conn_rad,obst_dist,bridge_conn_rad;
		bool  fileExist(const char * fname);
		void  freeResources();
		void  saveSearchSpace();
		void  freePath();
		bool  parametersChanged(const char *fileName);		
		bool  isEmptyLine(const char* buf);	
		bool  checkShortestDistance(double i,double j,double neigbhour_pixel_distance);
		bool  readSpaceFromFile(const char *filename);
		bool  saveSpace2File(const char *filename);
		void  expandObstacles();
		void  addCostToNodes();
		void  bridgeTest();
		void  generateRegularGrid();
		void  connectNodes();
	public :
		void  enableBridgeTest(bool);
        void  enableRegGrid(bool);
        void  enableObstPen(bool);
        void  enableExpObst(bool);
        void  enableShowTree(bool);
        void  setStart(Pose);
        void  setEnd(Pose);
        void  setMap(Map *);
		void  setExpRad(double);
		void  setBridgeLen(double);
		void  setBridgeRes(double);
		void  setRegGrid(double);
		void  setBridgeConRad(double);
		void  setRegGridConRad(double);
		void  setObstDist(double);
        void  generateSpace();
		void  printNodeList ();
		void  showConnections();
		Node* findPath(int coord);
		Node* findPath(Pose start, Pose end,int coord);		
		void  drawPath();
		void  drawSearchSpace();
		PathPlanner(Robot *,Map *map,double dG,double bridge_len,double bridge_res,double regGridDist,double reg_grid_conn_rad,double obst_pen,double bridge_conn_rad);
		~PathPlanner();
};

}

#endif /*PATHPLANNER_H_*/
