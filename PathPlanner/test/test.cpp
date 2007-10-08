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

#include "pathplanner.h"
#include "robot.h"
#include "map.h"

using namespace CasPlanner;
int main( int argc, char ** argv ) 
{
    bool negate = false;
    Pose start(5.721,-4.324,DTOR(180)),end(-9.5,-4.3,DTOR(180));
    double robotH=0.9,robotW=0.5,narrowestPath=0.987,mapRes= 0.05;
    double distanceToGoal = 0.4,bridgeLen=2.5,bridgeRes=0.1,regGridLen=0.2,regGridConRad=0.4,obstPenalty=3.0,bridgeConRad=0.5;

    Point robotCenter(-0.3f,0.0f);
    Robot *robot= new Robot(string("Robot"),robotH,robotW,narrowestPath,robotCenter);
    Map   *map  = new Map("include/casarea.png",mapRes,negate);
	
    PathPlanner * pathPlanner = new PathPlanner(robot,map,distanceToGoal,bridgeLen,bridgeRes,regGridLen,regGridConRad,obstPenalty,bridgeConRad);;
    
    pathPlanner->enableBridgeTest(true);
    pathPlanner->enableRegGrid(true);
    pathPlanner->enableObstPen(true);
    pathPlanner->enableExpObst(true);
    /*
     *  Search Space generation, Please note that you will need
     * to generate only one search space for each map. After
     * the Search Space generation you can search for as many paths
     * as you want.
     */
    pathPlanner->generateSpace();
    pathPlanner->printNodeList();
    
    pathPlanner->findPath(start,end,METRIC);
    /* 
     * Draws the Search Space and the Path into the map image and saves
     * them into two different images ending with _searchSpace.png and 
     * _path.png
     */
    pathPlanner->drawSearchSpace();
    pathPlanner->drawPath();
 
    delete robot;
    delete map;
    delete pathPlanner;
    return 1;
}
