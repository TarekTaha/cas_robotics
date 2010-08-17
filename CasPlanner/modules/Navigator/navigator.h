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
#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include <QObject>
#include <QString>
#include <QPointF>

#include <QReadWriteLock>
#include <QTime>
#include <vector>
#include "mapmanager.h"
#include "icp.h"
#include "timer.h"
#include "controller.h"
#include "pathplanner.h"


class ConfigFile;
class PlanningManager;
class RobotManager;
class PlayGround;
class ForceField;
class PlayGround;

using namespace CasPlanner;

class Navigator : public Controller
{
    Q_OBJECT
public:
    Navigator(PlayGround *,RobotManager *);
    ~Navigator();
    QString obst_avoid;
    Node * globalPath, *localPath;
    int readConfigs(ConfigFile *cf);
    void setPath(Node *path);
    void setObstAvoidAlgo(int);
    int  getObstAvoidAlgo();
    double nearestObstacle(LaserScan laser_scan);
    bool inLaserSpace(LaserScan laserScan,Pose robotLocation,QPointF waypoint);
    void  setPause(bool pause);
    Node * closestPathSeg(QPointF location,Node * all_path);
    Node * closestPathNode(QPointF location,Node * all_path);
    void generateLocalMap(LaserScan laserScan, Pose rob_location);
    bool mapModified(LaserScan laserScan, Pose rob_location);
    bool getGoal(LaserScan laserScan, Pose &goal);
    void stop();
    void run();
    void setupLocalPlanner();
    Pose wayPoint,lastWayPoint;
    PlayGround * playGround;
    RobotManager *robotManager;
    QVector<QPointF> trail;
    public
Q_SLOTS:
    void FollowPath();
    void StopNavigating();
Q_SIGNALS:
    void drawLocalPath(PathPlanner *,Pose *,int *);
    void pathTraversed();
    void glRender();
    void setWayPoint(Pose *);
    void renderMapPatch(Map*);
    void addMsg(int,int,QString);
protected:
    Pose	old_amcl,currentPose,EstimatedPos,laser_pose;
    double 	angle,prev_angle,theta,error_orientation,
    displacement,wdem,distance,distance_to_next,
    minR,minL,minAhead,avoidance_distance_left,
    avoidance_distance_right,avoidance_distance_ahead,speed,estimate_x,
    estimate_y,estimate_theta,velocity,delta_t,last_time;
    double robot_length, robot_width,pixel_res,dist_goal,local_dist,traversable_dist,
    bridge_len, bridge_res, reg_grid, obst_exp, conn_rad, obst_pen, bridge_conn_rad;
    QString robot_model;
    QPointF rotation_center;
    ForceField *FF;
    QPointF	begin,tracking_point,ni,SegmentStart,SegmentEnd;
    bool	log,position_found,end_reached,segment_navigated,stop_navigating,pause;
    int		platform,direction,path2Draw,obstAvoidAlgo;
    Node * local_path,* global_path,*last,*first,*path2Follow;
    PlanningManager *local_planner,*global_planner;
    MapManager mapManager;
    Map * mapPatch;
    Geom2D::ICP icp;
    std::vector<Geom2D::Point> local_map_icp,laser_scan_icp;
    Geom2D::Pose	delta_pose;
    QVector<QPointF> local_map;
    FILE * file;
    QReadWriteLock dataLock;
};

#endif /*NAVIGATOR_H_*/
