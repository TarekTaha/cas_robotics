#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include "PathPlanner.h"
#include "configfile.h"
#include "MapManager.h"
#include "utils.h"
#include <QObject>
#include <QString>
#include <QPointF>

class Navigator : public Controller
{
	public:
		QString obst_avoid;
        int config(ConfigFile *cf, int sectionid);
        int start(); 
        int stop();
        Node * FindPath(QPointF start, QPointF end);
        int FollowPath(Node* path);
       	PathPlanner * pathPlanner;
		Navigator();
		~Navigator();
};

#endif /*NAVIGATOR_H_*/
