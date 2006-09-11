#ifndef MAPPAINTER_H_
#define MAPPAINTER_H_

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include <QWidget>
#include <QMouseEvent>
#include <QPainter>
#include <QColor>
#include <QString>
#include "utils.h"
#include "PathPlanner.h"
#include "SearchSpaceNode.h"
#include "navigationtab.h"

using namespace CasPlanner;
class NavControlPanel;

class MapPainter : public QWidget
{
	Q_OBJECT
	public:
		MapPainter(QWidget *parent=0,QString name=0,NavControlPanel *navControlPanel=0);
		virtual ~MapPainter();
		void   setPathEnabled(int);
		Pose   local_pose,pose;
		void  SetMapFileName(QString name);
	signals:
		void setMap(QImage);
		void setStart(Pose);
		void setEnd(Pose);	
	public slots:
		void drawPath(PathPlanner *,Pose,int *);
		void drawPath(PathPlanner *);
	protected:
		QString mapName;
		QImage image;
		int step,path2Draw;
		QPoint mouse_pos;
		NavControlPanel *navControlPanel;
		PathPlanner * local_planner,* global_planner;
		bool start_initialized,end_initialized,drawPathEnabled,drawTreeEnabled;
		Pose start,end;
		void paintEvent(QPaintEvent *);
		void mousePressEvent ( QMouseEvent * event );
		void mouseMoveEvent ( QMouseEvent * event );
};

#endif /*MAPPAINTER_H_*/
