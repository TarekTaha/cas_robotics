#ifndef PLAYGROUNDTAB_H_
#define PLAYGROUNDTAB_H_

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QLabel>
#include <QtGui/QListView>
#include <QtGui/QSplitter>
#include <QtGui/QTableWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

#include "playground.h"

class PlayGroundTab : public QWidget
{
//	Q_OBJECT	
	public:
		PlayGroundTab(QWidget *parent=0,PlayGround *playG=0);
		~PlayGroundTab(){};
		PlayGround * playGround;
	    QHBoxLayout *hboxLayout;
	    QGroupBox *groupBox;
	    QSplitter *splitter;
	    QWidget *widget1;
	    QVBoxLayout *vboxLayout;
	    QLabel *label;
	    QListView *listView;
	    QWidget *widget2;
	    QVBoxLayout *vboxLayout1;
	    QLabel *label_2;
	    QTableWidget *tableWidget;	
};

#endif /*PLAYGROUNDTAB_H_*/
