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
#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <libplayerc++/playerc++.h>
#include <libplayerinterface/player.h>

#include <QMainWindow>

#include "playground.h"
#include "statusbar.h"
#include "tabcontainer.h"
class MainWindow : public QMainWindow 
{
Q_OBJECT
public:
	MainWindow(QWidget *parent=0); 
	MainWindow(QStringList strings, QWidget *parent=0); 
	~MainWindow();
	void createMenus();
	void createActions();
    public slots:
		void logData();
		void commStart(); 
		void captureScreenShot();
private: 
    TabContainer * tabcontainer;
	PlayGround   * playGround;
	int logCount,imageCounter;
	QPixmap originalPixmap; 
	QString fileName;
    QMenu 	*fileMenu;
    QMenu 	*helpMenu;
    QAction *openAct;
    QAction *saveAsAct;
    QAction *exitAct;
    QAction *aboutAct;
    QAction *aboutQtAct;	
};
#endif

