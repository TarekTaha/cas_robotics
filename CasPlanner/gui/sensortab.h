/***************************************************************************
 *   Copyright (C) 2006 by Waleed Kadous   *
 *   waleed@width   *
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
#ifndef SENSORTAB_H
#define SENSORTAB_H

#include <QWidget>
#include <ogrenderer.h>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QTreeWidget>
#include <QPushButton>
#include <QHash>
/**
	@author Waleed Kadous <waleed@cse.unsw.edu.au>
*/

class MapControlPanel: public QWidget 
{
Q_OBJECT
    public:  
	MapControlPanel(QWidget *parent = 0);
	//void setMapManager(QTMapDataInterface *mapManager);
    public slots:
	void updateMap(); 
	void handleSelection(); 
	void updateSelectedObject(double);
	void updateSelectedObject(int); 
	void load(); 
	void save();
	void captureMap(); 
	void exportHtml(); 
	void setToRoot(); 
    signals:
	void propsChanged(); 
    private: 
	//void setActionValues(MapObject *mo);
	QGroupBox showHideGB; 
	QCheckBox showGrids; 
	QCheckBox showOGs;
	QCheckBox showRobots; 
	QCheckBox showLabels; 
	QCheckBox showPointclouds; 
	QCheckBox showPatchBorders;
	QTreeWidget selectedObject;
	QGroupBox transformGB;
	QSpinBox visSB; 
	QDoubleSpinBox xSB; 
	QDoubleSpinBox ySB;
	QDoubleSpinBox zSB; 
	QDoubleSpinBox rSB;
	QDoubleSpinBox pSB;
	QDoubleSpinBox yaSB; 
	QPushButton setToRootBtn; 
	//QTMapDataInterface *mapManager;
	QGroupBox actionGB;
	QPushButton captureMapBtn; 
	QPushButton loadBtn; 
	QPushButton saveAsBtn;
	QPushButton exportBtn;
	//QHash<QTreeWidgetItem*, MapObject *> wiToMo;
	//QHash<MapObject *, QTreeWidgetItem *> moToWi;   
	friend class SensorContainer; 
	
	static const int RobotNodeType = QTreeWidgetItem::UserType+1; 
	static const int PatchNodeType = QTreeWidgetItem::UserType+2;
	static const int SnapNodeType = QTreeWidgetItem::UserType+3;
	
};

class SensorContainer : public QWidget
{
Q_OBJECT
    public:
	SensorContainer(QWidget *parent = 0);
	~SensorContainer();
	//void setMapManager(QTMapDataInterface *mapManager);
    private:
	OGRenderer ogRenderer;
	MapControlPanel mapControlPanel; 
	//QTMapDataInterface *mapManager; 
	friend class MapControlPanel; 
};

#endif
