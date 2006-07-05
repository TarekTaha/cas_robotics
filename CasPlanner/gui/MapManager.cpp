#include "MapManager.h"
#include "utils.h"
MapManager::MapManager()
{
}

MapManager::~MapManager()
{
}

QVector <QBitArray> MapManager::provideLaserOG(QVector<QPointF> laser_scan)
{
	QVector <QBitArray> retval;
	double dist=0,max_range=0,res=0.05;
	int height,width;
	for(int i =0; i<laser_scan.size();i++)
	{
		dist = Dist(QPointF(0,0),laser_scan[i]);
		if (dist >max_range)
			max_range = dist;
	}
	height = width = int(2.0*dist/res);
	qDebug("Width:%d Height:%d",width,height);
	// Create Data Structure that holds the data
	for(int i=0;i<width;i++)
	{
		QBitArray grid(height);
		retval.push_back(grid);
	}
	// Place Laser points in the appropriate grids
	for(int i =0; i<laser_scan.size();i++)
	{
		//Changing to image coordinate
		QPointF p(laser_scan[i].x(),laser_scan[i].y());
		p.setX(( p.x() + res*width /2)/res);
		p.setY((-p.y() + res*height/2)/res);
		retval[int(p.x())][int(p.y())] = true;
	}
	return retval;
}

QVector <QBitArray> MapManager::provideMapOG(QImage image)
{
	QVector <QBitArray> retval;
	for(int i=0;i<image.width();i++)
	{
		QBitArray pixel(image.height());
		QRgb color;
		for(int j=0;j<image.height();j++)
		{
			//qDebug("i:%d j:%d",i,j);
			color = image.pixel(i,j);
			// White color is occupied and black is free
			if ( double(qRed(color) + qGreen(color) + qBlue(color))/3*255.0 > 0.8)
				pixel[j]= true;
		}
		retval.push_back(pixel);
	}
	return retval;
}
