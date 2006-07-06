#include "MapManager.h"
#include "utils.h"
MapManager::MapManager()
{
}

MapManager::~MapManager()
{
}

Map  MapManager::provideLaserOG(QVector<QPointF> laser_scan)
{
	double dist=0,max_range=0,res=0.05;
	int height,width;
	for(int i =0; i<laser_scan.size();i++)
	{
		dist = Dist(QPointF(0,0),laser_scan[i]);
		if (dist >max_range)
			max_range = dist;
	}
	height = width = int(2.0*max_range/res);
	qDebug("Width:%d Height:%d",width,height);
	QBitArray * data = new QBitArray[width];		
	// Create Data Structure that holds the data
	for(int i=0;i<width;i++)
	{
		data[i].resize(height);
	}
	// Place Laser points in the appropriate grids
	for(int i =0; i<laser_scan.size();i++)
	{
		//Changing to image coordinate
		QPointF p(laser_scan[i].x(),laser_scan[i].y());
		p.setX(( p.x() + res*width /2)/res);
		p.setY((-p.y() + res*height/2)/res);
		//qDebug("Pixel X:%d Y:%d",int(p.x()),int(p.y()));		
		data[int(p.x())][int(p.y())] = true;
	}

	Map retval= Map(width,height,0.05,data);
	return retval;
}

Map  MapManager::provideMapOG(QImage image)
{
	QBitArray * data = new QBitArray[image.width()];
	for(int i=0;i<image.width();i++)
	{
		data[i].resize(image.height());
		QRgb color;
		for(int j=0;j<image.height();j++)
		{
			//qDebug("i:%d j:%d",i,j);
			color = image.pixel(i,j);
			// White color is occupied and black is free
			if ( double(qRed(color) + qGreen(color) + qBlue(color))/3*255.0 > 0.8)
				data[i][j]= true;
			else 
				data[i][j]= false;
		}
	}
	Map retval(image.width(),image.height(),0.05,data);
	qDebug("Retval H:%d W:%d",retval.width,retval.height);
	return retval;
}
