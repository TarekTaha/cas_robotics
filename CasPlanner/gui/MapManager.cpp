#include "MapManager.h"
#include "utils.h"
MapManager::MapManager()
{
}

MapManager::~MapManager()
{
}

Map * MapManager::provideLaserOG(QVector<QPointF> laser_scan)
{
	Map *retval;
	double dist=0,max_range=0,res=0.05;
	int height,width;
	// getting right Map dimensions
	for(int i =0; i<laser_scan.size();i++)
	{
		dist = Dist(QPointF(0,0),laser_scan[i]);
		if (dist >max_range)
			max_range = dist;
	}
	height = width = int(2.0*max_range/res);
	//qDebug("Width:%d Height:%d",width,height);
	// Creating Map with the right size
	retval = new Map(width,height,res);
	// Place Laser points in the appropriate grids
	for(int i =0; i<laser_scan.size();i++)
	{
		//Changing to image coordinate
		QPointF p(laser_scan[i].x(),laser_scan[i].y());
		p.setX( (( p.x() + res*width /2)/res) );
		p.setY( ((-p.y() + res*height/2)/res) );
//		qDebug("Pixel X:%d Y:%d",int(p.x()),int(p.y()));	
		// Some boundary crossing check
		if(p.x() > (width-1 )) p.setX(width -1);
		if(p.y() > (height-1)) p.setY(height-1);
		if(p.x() < 0) p.setX(0);
		if(p.y() < 0) p.setY(0);
		// Assigning value to occupancy grid
		retval->data[int(p.x())][int(p.y())] = true;
	}
	return retval;
}

Map *MapManager::provideMapOG(QImage image)
{
	Map * retval;
	double res = 0.05;	
	retval = new Map(image.width(),image.height(),res);
	for(int i=0;i<image.width();i++)
	{
		QRgb color;
		for(int j=0;j<image.height();j++)
		{
			color = image.pixel(i,j);
			// White color is occupied and black is free
			if ( double(qRed(color) + qGreen(color) + qBlue(color))/3*255.0 > 0.8)
				retval->data[i][j]= true;
			else 
				retval->data[i][j]= false;
		}
	}
	return retval;
}
