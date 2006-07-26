#include "MapManager.h"
#include "utils.h"
MapManager::MapManager()
{
}

MapManager::~MapManager()
{
}

Map * MapManager::provideLaserOG(QVector<QPointF> laser_scan, double local_dist,Pose pose)
{
	
	Map *retval;
	double dist=0,res=0.05;
	int height,width;
	// getting right Map dimensions
	height = int(2.0*local_dist/res);
	width =  int(2.0*local_dist/res);	
	// Creating Map with the right size
	retval = new Map(width,height,res,QPointF(width/2.0,height/2.0));
	for(int i =0; i < laser_scan.size();i++)
	{
		// Rotate it now, it will reduce the computation later on
		QPointF p(laser_scan[i].x(),laser_scan[i].y());
		p = Rotate(p,pose.phi);
		dist = Dist(QPointF(0,0),p);
//		assert( p.x() > 0 );
		//qDebug("Metric X:%f Y:%f dist=%f",p.x(),p.y(),dist);		
		// Changing to Pixel	
		p.setX( (( p.x() + res*width/2)/res) );
		p.setY( ((-p.y() + res*height/2)/res) );
		//qDebug("Pixel X:%d Y:%d",int(p.x()),int(p.y()));	
		// Some boundary crossing check
		if(p.x() > (width-1 )) p.setX(width -1);
		if(p.y() > (height-1)) p.setY(height-1);
		if(p.x() < 0) p.setX(0);
		if(p.y() < 0) p.setY(0);
		if (dist <= local_dist)
		{
			retval->data[int(p.x())][int(p.y())] = true;
		}
		else
		{
			retval->data[int(p.x())][int(p.y())] = false;			
		}
	}
	//qDebug("Width:%d Height:%d",width,height);
	return retval;
}

Map *MapManager::provideMapOG(QImage image)
{
	Map * retval;
	double res = 0.05;	
	bool negate=false;
	QPointF center(image.width()/2.0,image.height()/2.0);
	retval = new Map(image.width(),image.height(),res,center);
	for(int i=0;i<image.width();i++)
	{
		QRgb color;
		for(int j=0;j<image.height();j++)
		{
			color = image.pixel(i,j);
			if(negate)
			{
			// White color is occupied and black is free
			if ( double(qRed(color) + qGreen(color) + qBlue(color))/3*255.0 > 0.8)
				retval->data[i][j]= true;
			else 
				retval->data[i][j]= false;
			}
			else
			{
			// White color is occupied and black is free
			if ( double(qRed(color) + qGreen(color) + qBlue(color))/3*255.0 < 0.2)
				retval->data[i][j]= true;
			else 
				retval->data[i][j]= false;				
			}
		}
	}
	return retval;
}
