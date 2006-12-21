#include "mapmanager.h"
#include "utils.h"
MapManager::MapManager()
{
}

MapManager::~MapManager()
{
}

Map * MapManager::providePointCloud(LaserScan laserScan, double local_dist,Pose robotPose)
{
	Map *retval;
	double dist=0;
	Pose globalPose = Trans2Global(laserScan.laserPose,robotPose);
	// Creating Map with the right size
	retval = new Map(globalPose);
	for(int i =0; i < laserScan.points.size();i++)
	{
		QPointF p(laserScan.points[i].x(),laserScan.points[i].y());
		p = Trans2Global(p,globalPose);
		dist = Dist(robotPose.p,p);
		if(dist <=local_dist)
		{
			retval->pointCloud.push_back(p);
		}
	}
	return retval;		
}

Map * MapManager::provideLaserOG(LaserScan laserScan, double local_dist,double res,Pose robotPose)
{
	Map *retval;
	double dist=0;
	int height,width;
	// getting right Map dimensions
	qDebug("HERE OG1"); fflush(stdout);	
	height = int(2.0*local_dist/res);
	width =  int(2.0*local_dist/res);	
	Pose globalPose = Trans2Global(laserScan.laserPose,robotPose);
	// Creating Map with the right size
	retval = new Map(width,height,res,QPointF(width/2.0,height/2.0),globalPose);
	for(int i =0; i < laserScan.points.size();i++)
	{
		// Rotate it now, it will reduce the computation later on
		QPointF p(laserScan.points[i].x(),laserScan.points[i].y());
		p = Rotate(p,robotPose.phi);
		p = Trans2Global(p,laserScan.laserPose);
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
	qDebug("Width:%d Height:%d",width,height);
	qDebug("HERE OG1"); fflush(stdout);	
	return retval;
}

Map *MapManager::provideMapOG(QImage image,double res,Pose map_pose,bool negate)
{
	Map * retval;
	QPointF center(image.width()/2.0,image.height()/2.0);
	retval = new Map(image.width(),image.height(),res,center,map_pose);
	long int count=0;
	for(int i=0;i<image.width();i++)
	{
		QRgb color;
		for(int j=0;j<image.height();j++)
		{
			color = image.pixel(i,j);
			double color_ratio = (qRed(color) + qGreen(color) + qBlue(color))/(3.0*255.0);
			if(!negate)
			{
				// White color(255) is Free and Black(0) is Occupied
				if (  color_ratio > 0.9)
					retval->data[i][j]= false;
				else
				{
					retval->data[i][j]= true;
					count++;
				}
			}
			else
			{
				// White color(255) is Occupied and Black(0) is Free
				if ( color_ratio < 0.1)
					retval->data[i][j]= false;
				else 
					retval->data[i][j]= true;				
			}
		}
	}
//	qDebug("Count is:%ld",count);
	return retval;
}
