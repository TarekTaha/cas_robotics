#include "map.h"

Map::Map(int width, int height,float mapRes,QPointF center,Pose p)
{
	//printf("\n W:%d,H:%d,Res:%f",width,height,mapRes);
	this->global_pose = p;
    this->width   = width;
    this->height  = height;
    this->rawData = NULL;
    this->center = center;
	this->mapRes = mapRes;         
	this->grid = new bool * [width];
	for(int i=0; i < width; i++)
	{
		grid[i] = new bool [height];
		for(int j=0;j < height;j++)
			grid[i][j] = false;
	}
}
/* 
 * When rendering in OpenGL, the image dimensions should be a power of 2
 * and since in many cases our map is not, we have to scale the image to the
 * closest power of 2 , center the original in the new scaled space and fill
 * the rest of the space with empty.
 */
void Map::scale(int newWidth,int newHeight)
{
	this->temp = new bool * [newWidth];
	for(int i=0; i < newWidth; i++)
	{
		temp[i] = new bool [newHeight];
		for(int j=0;j < newHeight;j++)
			temp[i][j] = false;
	}
	// copy the old data to the new scaled map and center it
//	assert(newWidth < this->width || newHeight < this->height);
	int startIndexI =  (int)((newWidth -this->width)/2.0);
	int startIndexJ =  (int)((newHeight-this->height)/2.0);
//	printf("\n Old Scale W=%d H=%d, New W=%d H=%d",this->width,this->height,newWidth,newHeight);
//	printf("\n Start Index I=%d,J=%d",startIndexI,startIndexJ);
	int i=0,j=0,oldI=0,oldJ=0;
	for( i = startIndexI; i < (newWidth - startIndexI); i++)
	{
		oldJ=0;
		for( j = startIndexJ;j < (newHeight - startIndexJ);j++)
		{
			temp[i][j] = grid[oldI][oldJ];
			oldJ++;
		}
		oldI++;
	}
//	printf("\nI=%d,J=%d oldI=%d,oldJ=%d",i,j,oldI,oldJ);
	// remove the old copy
	for (int i=0; i < this->width; i++)
	{
    	delete  [] grid[i];
	}
	delete [] grid;
	// reassign the temp to the grid and change dimensions
	grid = temp;
	this->width  = newWidth;
	this->height = newHeight;
//	printf("\n Map Scaled Properly!!!"); fflush(stdout);
}

Map::Map(Pose p)
{
	this->global_pose = p;
    this->rawData = NULL;
	this->grid = NULL;
}        
Map::Map(float mapRes,Pose p)
{
	this->global_pose = p;
	this->mapRes = mapRes;
    this->rawData = NULL;
	this->grid = NULL;

}        
Map::Map(int width, int height, double resolution,  QByteArray rawData)
{
    this->width   = width; 
    this->height  = height; 
    this->rawData = rawData; 
    this->mapRes = resolution;
	this->grid = NULL;         
}        
Map::Map(): width(0), height(0), mapRes(0), rawData(NULL),grid(NULL)
{
    
}
Map::~Map()
{
	if(grid)
	{
		for (int i=0; i < width; i++)
		{
			//qDebug("Deleting Row %d",i);
			//fflush(stdout);
    		delete  [] grid[i];
		}
		delete [] grid;
		//qDebug("Previous Map Data deleted");
	}
}
// transfers from pixel coordinate to the main coordinate system
void Map::convertPix(QPointF  *p) 
{
	p->setX( p->x()*mapRes - mapRes*center.x());
	p->setY(-p->y()*mapRes + mapRes*center.y());
};
// transfers from main coordinate to the pixel coordinate system
void Map::convert2Pix(QPointF *p)
{
	p->setX(( p->x() + mapRes*center.x())/mapRes);
	p->setY((-p->y() + mapRes*center.y())/mapRes);
}        
