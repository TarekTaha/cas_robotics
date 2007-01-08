#ifndef MAP_H_
#define MAP_H_

class Map 
{
    public:
        int width, height;
        float mapRes;
        Pose global_pose;
        QByteArray rawData; 	// for OG-Maps
        bool    ** grid;        // for Planners
        QVector <QPointF> pointCloud;
        QPointF center;			// Axis Center of the Map
        Map(int width, int height,float mapRes,QPointF center,Pose p)
        {
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
        Map(Pose p)
        {
        	this->global_pose = p;
            this->rawData = NULL;
			this->grid = NULL;
        }        
        Map(float mapRes,Pose p)
        {
        	this->global_pose = p;
        	this->mapRes = mapRes;
            this->rawData = NULL;
			this->grid = NULL;

        }        
        Map(int width, int height, double resolution,  QByteArray rawData)
        {
            this->width   = width; 
            this->height  = height; 
            this->rawData = rawData; 
            this->mapRes = resolution;
			this->grid = NULL;         
        }        
        Map(): width(0), height(0), mapRes(0), rawData(NULL),grid(NULL)
        {
            
        }
        ~Map()
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
 
};
#endif /*MAP_H_*/
