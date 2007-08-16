#ifndef MAP_H_
#define MAP_H_

class Map 
{
    public:
        int width, height;
        float mapRes;
        Pose global_pose;
        QByteArray rawData; 	// for OG-Maps
        bool    ** grid, **temp;        // for Planners
        QVector <QPointF> pointCloud;
        QPointF center;			// Axis Center of the Map
        Map(int width, int height,float mapRes,QPointF center,Pose p)
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
        void scale(int newWidth,int newHeight)
        {
			this->temp = new bool * [newWidth];
			for(int i=0; i < newWidth; i++)
			{
				temp[i] = new bool [newHeight];
				for(int j=0;j < newHeight;j++)
					temp[i][j] = false;
			}        	
			// copy the old data to the new scaled map
			for(int i=0; i < this->width; i++)
			{
				for(int j=0;j < this->height;j++)
				{
					temp[i][j] = grid[i][j];
				}
			}
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
			printf("\n Map Scaled Properly!!!"); fflush(stdout);
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
		// transfers from pixel coordinate to the main coordinate system
		void convertPix(QPointF  *p) 
		{
			p->setX( p->x()*mapRes - mapRes*center.x());
			p->setY(-p->y()*mapRes + mapRes*center.y());
		};
		// transfers from main coordinate to the pixel coordinate system
		void convert2Pix(QPointF *p)
		{
			p->setX(( p->x() + mapRes*center.x())/mapRes);
			p->setY((-p->y() + mapRes*center.y())/mapRes);
		}        
};
#endif /*MAP_H_*/
