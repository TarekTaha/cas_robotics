#ifndef MAP_H_
#define MAP_H_

class Map 
{
    public: 
        int width; 
        int height;
        double resolution;
        QByteArray rawData; // for OG-Maps
        bool ** data;       // for Planners
        QPointF center;		// Axis Center of the Map
        Map(int width, int height, double resolution,QPointF center)
        {
            this->width   = width; 
            this->height  = height; 
            this->resolution = resolution;
            this->rawData = NULL;
            this->center = center;
			this->data = new bool * [width];
			for(int i=0; i < width; i++)
			{
				data[i] = new bool [height];
				for(int j=0;j < height;j++)
					data[i][j] = false;
			}
        }
        Map(int width, int height, double resolution,  QByteArray rawData)
        {
            this->width   = width; 
            this->height  = height; 
            this->rawData = rawData; 
            this->resolution = resolution;
			this->data = NULL;         
        }        
        Map(): width(0), height(0), resolution(0), rawData(NULL),data(NULL)
        {
            
        }
        ~Map()
        {
        	if(data)
        	{
				for (int i=0; i < width; i++)
				{
					//qDebug("Deleting Row %d",i);
					//fflush(stdout);
		    		delete  [] data[i];
				}
				delete [] data;
				//qDebug("Previous Map Data deleted");
        	}
        }
 
};
#endif /*MAP_H_*/
