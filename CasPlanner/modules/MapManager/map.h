#ifndef MAP_H_
#define MAP_H_
#include "utils.h"
#include <QByteArray>

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
        Map(int width, int height,float mapRes,QPointF center,Pose p);   
        void scale(int newWidth,int newHeight);      
        Map(Pose p);
        Map(float mapRes,Pose p);
        Map(int width, int height, double resolution,  QByteArray rawData);
        Map();
        ~Map();
		// transfers from pixel coordinate to the main coordinate system
		void convertPix(QPointF  *p); 
		// transfers from main coordinate to the pixel coordinate system
		void convert2Pix(QPointF *p);
};
#endif /*MAP_H_*/
