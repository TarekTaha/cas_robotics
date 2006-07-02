#include "MapManager.h"

MapManager::MapManager()
{
}

MapManager::~MapManager()
{
}

QVector <QBitArray> MapManager::provideLaserOG(QVector<QPointF> laser_scan)
{
	QVector <QBitArray> retval;
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
