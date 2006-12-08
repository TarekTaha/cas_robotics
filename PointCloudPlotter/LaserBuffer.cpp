#include "LaserBuffer.h"

LaserBuffer::LaserBuffer()
{
}

LaserBuffer::~LaserBuffer()
{
}

LaserBuffer::LaserBuffer(int servoPoints, int laserPoints)
{
	this->numScans = servoPoints;
	this->pointsPerScan = laserPoints;
	scans.assign(servoPoints,laserPoints);
	triangles.reserve(400000);
}

void LaserBuffer::clear()
{
	for(int i=0;i<scans.size();i++)
		for(int j=0;j<scans[i].points.size();j++)
		{
			scans[i].points[j].setXYZ(0,0,0);
		}
		numScans=0;
		pointsPerScan=0;
		numtri =0;
		lastScanCount=0;		
}

LaserBuffer::~LaserBuffer()
{
	triangles.clear();
	scans.clear();
}
