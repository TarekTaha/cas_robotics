#ifndef LASERBUFFER_H_
#define LASERBUFFER_H_
#include <vector>

using namespace std;
class Point
{	
public : 
	float x;
	float y;
	float z;	
	Point()
	{
		this->x = this->y = this->z =0;
	};
	Point(float x ,float y,float z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}
	Point & operator=(const Point &ps)
	{
		this->x = ps.x;
		this->y = ps.y;
		this->z = ps.z;
		return * this;
	}
	bool operator==(const Point& point) const
  	{
    	return ((this->x== point.x) && (this->y==point.y) && (this->z==point.z));
  	}
};
class LaserScan
{	
public : 
	vector <Point>	points;
	LaserScan(){};
	LaserScan(int laserMaxPoints)
	{
		points.reserve(laserMaxPoints);
	}
};
class LaserBuffer
{
	public:
		LaserBuffer();
		LaserBuffer(int servoMaxPoints, int laserMaxPoints);
		virtual ~LaserBuffer();
		vector <LaserScan>	scans;
	private :
		int numscans;
		int pointsperscan;
		bool scanning;
		float lastscan[1024];
		int lastscan_count;
		//float triangles[400000][9];
		vector< vector<float> > triangles;
		float normals[400000][3];
		//vector< vector<float> > normals;
		int numtri;	
};

#endif /*LASERBUFFER_H_*/
