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
	void setXYZ(float x ,float y,float z)
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
class Triangle
{
	public:	
		Point p1,p2,p3;
		Triangle()
		{
			p1.setXYZ(0,0,0);
			p2.setXYZ(0,0,0);			
			p3.setXYZ(0,0,0);			
		};
		Triangle(Point p1,Point p2, Point p3)
		{
			this->p1 = p1;
			this->p2 = p2;
			this->p3 = p3;
		}
		void setPoints(Point p1,Point p2, Point p3)
		{
			this->p1 = p1;
			this->p2 = p2;
			this->p3 = p3;
		}
};

class LaserScan
{	
public : 
	vector <Point>	points;
	LaserScan(){};
	~LaserScan()
	{
		points.clear();
	}
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
		vector <Triangle> triangles;	
		float normals[400000][3];
		void clear();
	public :
		int numScans;
		int pointsPerScan;
		bool scanning;
		float lastscan[1024];
		int lastScanCount;
		int numtri;	
};

#endif /*LASERBUFFER_H_*/
