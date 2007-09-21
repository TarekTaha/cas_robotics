/***************************************************************************
 *   Copyright (C) 2007 by Tarek Taha                                      *
 *   tataha@eng.uts.edu.au                                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef COMMON_H_
#define COMMON_H_

#include <sstream>
#include <string>
#include <vector>
#include <limits>
#include <cassert>
#include <iomanip>
#include <math.h>

////////////////////////////////////////////////////////////////////////////////
//                                 Maths stuff
////////////////////////////////////////////////////////////////////////////////

const int      MaxInt    = (std::numeric_limits<int>::max)();
const double   MaxDouble = (std::numeric_limits<double>::max)();
const double   MinDouble = (std::numeric_limits<double>::min)();
const float    MaxFloat  = (std::numeric_limits<float>::max)();
const float    MinFloat  = (std::numeric_limits<float>::min)();
const long int MaxLong  =  (std::numeric_limits<long int>::max)();

#define FORWARD 1
#define BACKWARD -1
#ifndef MAX
	#define MAX(x, y) ((x) > (y) ? (x) : (y))
#endif	
/*
 * Min
 * Return the minimum of two numbers.
 */
#ifndef MIN
	#define MIN(x, y) ((x) < (y) ? (x) : (y))
#endif
/*
 * Abs
 * Return the absolute value of the argument.
 */
#ifndef Abs 
	#define Abs(x) ((x) >= 0 ? (x) : -(x))
#endif	
//
/* This function takes two angles in radians
 * and returns the smallest angle between them in radians
 */
 
#ifndef M_PI
	#define M_PI        3.14159265358979323846
#endif

// Convert radians to degrees
#define RTOD(r) ((r) * 180 / M_PI)

// Convert degrees to radians
#define DTOR(d) ((d) * M_PI / 180)

// Normalize angle to domain -pi, pi
#define NORMALIZE(z) atan2(sin(z), cos(z))

//compares two real numbers. Returns true if they are equal
inline bool isEqual(float a, float b)
{
  if (fabs(a-b) < 1E-12)
  {
    return true;
  }

  return false;
}

inline bool isEqual(double a, double b)
{
  if (fabs(a-b) < 1E-8)
  {
    return true;
  }

  return false;
}

class Point
{
public:
	//! Constructor of the Point Class
    Point();
    //! Constructor of the Point Class
    Point(const Point &p);
    //! Constructor of the Point Class
    Point(double xpos, double ypos);

    double x() const;
    double y() const;
    void setX(double x);
    void setY(double y);

    double &rx();
    double &ry();

    Point &operator+=(const Point &p);
    Point &operator-=(const Point &p);
    Point &operator*=(double c);
    Point &operator/=(double c);

    friend inline bool operator==(const Point &, const Point &);
    friend inline bool operator!=(const Point &, const Point &);
    friend inline const Point operator+(const Point &, const Point &);
    friend inline const Point operator-(const Point &, const Point &);
    friend inline const Point operator*(double, const Point &);
    friend inline const Point operator*(const Point &, double);
    friend inline const Point operator-(const Point &);
    friend inline const Point operator/(const Point &, double);
	//! x Location
    double xp;
    //! y location
    double yp;
};

inline Point::Point() : xp(0), yp(0) { }

inline Point::Point(double xpos, double ypos) : xp(xpos), yp(ypos) { }

inline Point::Point(const Point &p) : xp(p.x()), yp(p.y()) { }
//! Accessor Method to the point's x
inline double Point::x() const
{
    return xp;
}
//! Accessor Method to the point's y
inline double Point::y() const
{
    return yp;
}
//! Set's the value of the Point's x
inline void Point::setX(double xpos)
{
    xp = xpos;
}
//! Set's the value of the Point's y
inline void Point::setY(double ypos)
{
    yp = ypos;
}
//! x reference accessor
inline double &Point::rx()
{
    return xp;
}
//! y reference accessor
inline double &Point::ry()
{
    return yp;
}
//! += Operator override.
inline Point &Point::operator+=(const Point &p)
{
    xp+=p.xp;
    yp+=p.yp;
    return *this;
}
//! -= Operator override.
inline Point &Point::operator-=(const Point &p)
{
    xp-=p.xp; yp-=p.yp; return *this;
}
//! *= Operator override.
inline Point &Point::operator*=(double c)
{
    xp*=c; yp*=c; return *this;
}
//! == Operator override.
inline bool operator==(const Point &p1, const Point &p2)
{
    return isEqual(p1.xp,p2.xp) && isEqual(p1.yp,p2.yp);
}
//! != Operator override.
inline bool operator!=(const Point &p1, const Point &p2)
{
    return !isEqual(p1.xp,p2.xp) || !isEqual(p1.yp,p2.yp);
}
//! + Operator override.
inline const Point operator+(const Point &p1, const Point &p2)
{
    return Point(p1.xp+p2.xp, p1.yp+p2.yp);
}
//! - Operator override.
inline const Point operator-(const Point &p1, const Point &p2)
{
    return Point(p1.xp-p2.xp, p1.yp-p2.yp);
}
//! * Operator override.
inline const Point operator*(const Point &p, double c)
{
    return Point(p.xp*c, p.yp*c);
}
//! * scalar Operator override.
inline const Point operator*(double c, const Point &p)
{
    return Point(p.xp*c, p.yp*c);
}
//! - Operator override.
inline const Point operator-(const Point &p)
{
    return Point(-p.xp, -p.yp);
}
//! /= Operator override.
inline Point &Point::operator/=(double c)
{
    xp/=c;
    yp/=c;
    return *this;
}
//! /= scalar Operator override.
inline const Point operator/(const Point &p, double c)
{
    return Point(p.xp/c, p.yp/c);
}
inline double Dist(Point a, Point b)
{
	return sqrt(pow(a.x() - b.x(),2) + pow(a.y() - b.y(),2));
}
class Line
{
	public:
		//! Start Point of the line
		Point start;
		//! End Point of the line
		Point end;
		//! Constructor
		Line(){};
		//! Constructor
		Line(Point a,Point b): start(a),end(b) {};
		//! Set value of start
		void SetStart(Point a) 
		{
			start = a;
		}
		//! Set value of end
		void SetEnd(Point a) 
		{
			end = a;
		}
		//! returns the line's segment magnitude
		double LineMag()
		{
			Point V = end - start;
			return sqrt(V.x()*V.x() + V.y()*V.y());
		}
};

class Pose
{	
public :
	//! X,Y location of the pose.
	Point p;
	//! Orientation at that pose.
	double phi;	
	//! Pose Constructor
	Pose(){};
	//! Pose Constructor
	Pose(double x ,double y,double theta)
	{
		p.setX(x);
		p.setY(y);
		phi = theta;
	}
//	Pose & operator=(const Pose &ps)
//	{
//		this->p.setX(ps.p.x());
//		this->p.setY(ps.p.y());
//		this->phi = ps.phi;
//		return * this;
//	}
	//! == Operator override
	bool operator==(const Pose& pose) const
  	{
    	return (isEqual(p.x(), pose.p.x()) && isEqual(p.y(),pose.p.y()) && isEqual(phi,pose.phi));
  	}
  	//! != Operator override
  	bool operator!=(const Pose& pose) const
  	{
    	return (p.x()!=pose.p.x() || p.y()!=pose.p.y() || phi != pose.phi);
  	}
};
// computes the signed minimum difference between the two angles.
inline double anglediffs(double a, double b)
{
  double d1, d2;
  a = NORMALIZE(a);
  b = NORMALIZE(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
};

inline double anglediff(double alfa, double beta) 
{
//	double diff;
//  	diff = beta - alfa;
//  	if (diff > M_PI)
//  	{
//    	diff -= 2 * M_PI;
//  	}
//  	else if (diff < -M_PI) 
//  	{
//    	diff += 2 * M_PI;
//  	}	
	double diff;
	if( alfa < 0 ) alfa+= 2*M_PI; 	if( alfa > 2*M_PI) alfa-= 2*M_PI;
	if( beta < 0 ) beta+= 2*M_PI;	if( beta > 2*M_PI) beta-= 2*M_PI;		
	diff = alfa - beta;
	if ( diff >  M_PI) diff=( 2*M_PI  - diff);
	if ( diff < -M_PI) diff=(-2*M_PI - diff);
	return Abs(diff);
};

inline double ATAN2(Point a,Point b)
{
	return atan2(a.y() - b.y(), a.x() - b.x());
}
inline Point Rotate(Point p,double angle)
{
	// Rotate 
	Point temp; temp.setX(p.x()); temp.setY(p.y());
	p.setX(temp.x()*cos(angle) - temp.y()*sin(angle));
	p.setY(temp.x()*sin(angle) + temp.y()*cos(angle));
	return p;
}

inline Point Trans2Global(Point p,Pose pose)
{
	// Rotate + Translate
	Point temp; temp.setX(p.x()); temp.setY(p.y());
	p.setX(temp.x()*cos(pose.phi) - temp.y()*sin(pose.phi) + pose.p.x());
	p.setY(temp.x()*sin(pose.phi) + temp.y()*cos(pose.phi) + pose.p.y());
	return p;
}

inline Pose Trans2Global(Pose p,Pose pose)
{
	// Rotate + Translate
	Point temp; temp.setX(p.p.x()); temp.setY(p.p.y());
	p.p.setX(temp.x()*cos(pose.phi) - temp.y()*sin(pose.phi) + pose.p.x());
	p.p.setY(temp.x()*sin(pose.phi) + temp.y()*cos(pose.phi) + pose.p.y());
	p.phi = NORMALIZE(p.phi + pose.phi);
	return p;
}
#endif /*COMMON_H_*/
