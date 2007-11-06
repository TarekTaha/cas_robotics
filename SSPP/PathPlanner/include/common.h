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
//------------------------------------------------------------------------
//
//  Name:   Vector2D.h
//
//  Desc:   2D vector struct
//
//  Author: Mat Buckland (fup@ai-junkie.com)
//
//  More Function added by: Tarek Taha (tataha@eng.uts.edu.au)
//------------------------------------------------------------------------
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
struct Vector2D
{
  double x;
  double y;

  Vector2D():x(0.0),y(0.0){}
  Vector2D(double a, double b):x(a),y(b){}

  //sets x and y to zero
  void Zero(){x=0.0; y=0.0;}

  //returns true if both x and y are zero
  bool isZero()const{return (x*x + y*y) < MinDouble;}

  //returns the length of the vector
  inline double    Length()const;

  //returns the squared length of the vector (thereby avoiding the sqrt)
  inline double    LengthSq()const;

  inline void      Normalize();

  inline double    Dot(const Vector2D& v2)const;

  //returns positive if v2 is clockwise of this vector,
  //negative if anticlockwise (assuming the Y axis is pointing down,
  //X axis to right like a Window app)
  inline int       Sign(const Vector2D& v2)const;

  //returns the vector that is perpendicular to this one.
  inline Vector2D  Perp()const;

  //adjusts x and y so that the length of the vector does not exceed max
  inline void      Truncate(double max);

  //returns the distance between this vector and th one passed as a parameter
  inline double    Distance(const Vector2D &v2)const;

  //squared version of above.
  inline double    DistanceSq(const Vector2D &v2)const;

  inline void      Reflect(const Vector2D& norm);

  //returns the vector that is the reverse of this vector
  inline Vector2D  GetReverse()const;


  //we need some overloaded operators
  const Vector2D& operator+=(const Vector2D &rhs)
  {
    x += rhs.x;
    y += rhs.y;

    return *this;
  }

  const Vector2D& operator-=(const Vector2D &rhs)
  {
    x -= rhs.x;
    y -= rhs.y;

    return *this;
  }

  const Vector2D& operator*=(const double& rhs)
  {
    x *= rhs;
    y *= rhs;

    return *this;
  }

  const Vector2D& operator/=(const double& rhs)
  {
    x /= rhs;
    y /= rhs;

    return *this;
  }

  bool operator==(const Vector2D& rhs)const
  {
    return (isEqual(x, rhs.x) && isEqual(y,rhs.y) );
  }

  bool operator!=(const Vector2D& rhs)const
  {
    return (x != rhs.x) || (y != rhs.y);
  }
  
};

//-----------------------------------------------------------------------some more operator overloads
inline Vector2D operator*(const Vector2D &lhs, double rhs);
inline Vector2D operator*(double lhs, const Vector2D &rhs);
inline Vector2D operator-(const Vector2D &lhs, const Vector2D &rhs);
inline Vector2D operator+(const Vector2D &lhs, const Vector2D &rhs);
inline Vector2D operator/(const Vector2D &lhs, double val);
//std::ostream& operator<<(std::ostream& os, const Vector2D& rhs);
//std::ifstream& operator>>(std::ifstream& is, Vector2D& lhs);


//------------------------------------------------------------------------member functions

//------------------------- Length ---------------------------------------
//
//  returns the length of a 2D vector
//------------------------------------------------------------------------
inline double Vector2D::Length()const
{
  return sqrt(x * x + y * y);
}


//------------------------- LengthSq -------------------------------------
//
//  returns the squared length of a 2D vector
//------------------------------------------------------------------------
inline double Vector2D::LengthSq()const
{
  return (x * x + y * y);
}


//------------------------- Vec2DDot -------------------------------------
//
//  calculates the dot product
//------------------------------------------------------------------------
inline double Vector2D::Dot(const Vector2D &v2)const
{
  return x*v2.x + y*v2.y;
}

//------------------------ Sign ------------------------------------------
//
//  returns positive if v2 is clockwise of this vector,
//  minus if anticlockwise (Y axis pointing down, X axis to right)
//------------------------------------------------------------------------
enum {clockwise = 1, anticlockwise = -1};

inline int Vector2D::Sign(const Vector2D& v2)const
{
  if (y*v2.x > x*v2.y)
  { 
    return anticlockwise;
  }
  else 
  {
    return clockwise;
  }
}

//------------------------------ Perp ------------------------------------
//
//  Returns a vector perpendicular to this vector
//------------------------------------------------------------------------
inline Vector2D Vector2D::Perp()const
{
  return Vector2D(-y, x);
}

//------------------------------ Distance --------------------------------
//
//  calculates the euclidean distance between two vectors
//------------------------------------------------------------------------
inline double Vector2D::Distance(const Vector2D &v2)const
{
  double ySeparation = v2.y - y;
  double xSeparation = v2.x - x;

  return sqrt(ySeparation*ySeparation + xSeparation*xSeparation);
}


//------------------------------ DistanceSq ------------------------------
//
//  calculates the euclidean distance squared between two vectors 
//------------------------------------------------------------------------
inline double Vector2D::DistanceSq(const Vector2D &v2)const
{
  double ySeparation = v2.y - y;
  double xSeparation = v2.x - x;

  return ySeparation*ySeparation + xSeparation*xSeparation;
}

//----------------------------- Truncate ---------------------------------
//
//  truncates a vector so that its length does not exceed max
//------------------------------------------------------------------------
inline void Vector2D::Truncate(double max)
{
  if (this->Length() > max)
  {
    this->Normalize();

    *this *= max;
  } 
}

//--------------------------- Reflect ------------------------------------
//
//  given a normalized vector this method reflects the vector it
//  is operating upon. (like the path of a ball bouncing off a wall)
//------------------------------------------------------------------------
inline void Vector2D::Reflect(const Vector2D& norm)
{
  *this += 2.0 * this->Dot(norm) * norm.GetReverse();
}

//----------------------- GetReverse ----------------------------------------
//
//  returns the vector that is the reverse of this vector
//------------------------------------------------------------------------
inline Vector2D Vector2D::GetReverse()const
{
  return Vector2D(-this->x, -this->y);
}


//------------------------- Normalize ------------------------------------
//
//  normalizes a 2D Vector
//------------------------------------------------------------------------
inline void Vector2D::Normalize()
{ 
  double vector_length = this->Length();

  if (vector_length > std::numeric_limits<double>::epsilon())
  {
    this->x /= vector_length;
    this->y /= vector_length;
  }
}


//------------------------------------------------------------------------non member functions

inline Vector2D Vec2DNormalize(const Vector2D &v)
{
  Vector2D vec = v;

  double vector_length = vec.Length();

  if (vector_length > std::numeric_limits<double>::epsilon())
  {
    vec.x /= vector_length;
    vec.y /= vector_length;
  }

  return vec;
}


inline double Vec2DDistance(const Vector2D &v1, const Vector2D &v2)
{

  double ySeparation = v2.y - v1.y;
  double xSeparation = v2.x - v1.x;

  return sqrt(ySeparation*ySeparation + xSeparation*xSeparation);
}

inline double Vec2DDistanceSq(const Vector2D &v1, const Vector2D &v2)
{

  double ySeparation = v2.y - v1.y;
  double xSeparation = v2.x - v1.x;

  return ySeparation*ySeparation + xSeparation*xSeparation;
}

inline double Vec2DLength(const Vector2D& v)
{
  return sqrt(v.x*v.x + v.y*v.y);
}

inline double Vec2DLengthSq(const Vector2D& v)
{
  return (v.x*v.x + v.y*v.y);
}

/*
inline Vector2D POINTStoVector(const POINTS & p)
{
  return Vector2D(p.x, p.y);
}

inline Vector2D POINTtoVector(const POINT& p)
{
  return Vector2D((double)p.x, (double)p.y);
}

inline POINTS VectorToPOINTS(const Vector2D& v)
{
  POINTS p;
  p.x = (short)v.x;
  p.y = (short)v.y;

  return p;
}

inline POINT VectorToPOINT(const Vector2D& v)
{
  POINT p;
  p.x = (long)v.x;
  p.y = (long)v.y;

  return p;
}
*/


//------------------------------------------------------------------------operator overloads
inline Vector2D operator*(const Vector2D &lhs, double rhs)
{
  Vector2D result(lhs);
  result *= rhs;
  return result;
}

inline Vector2D operator*(double lhs, const Vector2D &rhs)
{
  Vector2D result(rhs);
  result *= lhs;
  return result;
}

//overload the - operator
inline Vector2D operator-(const Vector2D &lhs, const Vector2D &rhs)
{
  Vector2D result(lhs);
  result.x -= rhs.x;
  result.y -= rhs.y;
  
  return result;
}

//overload the + operator
inline Vector2D operator+(const Vector2D &lhs, const Vector2D &rhs)
{
  Vector2D result(lhs);
  result.x += rhs.x;
  result.y += rhs.y;
  
  return result;
}

//overload the / operator
inline Vector2D operator/(const Vector2D &lhs, double val)
{
  Vector2D result(lhs);
  result.x /= val;
  result.y /= val;

  return result;
}

///////////////////////////////////////////////////////////////////////////////


//treats a window as a toroid
inline void WrapAround(Vector2D &pos, int MaxX, int MaxY)
{
  if (pos.x > MaxX) {pos.x = 0.0;}

  if (pos.x < 0)    {pos.x = (double)MaxX;}

  if (pos.y < 0)    {pos.y = (double)MaxY;}

  if (pos.y > MaxY) {pos.y = 0.0;}
}

//returns true if the point p is not inside the region defined by top_left
//and bot_rgt
inline bool NotInsideRegion(Vector2D p,
                            Vector2D top_left,
                            Vector2D bot_rgt)
{
  return (p.x < top_left.x) || (p.x > bot_rgt.x) || 
         (p.y < top_left.y) || (p.y > bot_rgt.y);
}

inline bool InsideRegion(Vector2D p,
                         Vector2D top_left,
                         Vector2D bot_rgt)
{
  return !((p.x < top_left.x) || (p.x > bot_rgt.x) || 
         (p.y < top_left.y) || (p.y > bot_rgt.y));
}

inline bool InsideRegion(Vector2D p, int left, int top, int right, int bottom)
{
  return !( (p.x < left) || (p.x > right) || (p.y < top) || (p.y > bottom) );
}

//------------------ isSecondInFOVOfFirst -------------------------------------
//
//  returns true if the target position is in the field of view of the entity
//  positioned at posFirst facing in facingFirst
//-----------------------------------------------------------------------------
inline bool isSecondInFOVOfFirst(Vector2D posFirst,
                                 Vector2D facingFirst,
                                 Vector2D posSecond,
                                 double    fov)
{
  Vector2D toTarget = Vec2DNormalize(posSecond - posFirst);

  return facingFirst.Dot(toTarget) >= cos(fov/2.0);
}
//------------------ Distances  / Intersections and Lines----------------------
//  By: Tarek Taha
// Informtion gathered from: 
//				http://astronomy.swin.edu.au/~pbourke/geometry/pointline/
//-----------------------------------------------------------------------------
inline double Magnitude( Point p1, Point p2 )
{
    Point V;
    V = p2 - p1;
    return sqrt( V.x() * V.x() + V.y() * V.y());
};

/* This Function takes 2 line and returne the interstection Point
 * of these two lines. It can also be used for 2 line segments.
 */
inline bool LineInterLine(Line L1,Line L2,Point &P)
{
	double Ua,Ub;
	double denominator;
	denominator = (
					(L2.end.y() - L2.start.y())*(L1.end.x() - L1.start.x()) 
					-
				  	(L2.end.x() - L2.start.x())*(L1.end.y() - L1.start.y())
				  );
	Ua = (
			(L2.end.x() - L2.start.x())*(L1.start.y() - L2.start.y()) -
			(L2.end.y() - L2.start.y())*(L1.start.x() - L2.start.x())
		 )
		 /
		 denominator;
	Ub = (
			(L1.end.x() - L1.start.x())*(L1.start.y() - L2.start.y()) -
			(L1.end.y() - L1.start.y())*(L1.start.x() - L2.start.x())
		 )
		 /
		 denominator;
	// Is the intersection point outside any of the segments?
    if( Ua < 0.0f || Ua > 1.0f  || Ub < 0.0f || Ub > 1.0f)
    	return false;
    P.setX(L1.start.x() + Ua*(L1.end.x() - L1.start.x()));
    P.setY(L1.start.y() + Ua*(L1.end.y() - L1.start.y()));
    return true;
}

inline double DistanceToLine( Point LineStart, Point LineEnd, Point P)
{
	double LineMag,distance;
	LineMag = Magnitude(LineStart,LineEnd);
	distance = (P.x()*(LineStart.y() - LineEnd.y()) + P.y()*(LineEnd.x() - LineStart.x())
				+(LineStart.x()*LineEnd.y() - LineEnd.x()*LineStart.y()))/LineMag ;
    return distance;
}

//inline double DistToLineSegment(Point LineStart, Point LineEnd, Point p)
//{
//	Vector2D A(LineStart.x(),LineStart.y()),B(LineEnd.x(),LineEnd.y()),P(p.x(),p.y());
//  	//if the angle between PA and AB is obtuse then the closest vertex must be A
//  	double dotA = (P.x - A.x)*(B.x - A.x) + (P.y - A.y)*(B.y - A.y);
//  	if (dotA <= 0) 
//		return Vec2DDistance(A, P);
//	//if the angle between PB and AB is obtuse then the closest vertex must be B
//  	double dotB = (P.x - B.x)*(A.x - B.x) + (P.y - B.y)*(A.y - B.y);
//   	if (dotB <= 0) 
//		return Vec2DDistance(B, P);
//   	//calculate the point along AB that is the closest to P
//  	//Vector2D Point = A + ((B - A) * dotA)/(dotA + dotB);
//	//calculate the distance P-Point
//  	//return Vec2DDistance(P,Point);
//	return DistanceToLine(LineStart,LineEnd, p); 
//}

inline double Dist2Seg(Line line, Point P)
{
    double LineMag;
    double U;
    Point Intersection;
 
    LineMag = line.LineMag();
 
    U = ((( P.x() - line.start.x() ) * ( line.end.x() - line.start.x() )) +
        ((  P.y() - line.start.y() ) * ( line.end.y() - line.start.y() )))
        /( LineMag * LineMag );

	if( U < 0.0f || U > 1.0f) 
		return MIN(Dist(line.start,P),Dist(line.end,P));
    Intersection.setX(line.start.x() + U * ( line.end.x() - line.start.x() ));
    Intersection.setY(line.start.y() + U * ( line.end.y() - line.start.y() ));
    return Magnitude( P, Intersection );
}
#endif /*COMMON_H_*/
