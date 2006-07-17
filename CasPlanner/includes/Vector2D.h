#ifndef S2DVECTOR_H
#define S2DVECTOR_H
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
#include <math.h>
#include <iosfwd>
#include <limits>
#include "utils.h"

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
inline double Magnitude( QPointF p1, QPointF p2 )
{
    QPointF Vector;
    Vector = p2 - p1;
    return sqrt( Vector.x() * Vector.x() + Vector.y() * Vector.y());
};

/* This Function takes 2 line and returne the interstection Point
 * of these two lines. It can also be used for 2 line segments.
 */
inline bool LineInterLine(Line L1,Line L2,QPointF &P)
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

inline double DistanceToLine( QPointF LineStart, QPointF LineEnd, QPointF P)
{
	double LineMag,distance;
	LineMag = Magnitude(LineStart,LineEnd);
	distance = (P.x()*(LineStart.y() - LineEnd.y()) + P.y()*(LineEnd.x() - LineStart.x())
				+(LineStart.x()*LineEnd.y() - LineEnd.x()*LineStart.y()))/LineMag ;
    return distance;
}

//inline double DistToLineSegment(QPointF LineStart, QPointF LineEnd, QPointF p)
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

inline double Dist2Seg(Line line, QPointF Point)
{
    double LineMag;
    double U;
    QPointF Intersection;
 
    LineMag = line.LineMag();
 
    U = ((( Point.x() - line.start.x() ) * ( line.end.x() - line.start.x() )) +
        ((  Point.y() - line.start.y() ) * ( line.end.y() - line.start.y() )))
        /( LineMag * LineMag );

	if( U < 0.0f || U > 1.0f) 
		return Min(Dist(line.start,Point),Dist(line.end,Point));
//    if( U < 0.0f)
//        return Dist(line.start,Point);   
//    if( U > 1.0f)
//        return Dist(line.end,Point);   
    Intersection.setX(line.start.x() + U * ( line.end.x() - line.start.x() ));
    Intersection.setY(line.start.y() + U * ( line.end.y() - line.start.y() ));
    return Magnitude( Point, Intersection );
}
#endif
