#ifndef UTILS_H
#define UTILS_H
//------------------------------------------------------------------------
//
//  Name: utils.h
//
//  Desc: misc utility functions and constants
//
//  Author: Mat Buckland (fup@ai-junkie.com)
//
//  More Function added by: Tarek Taha (tataha@eng.uts.edu.au)
//------------------------------------------------------------------------
#include <sstream>
#include <string>
#include <vector>
#include <limits>
#include <cassert>
#include <iomanip>
#include <math.h>
#include <QPointF>
#include <QVector>

//a few useful constants
const int     MaxInt    = (std::numeric_limits<int>::max)();
const double  MaxDouble = (std::numeric_limits<double>::max)();
const double  MinDouble = (std::numeric_limits<double>::min)();
const float   MaxFloat  = (std::numeric_limits<float>::max)();
const float   MinFloat  = (std::numeric_limits<float>::min)();

const double   Pi        = 3.14159;
const double   TwoPi     = Pi * 2;
const double   HalfPi    = Pi / 2;
const double   QuarterPi = Pi / 4;

//returns true if the value is a NaN
template <typename T>
inline bool isNaN(T val)
{
  return val != val;
}

inline double DegsToRads(double degs)
{
  return TwoPi * (degs/360.0);
}



//returns true if the parameter is equal to zero
inline bool IsZero(double val)
{
  return ( (-MinDouble < val) && (val < MinDouble) );
}

//returns true is the third parameter is in the range described by the
//first two
inline bool InRange(double start, double end, double val)
{
  if (start < end)
  {
    if ( (val > start) && (val < end) ) return true;
    else return false;
  }

  else
  {
    if ( (val < start) && (val > end) ) return true;
    else return false;
  }
}

template <class T>
T Maximum(const T& v1, const T& v2)
{
  return v1 > v2 ? v1 : v2;
}



//----------------------------------------------------------------------------
//  some random number functions.
//----------------------------------------------------------------------------

//returns a random integer between x and y
inline int   RandInt(int x,int y) {return rand()%(y-x+1)+x;}

//returns a random double between zero and 1
inline double RandFloat()      {return ((rand())/(RAND_MAX+1.0));}

inline double RandInRange(double x, double y)
{
  return x + RandFloat()*(y-x);
}

//returns a random bool
inline bool   RandBool()
{
  if (RandInt(0,1)) return true;

  else return false;
}

//returns a random double in the range -1 < n < 1
inline double RandomClamped()    {return RandFloat() - RandFloat();}


//returns a random number with a normal distribution. See method at
//http://www.taygeta.com/random/gaussian.html
inline double RandGaussian(double mean = 0.0, double standard_deviation = 1.0)
{				        
	double x1, x2, w, y1;
	static double y2;
	static int use_last = 0;

	if (use_last)		        /* use value from previous call */
	{
		y1 = y2;
		use_last = 0;
	}
	else
	{
		do 
    {
			x1 = 2.0 * RandFloat() - 1.0;
			x2 = 2.0 * RandFloat() - 1.0;
			w = x1 * x1 + x2 * x2;
		}
    while ( w >= 1.0 );

		w = sqrt( (-2.0 * log( w ) ) / w );
		y1 = x1 * w;
		y2 = x2 * w;
		use_last = 1;
	}

	return( mean + y1 * standard_deviation );
}



//-----------------------------------------------------------------------
//  
//  some handy little functions
//-----------------------------------------------------------------------


inline double Sigmoid(double input, double response = 1.0)
{
	return ( 1.0 / ( 1.0 + exp(-input / response)));
}


//returns the maximum of two values
template <class T>
inline T MaxOf(const T& a, const T& b)
{
  if (a>b) return a; return b;
}

//returns the minimum of two values
template <class T>
inline T MinOf(const T& a, const T& b)
{
  if (a<b) return a; return b;
}


//clamps the first argument between the second two
template <class T, class U, class V>
inline void Clamp(T& arg, const U& minVal, const V& maxVal)
{
  assert ( (minVal < maxVal) && "<Clamp>MaxVal < MinVal!");

  if (arg < (T)minVal)
  {
    arg = (T)minVal;
  }

  if (arg > (T)maxVal)
  {
    arg = (T)maxVal;
  }
}


//rounds a double up or down depending on its value
inline int Rounded(double val)
{
  int    integral = (int)val;
  double mantissa = val - integral;

  if (mantissa < 0.5)
  {
    return integral;
  }

  else
  {
    return integral + 1;
  }
}

//rounds a double up or down depending on whether its 
//mantissa is higher or lower than offset
inline int RoundUnderOffset(double val, double offset)
{
  int    integral = (int)val;
  double mantissa = val - integral;

  if (mantissa < offset)
  {
    return integral;
  }

  else
  {
    return integral + 1;
  }
}

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
  if (fabs(a-b) < 1E-12)
  {
    return true;
  }

  return false;
}


template <class T>
inline double Average(const std::vector<T>& v)
{
  double average = 0.0;
  
  for (unsigned int i=0; i < v.size(); ++i)
  {    
    average += (double)v[i];
  }

  return average / (double)v.size();
}


inline double StandardDeviation(const std::vector<double>& v)
{
  double sd      = 0.0;
  double average = Average(v);

  for (unsigned int i=0; i<v.size(); ++i)
  {     
    sd += (v[i] - average) * (v[i] - average);
  }

  sd = sd / v.size();

  return sqrt(sd);
}

/*
template <class container>
inline void DeleteSTLContainer(container& c)
{
  for (container::iterator it = c.begin(); it!=c.end(); ++it)
  {
    delete *it;
    *it = NULL;
  }
}

template <class map>
inline void DeleteSTLMap(map& m)
{
  for (map::iterator it = m.begin(); it!=m.end(); ++it)
  {
    delete it->second;
    it->second = NULL;
  }
}
*/
////////////////////////////////////////////////////////////////////////////////
//                                 Maths stuff
////////////////////////////////////////////////////////////////////////////////
#define FORWARD 1
#define BACKWARD -1

#define Max(x, y) ((x) > (y) ? (x) : (y))
/*
 * Min
 * Return the minimum of two numbers.
 */
#define Min(x, y) ((x) < (y) ? (x) : (y))
/*
 * Abs
 * Return the absolute value of the argument.
 */
#define Abs(x) ((x) >= 0 ? (x) : -(x))
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

inline double Dist(QPointF a, QPointF b)
{
	return sqrt(pow(a.x() - b.x(),2) + pow(a.y() - b.y(),2));
}
class Line
{
	public:
		QPointF start,end;
		Line(){};
		Line(QPointF a,QPointF b): start(a),end(b) {};
		void SetStart(QPointF a) 
		{
			start = a;
		}
		void SetEnd(QPointF a) 
		{
			end = a;
		}
		double LineMag()
		{
			QPointF V = end - start;
			return sqrt(V.x()*V.x() + V.y()*V.y());
		}
};

class Pose
{	
public : 
	QPointF p;
	double phi;	
	Pose(){};
	Pose(double x ,double y,double theta)
	{
		p.setX(x);
		p.setY(y);
		phi = theta;
	}
	bool operator==(const Pose& pose) const
  	{
    	return (isEqual(p.x(), pose.p.x()) && isEqual(p.y(),pose.p.y()) && isEqual(phi,pose.phi));
  	}
  	bool operator!=(const Pose& pose) const
  	{
    	return (p.x()!=pose.p.x() || p.y()!=pose.p.y() || phi != pose.phi);
  	}
};
//typedef struct _pose
//{
//	QPointF p;
//	double phi;
//} Pose;

inline double anglediff(double alfa, double beta) 
{
	double diff;
	if( alfa < 0 ) alfa+= 2*M_PI; 	if( alfa > 2*M_PI) alfa-= 2*M_PI;
	if( beta < 0 ) beta+= 2*M_PI;	if( beta > 2*M_PI) beta-= 2*M_PI;		
	diff = alfa - beta;
	if ( diff >  M_PI) diff=( 2*M_PI  - diff);
	if ( diff < -M_PI) diff=(-2*M_PI - diff);
	return Abs(diff);
};

inline double ATAN2(QPointF a,QPointF b)
{
	return atan2(a.y() - b.y(), a.x() - b.x());
}

inline QPointF Rotate(QPointF p,double angle)
{
	// Rotate 
	QPointF temp; temp.setX(p.x()); temp.setY(p.y());
	p.setX(temp.x()*cos(angle) - temp.y()*sin(angle));
	p.setY(temp.x()*sin(angle) + temp.y()*cos(angle));
	return p;
}

inline QPointF Trans2Global(QPointF p,Pose pose)
{
	// Rotate + Translate
	QPointF temp; temp.setX(p.x()); temp.setY(p.y());
	p.setX(temp.x()*cos(pose.phi) - temp.y()*sin(pose.phi) + pose.p.x());
	p.setY(temp.x()*sin(pose.phi) + temp.y()*cos(pose.phi) + pose.p.y());
	return p;
}

inline Pose Trans2Global(Pose p,Pose pose)
{
	// Rotate + Translate
	QPointF temp; temp.setX(p.p.x()); temp.setY(p.p.y());
	p.p.setX(temp.x()*cos(pose.phi) - temp.y()*sin(pose.phi) + pose.p.x());
	p.p.setY(temp.x()*sin(pose.phi) + temp.y()*cos(pose.phi) + pose.p.y());
	p.phi = NORMALIZE(p.phi + pose.phi);
	return p;
}

inline void MatrixMultipy(double a[2][2], double b[2], double c[2])
{
	c[0] = a[0][0] * b[0] + a[0][1] * b[1];
	c[1] = a[1][0] * b[0] + a[1][1] * b[1];
 	return;
}

inline double DotMultiply(QPointF p1,QPointF p2,QPointF p0)
{
	return ((p1.x()-p0.x())*(p2.x()-p0.x())+(p1.y()-p0.y())*(p2.y()-p0.y()));
}
/*****************************************************************/
typedef struct _tree
	{
		QPointF location;
		QVector <QPointF> children;
	}   Tree;
enum {LOCALPATH,GLOBALPATH,BOTHPATHS};
#endif
