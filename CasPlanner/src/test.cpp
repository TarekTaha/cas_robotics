#include <stdio.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <iostream.h>

const int curvefittingorder = 6;
const double Gapdist = 0.5;
const int NPOL = 4;
const double INF = 1E200;
const double EP = 1E-10;
const double PI = 3.14159265359;

//robot parameters
  /// RX , RY robot X and Y position
  /// RR  Robot Radius
  /// RS  Current Speed
  /// RD Current Orientation (rd)
  /// RM Mass kg
  /// RJ Moment of Inirtia
  /// Omega Current Rotation Speed (rd/sec)
  //system parameters
  /// TimeStep in seconds
  /// SysK fixed par
  /// SysC fixed par
  /// SysFR Dmin/Dmax fixed
  /// SysP Repulsive  Force Fixed
  /// SysQ Attractive Force Fixed
  /// OmegadotMax Max rotational acc
  /// OmegaMax Max rotationl speed  


struct POINT
{
 double x;
 double y; //POINT(double a=0, double b=0) { x=a; y=b;} //constructor
};
struct LINESEG
{
 POINT s;
 POINT e; //LINESEG(POINT a, POINT b) { s=a; e=b;}
 LINESEG() { }}
;

// RC: the location of robot's center. RC.x: x location. RC.y: y locatotion
main()
{
 //double PtoRobotDist(POINT [], POINT);
 double FindR(POINT [],POINT);
 void ForceField(double [], double [], char [20], POINT[]);

 char Filename[20] = "Book16.txt";

 double RS = 0.06, RD = -PI/2, RM = 1, RJ = 1, GoalX = 10, GoalY = 10, Omega = 0;
 double TimeStep = 0.01, SysK = 5.0, SysC = 1.25, SysFR = 0.2, SysP = 10, SysQ = 10, MaxSpeed = 0.08, MaxAcceT = 1, OmegadotMax = 1, OmegaMax = 1;

 POINT RC = {-3,-3}; //Robot's center
 POINT OffsetRC = {-0.3, 0}; // the offset of robot's rotation center to robot's center
 POINT OffsetV[NPOL] = {{-0.3, -0.325}, {0.9, -0.325}, {0.9, 0.325}, {-0.3, 0.325}}; // offset of robot's vertexs

 POINT RCR = {RC.x + (OffsetRC.x * cos(RD) - OffsetRC.y * sin(RD)),RC.y + (OffsetRC.x * sin(RD) + OffsetRC.y * cos(RD))}; //Robot's center of rotation

 POINT RobotVertex[NPOL + 1];

 for (int i = 0; i <= NPOL - 1; i++)
 {
   RobotVertex[i].x = RC.x + (OffsetV[i].x * cos(RD) - OffsetV[i].y * sin(RD));
   RobotVertex[i].y = RC.y + (OffsetV[i].x * sin(RD) + OffsetV[i].y * cos(RD));
 }
 RobotVertex[NPOL] = RobotVertex[0];

 double RR = FindR(RobotVertex, RCR);
 //printf ("R=%f\n",RR);

 double RobInfo[] = {RC.x, RC.y, RR, RS, RD, RM, RJ, GoalX, GoalY, Omega, RCR.x, RCR.y};
 double SysInfo[] = {TimeStep, SysK, SysC, SysFR, SysP, SysQ, MaxSpeed, MaxAcceT, OmegadotMax, OmegaMax};

 printf ("Before\n");
 printf ("RS = %f, RD = %f, Omega = %f\n", RobInfo[3], RobInfo[4], RobInfo[9]);

 ForceField(RobInfo, SysInfo, Filename, RobotVertex);
 printf ("After\n");

 printf ("RS = %f, RD = %f, Omega = %f\n", RobInfo[3], RobInfo[4], RobInfo[9]);
}

