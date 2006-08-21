#include "Controller.h"

Controller::Controller()
{
}

Controller::~Controller()
{
}
//Model2DRigidDiff(string path = ""):Model2DRigid(path) {
//  double alpha;
//
//  StateDim = 3;
//  InputDim = 2;
//
//  MaxSteeringAngle = PI/12.0;
//  MaxSteeringAngle = PI/2.0;
//  CarLength = 2.0;
//
//   Make the list of Inputs
//  Inputs.clear();  // Otherwise its parent constructor will make some inputs
//  for (alpha = -MaxSteeringAngle; alpha <= MaxSteeringAngle; 
//       alpha += 2.0*MaxSteeringAngle/6.0) {
//    Inputs.push_back(MSLVector(1.0,alpha)); 
//    Inputs.push_back(MSLVector(-1.0,alpha)); 
//  }
//
//  READ_OPTIONAL_PARAMETER(Inputs);
//
//}
//
//
//Model2DRigidDiff::StateTransitionEquation(const MSLVector &x, const MSLVector &u) {
//
//  MSLVector dx(3);
//  dx[0] = u[0]*cos(x[2]);
//  dx[1] = u[0]*sin(x[2]);
//  dx[2] = u[0]*tan(u[1])/CarLength;
//  return dx;
//}

ControlAction Controller::getAction(double angle_current,double angle_ref,double displacement,int direction,double speed)
{
	ControlAction cntrl;
	if(direction == -1)	  angle_current += M_PI;
	if(angle_ref     < 0) angle_ref    += 2*M_PI;
	if(angle_current < 0) angle_current+= 2*M_PI;
	double orientation_error = angle_current - angle_ref;
	if ( orientation_error >  M_PI) orientation_error= (-2*M_PI + orientation_error);
	if ( orientation_error < -M_PI) orientation_error= ( 2*M_PI + orientation_error);
	cntrl.angular_velocity = (-k_dist*speed*displacement - k_theta*speed*orientation_error);
	if (Abs(orientation_error)>DTOR(15))
	{
		cntrl.linear_velocity  = 0;
//		cntrl.angular_velocity *= 4;
	}
	else
		cntrl.linear_velocity = speed;
	//qDebug("Ori-Err=[%.3f] Dist-Err=[%.3f] Wdem=[%.3f]",RTOD(orientation_error),displacement,cntrl.angular_velocity);
	return cntrl;
};
