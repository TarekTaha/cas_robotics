#include "controller.h"

Controller::Controller()
{
}

Controller::~Controller()
{
}

double Controller::deltaFunction(double y1,double v)
{
	const double thetaA = M_PI/4.0f;
	return -sin(v)*thetaA*tanh(y1);
	       
}

/*
 * Path Follower based on "Adaptive, Non-Singular Path-Following Control of Dynamic Wheeled Robots" paper 
 * by D. Soetanto, L. Lapierre, A.Pascoal . Conference of Decision and Control, December 2003.
 */
 
ControlAction Controller::getAction(double angle_current,double angle_ref,int direction,double v,double y1,double s1)
{
	ControlAction cntrl;
	double k1=1,k2=1,lambda=1;
	if(direction == -1)	  angle_current += M_PI;
	if(angle_ref     < 0) angle_ref    += 2*M_PI;
	if(angle_current < 0) angle_current+= 2*M_PI;
	double orientation_error = angle_current - angle_ref;
	if ( orientation_error >  M_PI) orientation_error= (-2*M_PI + orientation_error);
	if ( orientation_error < -M_PI) orientation_error= ( 2*M_PI + orientation_error);	
	cntrl.linear_velocity  = v * cos(orientation_error) + k1*s1;
	cntrl.angular_velocity = 0 -lambda*y1*v*(sin(orientation_error) - sin(deltaFunction(y1,v)))/(orientation_error - deltaFunction(y1,v)) - k2*(orientation_error - deltaFunction(y1,v));
	return cntrl;
}

/* 
 * Linear Controller
 */

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
		cntrl.angular_velocity *= 2;
	}
	else
		cntrl.linear_velocity = speed;
	//qDebug("Ori-Err=[%.3f] Dist-Err=[%.3f] Wdem=[%.3f]",RTOD(orientation_error),displacement,cntrl.angular_velocity);
	return cntrl;
};
