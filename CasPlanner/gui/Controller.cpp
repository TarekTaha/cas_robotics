#include "Controller.h"

Controller::Controller()
{
}

Controller::~Controller()
{
}

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
		cntrl.linear_velocity = 0;
		//cntrl.angular_velocity *=2;
	}
	else
		cntrl.linear_velocity = speed;
	qDebug("Ori-Err=[%.3f] Dist-Err=[%.3f] Wdem=[%.3f]",RTOD(orientation_error),displacement,cntrl.angular_velocity);
	return cntrl;
};