#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <QThread> 
#include "Vector2D.h"

typedef struct _control_action
	{
		double linear_velocity;
		double angular_velocity;
	} ControlAction;
	
class Controller: public QThread 
{
	public:
		double k_dist,k_theta,safety_dist,tracking_dist,linear_velocity;
		ControlAction getAction(double angle_current,double angle_ref,double displacement,int direction,double speed);
		Controller();
		~Controller();
};

#endif /*CONTROLLER_H_*/
