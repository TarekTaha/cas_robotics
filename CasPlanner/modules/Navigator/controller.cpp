/***************************************************************************
 *   Copyright (C) 2006 - 2007 by                                          *
 *      Tarek Taha, CAS-UTS  <tataha@tarektaha.com>                        *
 *                                                                         *
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
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/
#include "controller.h"

Controller::Controller()
{
}

Controller::~Controller()
{
}

double Controller::deltaFunctionPrime(double y1, double v)
{	
//	const double thetaA = M_PI/4.0f;
//	return (-cos(v)*thetaA*tanh(y1) -sin(v)*thetaA*(1-tanh(y1)*tanh(y1)));
	return 0;
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
	cntrl.angular_velocity = deltaFunctionPrime(y1,v) -lambda*y1*v*(sin(orientation_error) - sin(deltaFunction(y1,v)))/(orientation_error - deltaFunction(y1,v)) - k2*(orientation_error - deltaFunction(y1,v));
//	qDebug("\nControl Action Linear:%f Angular:%f Theta Error=%f",cntrl.linear_velocity,cntrl.angular_velocity,RTOD(orientation_error));fflush(stdout);	
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
//		cntrl.linear_velocity  = 0;
		cntrl.angular_velocity *= 4;
	}
//	else
		cntrl.linear_velocity = speed;
	//qDebug("Ori-Err=[%.3f] Dist-Err=[%.3f] Wdem=[%.3f]",RTOD(orientation_error),displacement,cntrl.angular_velocity);
	return cntrl;
};
