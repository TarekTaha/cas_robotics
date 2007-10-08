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
		ControlAction getAction(double angle_current,double angle_ref,int direction,double v,double y1,double s1);
		double deltaFunction(double y1,double v);
		double deltaFunctionPrime(double y1, double v);
		Controller();
		~Controller();	
};

#endif /*CONTROLLER_H_*/
