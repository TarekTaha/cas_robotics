/***************************************************************************
 *      Copyright (C) 2006 - 2008 by                                       *
 *      Tarek Taha, CAS-UTS  <tataha@cas.edu.au>                           *
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

//  vFD : video file descriptor (eg. /dev/video0)
//	Pan	: pan angle in 1/64th of degree
//	Tilt: tilt angle in 1/64th of degree
//	Reset:reset pan/tilt to the device origin

#ifndef PANTILT_H_
#define PANTILT_H_
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>

#include <linux/videodev.h>
#include "uvcvideo.h"
#include "dynctrl-logitech.h"

#define PAN			1 
#define TILT 		2
#define PANTILT		3
#define INCPANTILT 	64 // 1°
#define PANSTEP 	1
#define TILTSTEP 	1

class PanTilt
{
	public:
		PanTilt(char *vFD);
		~PanTilt();
		void pantilt(int pan, int tilt);
		void pan(int pan);		
		void tilt(int tilt);		
		void reset(int resetCode);	
	private:
		int fDesc;
};

#endif /*PANTILT_H_*/
