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
#include "PanTilt.h"

PanTilt::PanTilt(char *vFD)
{
	if ((fDesc = open(vFD, O_RDWR )) == -1) 
	{
		perror("ERROR opening V4L interface \n");
	}
}

PanTilt::~PanTilt()
{
	if (fDesc!= -1) 
	{
		close(fDesc);
	}
}

void PanTilt::reset(int reset)
{
    struct v4l2_ext_control xctrls[2];
    struct v4l2_ext_controls ctrls;
	if (fDesc == -1) 
	{
		perror("ERROR Open Interface First\n");
		return;
	}
	
	switch(reset) 
	{
		case 1:
			xctrls[0].id = V4L2_CID_PAN_RESET;
			xctrls[0].value = 1;
			break;
		case 2:
			xctrls[0].id = V4L2_CID_TILT_RESET;
			xctrls[0].value = 1;
			break;
		case 3:
			xctrls[0].value = 3;
			xctrls[0].id = V4L2_CID_PANTILT_RESET;
			break;
	}

	ctrls.count = 1;
	ctrls.controls = xctrls;

	ioctl(fDesc, VIDIOC_S_EXT_CTRLS, &ctrls);
	return;
}

void PanTilt::pan(int pan)
{
        struct v4l2_ext_control xctrls[2];
        struct v4l2_ext_controls ctrls;
        
    	if (fDesc == -1)
    	{
    		perror("ERROR Open Interface First\n");
    		return;
    	}
    	
    	xctrls[0].id = V4L2_CID_PAN_RELATIVE;
    	xctrls[0].value = pan;
    	
    	ctrls.count = 1;
    	ctrls.controls = xctrls;
    	
        ioctl(fDesc, VIDIOC_S_EXT_CTRLS, &ctrls);
        return;
}

void PanTilt::tilt(int tilt)
{
        struct v4l2_ext_control xctrls[2];
        struct v4l2_ext_controls ctrls;
        
    	if (fDesc == -1)
    	{
    		perror("ERROR Open Interface First\n");
    		return;
    	}
    	
     	xctrls[0].id = V4L2_CID_TILT_RELATIVE;
    	xctrls[0].value = tilt;
    	
    	ctrls.count = 1;
    	ctrls.controls = xctrls;
    	
        ioctl(fDesc, VIDIOC_S_EXT_CTRLS, &ctrls);
        return;
}

void PanTilt::pantilt(int pan, int tilt)
{
        struct v4l2_ext_control xctrls[2];
        struct v4l2_ext_controls ctrls;
        
    	if (fDesc == -1)
    	{
    		perror("ERROR Open Interface First\n");
    		return;
    	}
    	
    	xctrls[0].id = V4L2_CID_PAN_RELATIVE;
    	xctrls[0].value = pan;
    	xctrls[1].id = V4L2_CID_TILT_RELATIVE;
    	xctrls[1].value = tilt;
    	
    	ctrls.count = 2;
    	ctrls.controls = xctrls;
    	
        ioctl(fDesc, VIDIOC_S_EXT_CTRLS, &ctrls);
        return;
}
