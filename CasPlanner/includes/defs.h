/***************************************************************************
 *   Copyright (C) 2005 by Tarek Taha                                      *
 *   tataha@eng.uts.edu.au                                                 *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
//#define OPEN 1
//#define NEW 0
//#define CLOSED 2
//#include <unistd.h> 
//#include <time.h> 
//#include <stdio.h>
//#include <stdlib.h>
//#include <math.h>
//#include <sys/types.h>
//#include <netinet/in.h>
//#include <string.h>
//#include <error.h>
//#include "SDL/SDL.h"
//#include <list>
//#include <gdk-pixbuf/gdk-pixbuf.h>
//#include <gtk/gtkmain.h>
//#include<gtk/gtk.h>
//#include<glib/gprintf.h>
//#include <assert.h>
//#include <playerclient.h>
//#include "common.h"
//#include "wheelchairproxy.h"
//#define max_speed 0.1
//#define max_turn_rate 0.2 // wheelchair negative rotations
//#define FORWARD 1
//#define BACKWARD -1
//#include "callbacks.h"
//#include "interface.h"
//#include "support.h"
//#include <Node.h>
//#include <Point.h>
//#include "Vector2D.h"
//enum {WHEELCHAIR,STAGE};
/*
 * Max
 * Return the maximum of two numbers.
 */
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
 
