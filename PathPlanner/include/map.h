/***************************************************************************
 *   Copyright (C) 2007 by Tarek Taha                                      *
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
#ifndef MAP_H_
#define MAP_H_

#include "common.h"
#include <gdk-pixbuf/gdk-pixbuf.h>
#include <gtk/gtk.h>
#include <glib/gprintf.h>
#include <vector>
#include <sys/stat.h>

namespace CasPlanner
{
class Map 
{
	private:
		bool negate;
		GdkPixbuf * pixbuf;
        int width, height;
        double mapRes;		
	public:
		//! Hold the grid presenation of the map.
        bool  ** grid;
        //! The file name of the map's image file.
       	char  * mapFileName;
       	//! The center of the Map. Used for conversion from the Metric to the Pixel coordinate and vice versa.
        Point center;
        int   getWidth();
        int   getHeight();
        double getMapRes();
        bool  getNegate();
		void savePixelBuffer(char * name);
		void drawPixel(int,int,int,int,int);
		void allocateGrid();
		void freeGrid();
		int  readMapFile(char * filename);
		~Map();
        Map(int width, int height,float MapRes,bool negate);
        Map(char * filename,float MapRes,bool negate);
		//! transfers from pixel coordinate to the main coordinate system
		void convertPix(Point  *p); 
		//! transfers from main coordinate to the pixel coordinate system
		void convert2Pix(Point *p);
};
}
#endif /*Map_H_*/
