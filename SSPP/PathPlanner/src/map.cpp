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
#include "map.h"
#include <math.h>
#include <cassert>
#include <iostream>
using namespace std;

namespace CasPlanner
{
//! Map Constructor
/*! Constructor of the Map Class:
 * @param filename is the file name of the image map to open.
 * @param pixelRes is the underlying pixel/m resolution of the map.
 * @param negate_in specifies if the white pixels are empty or occupied in the map.
 */
Map::Map(char* filename, float pixelRes,bool negate_in):
negate(negate_in),
pixbuf(NULL),
width(0),
height(0),
mapRes(pixelRes),
grid(NULL),
mapFileName(filename)
{
	readMapFile(filename); 
};
//! Map Constructor
/*! An alternative Constructor of the Map Class used for the obstacle expansion.
 * @param width_in is the width of the map grid to be allocated.
 * @param height_in is the height of the map grid to be allocated. 
 * @param pixelRes is the underlying pixel/m resolution of the map.
 * @param negate_in specifies if the white pixels are empty or occupied in the map.
 */
Map::Map(int width_in, int height_in , float pixelRes,bool negate_in):
negate(negate_in),
pixbuf(NULL),
drawingPixBuf(NULL),
width(width_in),
height(height_in),   
mapRes(pixelRes),
grid(NULL),
mapFileName("mapFromInterface.")
{
//	GError* error = NULL;
	allocateGrid();
	this->pixbuf = gdk_pixbuf_new(GDK_COLORSPACE_RGB, true, 8, width_in, height_in);
//	pixbuf = gdk_pixbuf_new_from_file("casarea_sub.jpeg", &error);
	this->drawingPixBuf = gdk_pixbuf_copy(this->pixbuf);  
};
//! Allocates memory for the grid
/*! This method allocates memory for the grid representation of the Map.
 * This grid is an WxH matrix of bool which represent if the underlying image 
 * pixels are free or occupied.
 */
void Map::allocateGrid()
{
	if(grid)
		freeGrid();
	this->grid = new bool * [width];
//	printf("\nAllocating Grid %d X %d map, at %.3f m/pix",this->width, this->height, this->mapRes); fflush(stdout);
	for(int i=0; i < width; i++)
	{
		grid[i] = new bool [height];
		for(int j=0;j < height;j++)
			grid[i][j] = true;
	}
	center.setX(width/2.0f);
	center.setY(height/2.0f);
};
//! Frees the memory allocated for the grid
void Map::freeGrid()
{
	if(grid)
	{
		for (int i=0; i < width; i++)
		{
    		delete  [] grid[i];
		}
		delete [] grid;
	}	
}
//! Destructor of the Map Class.
Map::~Map()
{
	freeGrid();
	if(pixbuf)
		gdk_pixbuf_unref(pixbuf);
	if(drawingPixBuf)
		gdk_pixbuf_unref(drawingPixBuf);
}
//! Accessor Method to the Map's Height.
int Map::getHeight()
{
	return this->height;
}
//! Accessor Method to the Map's Width.
int Map::getWidth()
{
	return this->width;
}
//! Accessor Method to the Map's Resolution.
double Map::getMapRes()
{
	return this->mapRes;
}
//! Accessor Method for the map's nagate.
bool  Map::getNegate()
{
	return this->negate;
}
//! Transfers from pixel coordinate to the main coordinate system
/*! This method converts from the pixel coordinate system where the 
 *  origin O(0,0) is the upper left corner of the image to the metric 
 * Coordinate system where the Origin O' is usually the center of the image.
 * Note: the center of the metric system can be define by the #CasPlanner::Map::center attribute in 
 * the #CasPlanner::Map class and the default is that middle of the image.
 */
void Map::convertPix(Point  *p) 
{
	p->setX( p->x()*mapRes - mapRes*center.x());
	p->setY(-p->y()*mapRes + mapRes*center.y());
};

//! Transfers from main coordinate to the pixel coordinate system
/*! This method transfers from the metric coordinate system to the who's 
 *  origin O' is defined by the #CasPlanner::Map:center in the #CasPlanner::Map class, to the image 
 * pixel coordinate system who's origin O(0,0) is the upper left corner of the image.
 */ 
void Map::convert2Pix(Point *p)
{
	p->setX(( p->x() + mapRes*center.x())/mapRes);
	p->setY((-p->y() + mapRes*center.y())/mapRes);
}        
//! Draws an RGB pixel inside the pixel Buffer
/*! This method is used to draw RBG pixels inside the pixel buffer for
 * debugging reasons. This is very handy to show the path or the search Space
 * generated.
 * @param red is the   Red value of the pixel.
 * @param green is the Green value of the pixel.
 * @param blue is the  Blue value of the pixel.
 * @param i is the column index of the pixel.
 * @param j is the row index of the pixel.
 */
void Map::drawPixel (int red,int green,int blue,int i,int j)
{
  	if(!this->drawingPixBuf)
  	{
  		puts("I can't draw into an empty pixel Buffer !!!");
  		return;
  	}
	int rowstride=0, n_channels, bps;
  	guchar *pixels;
  	guchar * p;
  	rowstride = gdk_pixbuf_get_rowstride(this->drawingPixBuf);
  	bps = gdk_pixbuf_get_bits_per_sample(this->drawingPixBuf)/8;
  	n_channels = gdk_pixbuf_get_n_channels(this->drawingPixBuf);
  	pixels = gdk_pixbuf_get_pixels(this->drawingPixBuf);
  	if(gdk_pixbuf_get_has_alpha(this->drawingPixBuf))
	  	n_channels++;
  	p= pixels +j*rowstride + i*n_channels;
  	p[0]=red;
  	p[1]=green;
  	p[2]=blue;
  	//p[3]=;
	  return;
}
//! Saves the Pixel buffer into an image file.
/*! This method saves the current pixel buffer into a file. This is mainly for debugging
 * reasons for showing the Search Space and the Path generated .
 */
void Map::savePixelBuffer(char * extname)
{
	char command[40],filename[40];
	struct stat stat_buf;
  	if(!this->drawingPixBuf)	
	{
		cout<<"		--->>> Nothing To SAVE Buffer Empty !!! ";
		return;
	}
	char * pos = strrchr(mapFileName,'.');
	strncpy(filename,mapFileName,pos-mapFileName);
	filename[pos-mapFileName]= '\0';
	strcat(filename,extname);
	// Delete the file if it exists
 	if (stat(filename,&stat_buf) != 0 || (stat_buf.st_mode & S_IFMT) == S_IFREG)
	{
		sprintf(command,"%s%s","rm -f -r ",filename); 
		if(system(command)==-1)
		{
			perror("\nError Happened while trying to Delete Existing File");
			exit(1);
		}
		else
			cout<<"\n	--->>> Map already existed with the same name : Deleted Successfully";
	}
	cout<<"\n	--->>> Saving the map into: "<<filename; fflush(stdout);
	// Save the file
	gdk_pixbuf_save(this->drawingPixBuf,filename,"png",NULL,NULL);
  	cout<<"\n	--->>> PIXEL BUFFER SAVED <<<---	"; fflush(stdout);
	
	if(this->drawingPixBuf)
	{
		gdk_pixbuf_unref(this->drawingPixBuf);
		this->drawingPixBuf = gdk_pixbuf_copy(this->pixbuf);
	}	  	
};
//! Reads the map pixel buffer from the image.
/*! This method is used to read the image file and copy it to a 
 * pixel buffer which will then be used to greate the underlying grid representation
 * of the map.
 */
int Map::readMapFile(char * filename)
{
  	guchar* pixels;
  	guchar* p;
  	int rowstride, n_channels, bps;
  	GError* error = NULL;
  	int i,j,k;
  	double occ;
  	int color_sum;
  	double color_avg;
  	this->mapFileName = filename;
    g_type_init();
  	printf("\nMapFile loading image file: %s...", this->mapFileName);
  	fflush(stdout);

  	// Read the image
  	if(!(pixbuf = gdk_pixbuf_new_from_file(this->mapFileName, &error)))
  	{
    	printf("\nfailed to open image file %s", this->mapFileName);
    	return(-1);
  	}
	this->drawingPixBuf = gdk_pixbuf_copy(this->pixbuf);
  	this->width = gdk_pixbuf_get_width(pixbuf);
  	this->height = gdk_pixbuf_get_height(pixbuf);

	allocateGrid();

 	rowstride = gdk_pixbuf_get_rowstride(pixbuf);
  	bps = gdk_pixbuf_get_bits_per_sample(pixbuf)/8;
  	n_channels = gdk_pixbuf_get_n_channels(pixbuf);
  	if(gdk_pixbuf_get_has_alpha(pixbuf))
    	n_channels++;
  	// Read data
  	pixels = gdk_pixbuf_get_pixels(pixbuf);
  	int count =0;
  	for(j = 0; j < this->height; j++)
  	{
    	for (i = 0; i < this->width; i++)
    	{
      		p = pixels + j*rowstride + i*n_channels*bps;
      		color_sum = 0;
      		for(k=0;k<n_channels;k++)
        		color_sum += *(p + (k * bps));
      		color_avg = color_sum / (double)n_channels;

      		if(this->negate)
        		occ = color_avg / 255.0;
      		else
        		occ = (255 - color_avg) / 255.0;
      		if(occ < 0.1)
      		{
        		this->grid[i][j] = 0;
      		}
	  		else
	  		{
	  			this->grid[i][j] = 1;
	  			count++;
	  		}
    	}
  	}
  	printf("\nMapFile read a %d X %d map, at %.3f m/pix",this->width, this->height, this->mapRes); fflush(stdout);
  	return(0);	
};
}
