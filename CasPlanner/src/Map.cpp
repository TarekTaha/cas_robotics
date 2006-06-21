#include "Map.h"
#include<iostream>
using namespace std;
namespace CasPlanner
{


Map::Map(char const *MapFilename, double pixel_size)
{
  	this->MapFilename = MapFilename;
  	this->pixel_size = pixel_size;
  	this->negate = true;
  	this->mapdata = NULL;
  	this->size_x = this->size_y = 0;
	this->mapdata=NULL;
	this->pixbuf = NULL;
}

GdkPixbuf * Map::ReadMap()
{
  	guchar* pixels;
  	guchar* p;
  	int rowstride, n_channels, bps;
  	GError* error = NULL;
  	int x,y,k;
  	double occ;
  	int color_sum;
  	double color_avg;
	if (this->mapdata)
		{
		for (int i=0; i < this->size_x; i++)
        		delete  [] this->mapdata[i];
    		delete [] this->mapdata;
		}
	if (!this->pixbuf)
	{
  		printf("\n	+++>>> Loading Image File: %s... <<<+++", this->MapFilename);
  		fflush(stdout);	
  		if(!(this->pixbuf = gdk_pixbuf_new_from_file(this->MapFilename, &error))) 	// Read the image
  		{
    			printf("\nfailed to open image file %s", this->MapFilename);
    			return(NULL);
  		}
	}
  	this->size_x = gdk_pixbuf_get_width(this->pixbuf);
	this->size_y = gdk_pixbuf_get_height(this->pixbuf);	
	this->mapdata= new bool* [size_x];
	for(int m=0;m<size_x;m++)
		this->mapdata[m] = new bool [size_y];
  	rowstride = gdk_pixbuf_get_rowstride(this->pixbuf);
  	bps = gdk_pixbuf_get_bits_per_sample(this->pixbuf)/8;
  	n_channels = gdk_pixbuf_get_n_channels(this->pixbuf);
  	if(gdk_pixbuf_get_has_alpha(this->pixbuf))
    	n_channels++;
  	pixels = gdk_pixbuf_get_pixels(this->pixbuf);	// Read data
  	for(y = 0; y < this->size_y; y++)
  	{
		for (x = 0; x < this->size_x; x++)
		{
	  		p = pixels + y*rowstride + x*n_channels*bps;
	  		color_sum = 0;
	  		for(k=0;k<n_channels;k++)
	    		color_sum += *(p + (k * bps));
	  		color_avg = color_sum / (double)n_channels;
	  		if(this->negate) // Negate = True means white is occuppied and Black is free
	        	occ = color_avg / 255.0; //White is 0xFF 0xFF 0xFF = 255 Black = 0x00 0x00 0x00 = 0
	  		else
	    		occ = (255 - color_avg) / 255.0;
	  		if(occ > 0.9)
				this->mapdata[x][y]= 1; // occupied
	  		else 
				this->mapdata[x][y]= 0; // free
		}
  }
  	printf("\n	--->>> IMAGE READ of Height=%d Width=%d Resolution=%.3f <<<---",this->size_y, this->size_x, this->pixel_size);
  	fflush(stdout);
  	return this->pixbuf;
}

Map::~ Map()
{
	for (int i=0; i < this->size_x; i++)
        	delete  [] this->mapdata[i];
    	delete [] this->mapdata;
    	gdk_pixbuf_unref(this->pixbuf);
    	cout <<"\n	<<<--- IMAGE DATA FREED <<<---";
    	fflush(stdout); 
};
MapInfo Map::GetMapInfo()
{
	MapInfo mapinfo;
	if(this->mapdata == NULL)
		{
		mapinfo.pixel_size=-1;
		return mapinfo;
		}
  	mapinfo.width = this->size_x;
  	mapinfo.height =this->size_y;
	mapinfo.pixel_size=this->pixel_size;
	return mapinfo;
};
void Map::DrawPixel (int red,int green,int blue,int i,int j)
{
	int rowstride=0, n_channels, bps;
  	guchar *pixels;
  	guchar * p;
  	rowstride = gdk_pixbuf_get_rowstride(this->pixbuf);
  	bps = gdk_pixbuf_get_bits_per_sample(this->pixbuf)/8;
  	n_channels = gdk_pixbuf_get_n_channels(this->pixbuf);
  	pixels = gdk_pixbuf_get_pixels(this->pixbuf);
  	if(gdk_pixbuf_get_has_alpha(this->pixbuf))
	  	n_channels++;
  	p= pixels +j*rowstride + i*n_channels;
  	p[0]=red;
  	p[1]=green;
  	p[2]=blue;
  	//p[3]=;
	  return;
}
void Map::SavePixelBufferToFile()
{
  	gchar *savefile,*file;
  	file=g_strdup(this->MapFilename);
  	savefile=strchr(file,'.');  	//ignore the extension and save as jpeg.
  	if(savefile!=NULL)
	  	*savefile='\0';   
  	savefile=g_strdup_printf("%s%s",file,"_FreeSpace.png");
  	gdk_pixbuf_save(this->pixbuf,savefile,"png",NULL,NULL);//save the file
  	cout<<"\n	--->>> PIXEL BUFFER SAVED <<<---	";
  	g_free(savefile);
};

}
