#ifndef MAP_H_
#define MAP_H_
#include "gdk-pixbuf/gdk-pixbuf.h"
#include "gtk/gtkmain.h"
#include "gtk/gtk.h"
#include "glib/gprintf.h"
namespace CasPlanner
{

class MapInfo
	{
	public :
		int height;
		int width;
		double pixel_size;
	};
class Map
{
	private:
    		const char* MapFilename;
    		double pixel_size;
    		int negate;
    		int size_x, size_y;
  		GdkPixbuf * pixbuf;
	public:
		bool   * * mapdata;
		MapInfo    GetMapInfo();
		void SavePixelBufferToFile();
		void DrawPixel(int,int,int,int,int);
		GdkPixbuf * ReadMap();
		~Map();
		Map(char const *, double);
};

}

#endif /*MAP_H_*/
