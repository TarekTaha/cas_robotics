/***************************************************************************
 *   Copyright (C) 2005 by Tarek Taha   *
 *   tataha@eng.uts.edu.au   *
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

#ifdef HAVE_CONFIG_H 
#include <config.h>
#endif

#include <iostream>
#include <cstdlib>
#include <gtk/gtk.h>
#include <vector>
using namespace std;
using std::vector;
#include "Vector2D.h"
#include "astar.h"
//#include "astar_sim.h"
#include "interface.c"
#include "support.c"
#include "callbacks.c"



int main(int argc, char *argv[])
{
GtkWidget *window1;
  /* init threads */    
  if( !g_thread_supported() )
  {
     g_thread_init(NULL);
     gdk_threads_init();                   // Called to initialize internal mutex "gdk_threads_mutex".
     printf("g_thread supported\n");
  }
  else
  {
     printf("g_thread NOT supported\n");
  }
#ifdef ENABLE_NLS
	bindtextdomain (GETTEXT_PACKAGE, PACKAGE_LOCALE_DIR);
  	bind_textdomain_codeset (GETTEXT_PACKAGE, "UTF-8");
  	textdomain (GETTEXT_PACKAGE);
#endif
gtk_set_locale ();
gtk_init (&argc, &argv);
add_pixmap_directory ("/usr/local/share/pixmaps");
  /*
   * The following code was added by Glade to create one of each component
   * (except popup menus), just so that you see something after building
   * the project. Delete any components that you don't want shown initially.
   */
window1 = create_window1 ();
gtk_widget_show (window1);
gdk_threads_enter ();
	gtk_main ();
gdk_threads_leave ();
return EXIT_SUCCESS;
}
