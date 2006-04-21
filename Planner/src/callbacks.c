#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif
#include <gtk/gtk.h>
#include <stdlib.h>
Point start, end,mouse_location;
const char * MapFilename;
int platform;
AstarPlanner * Planner=NULL;
PathFollower * Follower=NULL;
static GdkPixbuf *pixbuf=NULL;
bool start_activated=FALSE,end_activated=FALSE,initialized=FALSE,initialize_angle=FALSE;
gint drawable_x,drawable_y,h,w;
double initial_angle,final_angle,pixel_size,tracking_distance,linear_speed,safety_distance,obstacle_radius,
       pixels_per_tile,node_connection_distance,reg_grid_distance,bridge_length,k_distance,k_theta,bridge_gap,k_obstacle;
GtkWidget *temp, * view;
GtkTextBuffer *text_buffer;
GtkTextIter     startv, endv;
GTimer *timer;
GdkGC *gc = NULL;
GdkColor blue = {0};
Node * way_points,*way_temp,*current;
typedef struct
	{
	double kd,kt,tracking_distance;
	} thread_args;
thread_args *args = (thread_args *) args;
bool threading= TRUE;
bool st=TRUE;
void ReadPixBufferFromFile(const char * filename) // Reads a pixbuff from an RBG Map
{
	pixbuf = gdk_pixbuf_new_from_file(filename,NULL);
	h = gdk_pixbuf_get_height(pixbuf);
	w = gdk_pixbuf_get_width(pixbuf);
}
void RefreshDrawingArea(GtkWidget * widget) // Refreshes the Drawing Area by redrawing the pixel Buffer.
{
	if (!pixbuf)
			ReadPixBufferFromFile(MapFilename);
	temp = lookup_widget (GTK_WIDGET(widget),"drawingarea1");
  	gdk_draw_pixbuf(temp->window, NULL, pixbuf, 
		    	0, 0,
                    	0, 0,
                       -1,-1,
                    	GDK_RGB_DITHER_NORMAL, 0, 0);
	gtk_widget_set_size_request(temp,w,h);
};
void RedrawMap(GtkWidget * widget) // Redraws the Map from the saved and buffered file, Was used for testing only
	{
  	gchar *spacefile,*file;
	if (pixbuf)
		gdk_pixbuf_unref(pixbuf);
  	file=g_strdup(MapFilename);
  	spacefile=strchr(file,'.');
  	if(spacefile!=NULL)
  		*spacefile='\0';   
  	spacefile=g_strdup_printf("%s%s",file,"_FreeSpace.jpeg");
	pixbuf = gdk_pixbuf_new_from_file(spacefile,NULL);
	h = gdk_pixbuf_get_height(pixbuf);
	w = gdk_pixbuf_get_width(pixbuf);
	RefreshDrawingArea(widget);
	};
static void scroll(GtkWidget *v) // Supposed to scroll to the end of the Text View for some reason, that i will discover later, it's not working
{
	GtkTextBuffer	*buffer;
	GtkTextIter	start,end;
	GtkTextMark* mark;
	if (!v)
		return;
	buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(v));
    gtk_text_buffer_get_bounds (buffer, &start, &end);
   	mark = gtk_text_buffer_create_mark    (buffer, NULL, &end, 1);
   	gtk_text_view_scroll_to_mark(GTK_TEXT_VIEW(v), mark, 0.0, 0, 0.0, 1.0);
   	gtk_text_buffer_delete_mark (buffer, mark);
}
void add_text (GtkWidget * widget, char const * text) // Adds text to the Text View Widget
{
	view = lookup_widget (GTK_WIDGET(widget),"textview1");
  	text_buffer = gtk_text_view_get_buffer (GTK_TEXT_VIEW (view));
	gtk_text_buffer_get_end_iter(text_buffer, &endv);
	gtk_text_buffer_insert(text_buffer, &endv, text, -1);
	scroll(view);
}
void draw_pixel (int red,int green,int blue,int i,int j) // Draws a pixel of certain RBG colors into the pix buffer
	{
  	int rowstride=0, n_channels, bps;
  	guchar *pixels;
  	guchar * p;
  	rowstride = gdk_pixbuf_get_rowstride(pixbuf);
  	bps = gdk_pixbuf_get_bits_per_sample(pixbuf)/8;
  	n_channels = gdk_pixbuf_get_n_channels(pixbuf);
  	pixels = gdk_pixbuf_get_pixels(pixbuf);
  	if(gdk_pixbuf_get_has_alpha(pixbuf))
  		n_channels++;
  	p= pixels +j*rowstride + i*n_channels;
  	p[0]=red;
  	p[1]=green;
  	p[2]=blue;
  	//p[3]=;
	  return;
	}
void draw_path(GtkWidget * w)
{
  if (!Planner || !Planner->path)
	{
		add_text (GTK_WIDGET(w),g_strdup_printf("\n->NO Path Generated Yet, plan a Path First"));
		return;
	}

	Point l_start,l_end;
  	gchar *savefile,*file;
  	//int step=0;
  	Node *p;
  	p = Planner->path;
	while(p != NULL && p->next!=NULL)
	{
		l_start =  p->location;
		Planner->ConvertToPixel(&l_start);
		l_end   =  p->next->location;
		Planner->ConvertToPixel(&l_end);
		//add_text (GTK_WIDGET(w),g_strdup_printf("\n--> Step %03d: (%.4f, %.4f)", ++step, l_start.x, l_start.y));
  		temp = lookup_widget (w,"drawingarea1");
		if(p->parent!=NULL)
		{
//		for(int i=0;i<Planner->number_of_point_to_check;i++)
//			{

//			}
 		gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)p->wheelchair.check_points[0].x,(int)p->wheelchair.check_points[0].y,
		(int)p->wheelchair.check_points[4].x,(int)p->wheelchair.check_points[4].y);
  		gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)p->wheelchair.check_points[4].x,(int)p->wheelchair.check_points[4].y,
		(int)p->wheelchair.check_points[5].x,(int)p->wheelchair.check_points[5].y);
  		gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)p->wheelchair.check_points[5].x,(int)p->wheelchair.check_points[5].y,
		(int)p->wheelchair.check_points[1].x,(int)p->wheelchair.check_points[1].y);
  		gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)p->wheelchair.check_points[1].x,(int)p->wheelchair.check_points[1].y,
		(int)p->wheelchair.check_points[0].x,(int)p->wheelchair.check_points[0].y);

  		gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)l_start.x,(int)l_start.y,(int)l_end.x,(int)l_end.y);
		}
		p = p->next;
	} 
  	cout << "\n Path Drawn";
  	file=g_strdup(MapFilename);
  	savefile=strchr(file,'.');
  	if(savefile!=NULL)
  		*savefile='\0';   
  	savefile=g_strdup_printf("%s%s",file,"_PATH.jpeg");
  	gdk_pixbuf_save(pixbuf,savefile,"jpeg",NULL,NULL);//save the file
  	g_free(savefile);
  	g_free(file);
	//RefreshDrawingArea(w);
};
void draw_pixel_path(GtkWidget * w) // This draws the path generated by the 1 pixel/Tile using A start, Just draws Pixels
				    // Then it saves the generated path from the pixbuffer into the file adding _PATH to the file name
{
  if (!Planner->path)
	{
		add_text (GTK_WIDGET(w),g_strdup_printf("\n->NO Path Generated Yet, plan a Path First"));
		return;
	}
  	gchar *savefile,*file;
  	int step=0;
  	Node *p;
  	p = Planner->path;
	Point ni;
	while(p != NULL )
	{
		ni = p->location;
		add_text (GTK_WIDGET(w),g_strdup_printf("\n--> Step %03d: (%d, %d)", ++step, (int)ni.x, (int)ni.y));
		draw_pixel(0x14,0x08,0xff,(int)ni.x,(int)ni.y);
		if(p->parent==NULL)
			cout<<"\nI AM THE ROOOT";
		for(int i=0;i<4;i++)
			{
			draw_pixel(0x2c,0xff,0x0f,(int)(p->wheelchair.check_points[i].x),(int)(p->wheelchair.check_points[i].y));
			}
		fflush(stdout);
  		temp = lookup_widget (w,"drawingarea1");
		p = (Node *)p->next;
	} 
  	cout << "\n Path Drawn";
  	file=g_strdup(MapFilename);
  	savefile=strchr(file,'.');
  	if(savefile!=NULL)
  		*savefile='\0';   
  	savefile=g_strdup_printf("%s%s",file,"_PATH.jpeg");
  	gdk_pixbuf_save(pixbuf,savefile,"jpeg",NULL,NULL);//save the file
  	g_free(savefile);
  	g_free(file);
	RefreshDrawingArea(w);
};
void ConstructWaypoints(GtkWidget * w,double x,double y) // Using the mouse right button we can generate a path , this function does that ;)
	{
	Point s,line_start,line_end;
	if(!Planner)
		{
		Planner = new AstarPlanner(start,end,initial_angle,final_angle,pixel_size,obstacle_radius,w,MapFilename);
		Planner->ReadMap();
		}
	s.x=x;
	s.y=y;
	Planner->ConvertPixel(&s);
	way_temp = new Node;
	way_temp->location.x = s.x;
	way_temp->location.y = s.y;
	way_temp->parent = NULL;
	way_temp->prev = NULL;
	way_temp->angle= 0;
	way_temp->next =NULL;
	if (!way_points)
		{
		way_points = way_temp;
		current = way_temp;
		}
	else
		current->next = way_temp;
	if (current && current->next)
	{
		line_start.x = current->location.x;
		line_start.y = current->location.y;
		line_end.x   = current->next->location.x;
		line_end.y   = current->next->location.y;
		way_temp->parent = current;
		current->angle = atan2(current->next->location.y - current->location.y  ,current->next->location.x - current->location.x);
		Planner->ConvertToPixel(&line_start);
		Planner->ConvertToPixel(&line_end);
		printf("\n Start X[%.3f]Y[%.3f] End X[%.3f]Y[%.3f]",line_start.x,line_start.y,line_end.x,line_end.y);
  		temp = lookup_widget (w,"drawingarea1");
  		gdk_draw_line(temp->window,(GdkGC*)(temp)->style->white_gc,(int)line_start.x,(int)line_start.y,(int)line_end.x,(int)line_end.y);
	}
	current = way_temp;
	};
static void SetTargets (GtkWidget *widget, gdouble x, gdouble y) // Captures the left mouse clicks and uses them to set the source and target locations
{
  GtkWidget *w;
  //GdkGC *gc;
  //GdkColormap *colormap;
  GdkColor      mycolor         = { 0, 0x15, 0x3a, 0xfa };
  //GdkGCValues values;
  Point s,e;
	if(!Planner)
		{
		Planner = new AstarPlanner(start,end,initial_angle,final_angle,pixel_size,obstacle_radius,w,MapFilename);
		Planner->ReadMap();
		}
  mycolor.red   = 0x12;
  mycolor.green = 0xf2;
  mycolor.blue  = 0xfa;
  temp = lookup_widget (widget,"drawingarea1"); // Draws a rectangle aroung the target and destination locations.-90
  if (! start_activated ) // Sets the starting Location
	{
	if (!initialize_angle)
		{
			RefreshDrawingArea(widget);
			start.x = x;
			start.y = y;
			printf("\n---> Setting Source X=%f Y=%f",x,y);
			fflush(stdout);
        		w = lookup_widget (widget,"entry1");
        		gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%f", x));
        		w = lookup_widget (widget,"entry2");
        		gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%f", y));
			initialize_angle = TRUE;
			gdk_draw_arc (temp->window,(GdkGC*)(temp)->style->white_gc,TRUE,(int)x-5,(int)y-5,10,10,0,360*64);
		}
	else
		{	
			s.x = start.x;
			s.y = start.y;
			e.x = x;
			e.y = y;
			Planner->ConvertPixel(&s);
			Planner->ConvertPixel(&e);
			initial_angle = atan2(e.y - s.y,e.x - s.x);
			cout<<"\n---> Initial Angle is= "<<RTOD(initial_angle);
        		w = lookup_widget (widget,"entry3");
        		gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%f", RTOD(initial_angle)));
			start_activated  = TRUE;
			initialize_angle = FALSE;
			gdk_draw_line(temp->window,(GdkGC*)(temp)->style->white_gc,(int)start.x,(int)start.y,(int)x,(int)y);
		}
	}
  else // Sets the target Location
	{
	if (!initialize_angle)
		{
			end.x = x;
			end.y = y;
			printf("\n<--- Setting Target X=%f Y=%f",x,y);
			fflush(stdout);
        		w = lookup_widget (widget,"entry4");
	        	gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%f", x));
        		w = lookup_widget (widget,"entry5");
        		gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%f", y));
			initialize_angle = TRUE;
			gdk_draw_arc (temp->window,(GdkGC*)(temp)->style->white_gc,TRUE,(int)x-5,(int)y-5,10,10,0,360*64);
		}
	else
		{
			s.x = end.x;
			s.y = end.y;
			e.x = x;
			e.y = y;
			Planner->ConvertPixel(&s);
			Planner->ConvertPixel(&e);
			final_angle = atan2(e.y - s.y,e.x - s.x);
        		w = lookup_widget (widget,"entry6");
        		gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%f", RTOD(final_angle)));
			cout<<"\n<--- Final Angle is= "<<RTOD(final_angle);
			fflush(stdout);
			initialize_angle = FALSE;
			start_activated = end_activated =FALSE;
			gdk_draw_line(temp->window,(GdkGC*)(temp)->style->white_gc,(int)end.x,(int)end.y,(int)x,(int)y);
		}
	}  
}
void
on_filechooserbutton1_selection_changed
                                        (GtkFileChooser  *filechooser,
                                        gpointer         user_data)
{
	MapFilename = gtk_file_chooser_get_filename(filechooser);
	if (pixbuf)
		gdk_pixbuf_unref(pixbuf);
	pixbuf = gdk_pixbuf_new_from_file(MapFilename,NULL);
	h = gdk_pixbuf_get_height(pixbuf);
	w = gdk_pixbuf_get_width(pixbuf);
	printf("\nFile name:%s",MapFilename);
}

gboolean
on_drawingarea1_button_release_event   (GtkWidget       *widget,
                                        GdkEventButton  *event,
                                        gpointer         user_data)
{
 switch (event->button)
	{
	case 1: 
		if(pixbuf != NULL)
			SetTargets (widget, event->x, event->y);
		break;
	case 2:		
		printf("\nClearing List");
		while (way_points!=NULL)
			{
			way_temp = way_points;
			way_points = way_points->next;
			delete way_temp;
			}
		break;
	case 3:		
		ConstructWaypoints(widget,event->x,event->y);
		break;
	default : printf("\n Unknown Key Pressed <<<--- Should Never happen ;)");
	};
  return TRUE;
}


gboolean
on_drawingarea1_configure_event        (GtkWidget       *widget,
                                        GdkEventConfigure *event,
                                        gpointer         user_data)
{
	GtkWidget *w;
  	drawable_x = event->x;
  	drawable_y = event->y;
/***************************************************************************** Initialization ****************************************************/
 	if (!initialized)
	{
		//MapFilename = "maps/casarea.jpeg"; pixel_size = 0.047;
		MapFilename = "maps/casareaicp.jpeg"; pixel_size = 0.05;
		//MapFilename = "maps/cas.png"; pixel_size = 0.064;		
  		cout<<"\n Defaults Initialized";
		node_connection_distance = 0.5; // radius of circle inside which the nodes should be connected
		reg_grid_distance = 0.2;	// Distance between nodes generated as a regular grid space
		obstacle_radius = 0.2;		// obstacle expansion Radius
		bridge_length = 1.5;		// length of the bridge segment to be tested
		bridge_gap = 0.1;
		initialized = TRUE;		
		k_distance = 1.5;			// Gain applied on Distance Displacement for path following controller
		k_theta = 2.8;				// Gain applied on Orientation Error for path following controller
		k_obstacle = 0.8;
		tracking_distance = 0.0;	// Distance from the center of motion to the tracking point
		initial_angle = 180;		// Initial Angle Pose of the Robot
		way_points = NULL;
	   	platform = STAGE; 			// Default Platform
	   	
   		w = lookup_widget (widget,"pixel_size");
   		gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%.3f", pixel_size));
   		
   		w = lookup_widget (widget,"nodes_connection_radius");
   		gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%.1f", node_connection_distance));

   		w = lookup_widget (widget,"reg_grid_dist");
   		gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%.1f", reg_grid_distance));

   		w = lookup_widget (widget,"obstacle_radius");
   		gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%.2f", obstacle_radius));

   		w = lookup_widget (widget,"bridge_length");
   		gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%.2f", bridge_length));

//   		w = lookup_widget (widget,"pixel_size");
//   		gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%f", bridge_gap));

   		w = lookup_widget (widget,"k_distance");
   		gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%.2f", k_distance));

   		w = lookup_widget (widget,"k_theta");
   		gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%.2f", k_theta));

	}
/***************************************************************************** END Initialization *************************************************/
	printf("\n--->> Configure Event X=%d and Y =%d Height =%d Width=%d",event->x,event->y,event->height,event->width);
	fflush(stdout);
	RefreshDrawingArea(widget); // if the pixbuf == NULL then the refresh function will load the default file automatically
  return TRUE;
}


gboolean
on_drawingarea1_expose_event           (GtkWidget       *widget,
                                        GdkEventExpose  *event,
                                        gpointer         user_data)
{  
  if (pixbuf)
	{
  	printf("\n--->> Expose Event X=%d and Y =%d Height =%d Width=%d",event->area.x,event->area.y,event->area.height,event->area.width);
	fflush(stdout);
 	gtk_widget_set_size_request(widget,w,h);
  	gdk_draw_pixbuf(widget->window, NULL, pixbuf, 
                    event->area.x, event->area.y,
                    event->area.x, event->area.y,
                    event->area.width, event->area.height,
                    GDK_RGB_DITHER_NORMAL, 0, 0);
	if(Planner)
		{
//			Planner->draw_tree();	
			if(Planner->path)
				Planner->draw_path();
		}
	}
  return FALSE;
}


gboolean
on_drawingarea1_motion_notify_event    (GtkWidget       *widget,
                                        GdkEventMotion  *event,
                                        gpointer         user_data)
{
  int x, y;
  GdkModifierType state;
  if (event->is_hint)
    gdk_window_get_pointer (event->window, &x, &y, &state);
  else
    {
     mouse_location.x = x = (int) event->x;
     mouse_location.y = y = (int) event->y;
      //state = event->state;
    }
  cout<<"\n Mouse Location is x="<<mouse_location.x<<" y="<<mouse_location.y;
  fflush(stdout);
  if (state & GDK_BUTTON1_MASK && pixbuf != NULL)
	{
    		SetTargets (widget, x, y);
		//gdk_draw_point(widget,NULL,x,y);
	}
  return TRUE;
}


void
on_new1_activate                       (GtkMenuItem     *menuitem,
                                        gpointer         user_data)
{

}


void
on_open1_activate                      (GtkMenuItem     *menuitem,
                                        gpointer         user_data)
{
 //GtkWidget *chooser;
 //chooser = create_filechooserdialog1();
 //gtk_widget_show_all(chooser);

}


void
on_save1_activate                      (GtkMenuItem     *menuitem,
                                        gpointer         user_data)
{

}


void
on_save_as1_activate                   (GtkMenuItem     *menuitem,
                                        gpointer         user_data)
{

}


void
on_quit1_activate                      (GtkMenuItem     *menuitem,
                                        gpointer         user_data)
{

}


void
on_cut1_activate                       (GtkMenuItem     *menuitem,
                                        gpointer         user_data)
{

}


void
on_copy1_activate                      (GtkMenuItem     *menuitem,
                                        gpointer         user_data)
{

}


void
on_paste1_activate                     (GtkMenuItem     *menuitem,
                                        gpointer         user_data)
{

}


void
on_delete1_activate                    (GtkMenuItem     *menuitem,
                                        gpointer         user_data)
{

}


void
on_about1_activate                     (GtkMenuItem     *menuitem,
                                        gpointer         user_data)
{

}

void
on_tracking_distance_value_changed     (GtkRange        *range,
                                        gpointer         user_data)
{
	tracking_distance = gtk_range_get_value(range);
	//if(tracking_distance)
		//add_text (GTK_WIDGET(range),g_strdup_printf("\n->Tracking Distance changed to:%.3f", tracking_distance));
}


void
on_Linea_speed_value_changed           (GtkRange        *range,
                                        gpointer         user_data)
{
	linear_speed = gtk_range_get_value(range);
	//if(linear_speed)
		//add_text (GTK_WIDGET(range),g_strdup_printf("\n->Linear Speed changed to:%.3f", linear_speed));
	if(Follower)
		Follower->speed = linear_speed;
}		


void
on_safety_distance_value_changed       (GtkRange        *range,
                                        gpointer         user_data)
{
	safety_distance = gtk_range_get_value(range);
//	if(safety_distance)
		//add_text (GTK_WIDGET(range),g_strdup_printf("\n->Safety Distance changed to:%.3f", safety_distance));
}


void
on_startx_value_changed                (GtkRange        *range,
                                        gpointer         user_data)
{
GtkWidget * w;
	start.x = gtk_range_get_value(range);
        w = lookup_widget (GTK_WIDGET(range),"entry1");
        gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%.3f", start.x));
}


void
on_starty_value_changed                (GtkRange        *range,
                                        gpointer         user_data)
{
GtkWidget * w;
        w = lookup_widget (GTK_WIDGET(range),"entry2");
        gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%.3f", gtk_range_get_value(range)));
	start.y = gtk_range_get_value(range);
}


void
on_start_theta_value_changed           (GtkRange        *range,
                                        gpointer         user_data)
{
GtkWidget * w;
        w = lookup_widget (GTK_WIDGET(range),"entry3");
        gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%.3f", gtk_range_get_value(range)));
}


void
on_targetx_value_changed               (GtkRange        *range,
                                        gpointer         user_data)
{
GtkWidget * w;
        w = lookup_widget (GTK_WIDGET(range),"entry4");
        gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%.3f", gtk_range_get_value(range)));
	end.x = gtk_range_get_value(range);
}


void
on_targety_value_changed               (GtkRange        *range,
                                        gpointer         user_data)
{
GtkWidget * w;
        w = lookup_widget (GTK_WIDGET(range),"entry5");
        gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%.3f", gtk_range_get_value(range)));
	end.y = gtk_range_get_value(range);
}


void
on_target_theta_value_changed          (GtkRange        *range,
                                        gpointer         user_data)
{
GtkWidget * w;
        w = lookup_widget (GTK_WIDGET(range),"entry6");
        gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%.3f", gtk_range_get_value(range)));
}


void
on_pixel_size_changed                  (GtkEditable     *editable,
                                        gpointer         user_data)
{
	pixel_size = atof(gtk_editable_get_chars(editable,0,-1));
//	if(pixel_size)
		//add_text (GTK_WIDGET(editable),g_strdup_printf("\n->Pixel Size changed to:%.3f", pixel_size));
}


void
on_pixels_per_tile_changed             (GtkEditable     *editable,
                                        gpointer         user_data)
{
	pixels_per_tile = atof(gtk_editable_get_chars(editable,0,-1));
//	if(pixels_per_tile)
		//add_text (GTK_WIDGET(editable),g_strdup_printf("\n->Pixels Per Tile changed to:%.3f", pixels_per_tile));
}


void
on_obstacle_radius_changed             (GtkEditable     *editable,
                                        gpointer         user_data)
{
	obstacle_radius = atof(gtk_editable_get_chars(editable,0,-1));
//	if(obstacle_radius)
		//add_text (GTK_WIDGET(editable),g_strdup_printf("\n->Obstacle Radius changed to:%.3f", obstacle_radius));
}


void
on_bridge_length_changed               (GtkEditable     *editable,
                                        gpointer         user_data)
{
	bridge_length = atof(gtk_editable_get_chars(editable,0,-1));
//	if(bridge_length)
		//add_text (GTK_WIDGET(editable),g_strdup_printf("\n->Bridge Length changed to:%.3f", bridge_length));
}


void
on_k_distance_changed                  (GtkEditable     *editable,
                                        gpointer         user_data)
{
	k_distance = atof(gtk_editable_get_chars(editable,0,-1));
	//add_text (GTK_WIDGET(editable),g_strdup_printf("\n->K-Distance changed to:%.3f", k_distance));
	if(Follower)
		Follower->kd = k_distance;
}


void
on_k_theta_changed                     (GtkEditable     *editable,
                                        gpointer         user_data)
{
	k_theta = atof(gtk_editable_get_chars(editable,0,-1));
	//add_text (GTK_WIDGET(editable),g_strdup_printf("\n->K-theta changed to:%.3f", k_theta));
	if(Follower)
		Follower->kt = k_distance;	
}



void
on_entry1_changed                      (GtkEditable     *editable,
                                        gpointer         user_data)
{
	start.x=atof(gtk_editable_get_chars(editable,0,-1));
//	if(start.x)
		//add_text (GTK_WIDGET(editable),g_strdup_printf("\n->Start X changed to:%.3f", start.x));
}


void
on_entry2_changed                      (GtkEditable     *editable,
                                        gpointer         user_data)
{
	start.y=atof(gtk_editable_get_chars(editable,0,-1));
//	if(start.y)
		//add_text (GTK_WIDGET(editable),g_strdup_printf("\n->Start Y changed to:%.3f", start.y));
}


void
on_entry3_changed                      (GtkEditable     *editable,
                                        gpointer         user_data)
{
	initial_angle=DTOR(atof(gtk_editable_get_chars(editable,0,-1)));
//	if(start.y)
		//add_text (GTK_WIDGET(editable),g_strdup_printf("\n->Initial Angle changed to:%.3f", RTOD(initial_angle)));
}


void
on_entry4_changed                      (GtkEditable     *editable,
                                        gpointer         user_data)
{
	end.x=atof(gtk_editable_get_chars(editable,0,-1));
//	if(end.x)
		//add_text (GTK_WIDGET(editable),g_strdup_printf("\n->End X changed to:%.3f", end.x));
}


void
on_entry5_changed                      (GtkEditable     *editable,
                                        gpointer         user_data)
{
	end.y=atof(gtk_editable_get_chars(editable,0,-1));
//	if(end.y)
		//add_text (GTK_WIDGET(editable),g_strdup_printf("\n->End Y changed to:%.3f", end.y));
}


void
on_entry6_changed                      (GtkEditable     *editable,
                                        gpointer         user_data)
{
	//theta=atof(gtk_editable_get_chars(editable,0,-1));
}
void
on_generate_cspace_released            (GtkButton       *button,
                                        gpointer         user_data)
{
	if (Planner)
		delete Planner;
	timer = g_timer_new();
	Planner = new AstarPlanner(start,end,initial_angle,final_angle,pixel_size,obstacle_radius,GTK_WIDGET(button),MapFilename);
	Planner->ReadMap();
	while (gtk_events_pending())
		gtk_main_iteration();
	Planner->ExpandObstacles();
	add_text (GTK_WIDGET(button),g_strdup_printf("\n-->Obstacle Expansion took:%gs", g_timer_elapsed( timer, NULL )));
	g_timer_start(timer);
	while (gtk_events_pending())
		gtk_main_iteration();
	Planner->GenerateRegularGrid(reg_grid_distance);
	add_text (GTK_WIDGET(button),g_strdup_printf("\n-->Grid Generation took:%gs", g_timer_elapsed( timer, NULL )));
	g_timer_start(timer);
	while (gtk_events_pending())
		gtk_main_iteration();
	Planner->BridgeTest(bridge_length,bridge_gap); // m gap between nodes
	add_text (GTK_WIDGET(button),g_strdup_printf("\n-->Bridge Test took:%gs", g_timer_elapsed( timer, NULL )));
    g_timer_destroy( timer );
	while (gtk_events_pending())
		gtk_main_iteration();
	Planner->AddCostToNodes(3);
	Planner->ConnectNodes(node_connection_distance); // Connect Nodes that are within certain radius
	while (gtk_events_pending())
		gtk_main_iteration();
	if (pixbuf)
		gdk_pixbuf_unref(pixbuf);
	pixbuf = Planner->pixbuf;
	RefreshDrawingArea(GTK_WIDGET(button));
	Planner->ShowConnections();
}


void
on_path_plan_released                  (GtkButton       *button,
                                        gpointer         user_data)
{
	if (!Planner)
		{
		add_text (GTK_WIDGET(button),g_strdup_printf("\n-->Initialize Planner First ..."));
		return;
		}
	if (!Planner->search_space)
		{
		add_text (GTK_WIDGET(button),g_strdup_printf("\n-->Generate Search Space first ..."));
		return;
		}
	timer = g_timer_new();
	Planner->FreePath(); // Free previously generated Path if exists
	Planner->StartSearch(start,end,initial_angle,final_angle);
	add_text (GTK_WIDGET(button),g_strdup_printf("\n-->Path Planning took:%gs", g_timer_elapsed( timer, NULL )));
    g_timer_destroy( timer );
	Planner->PrintNodeList();
	Planner->draw_path();
//	Planner->draw_tree();
	//draw_straight_path(GTK_WIDGET(button));
	gdk_beep();
}
void
on_simulate_radio_toggled              (GtkToggleButton *togglebutton,
                                        gpointer         user_data)
{
	platform = STAGE;
	k_distance = 15;			// Gain applied on Distance Displacement for path following controller
	k_theta = 28;				// Gain applied on Orientation Error for path following controller
	GtkWidget * w;
   	w = lookup_widget (GTK_WIDGET(togglebutton),"k_distance");
   	gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%.2f", k_distance));
   	w = lookup_widget (GTK_WIDGET(togglebutton),"k_theta");
   	gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%.2f", k_theta));
}


void
on_wheelchair_radio_toggled            (GtkToggleButton *togglebutton,
                                        gpointer         user_data)
{
	platform = WHEELCHAIR;
	k_distance = 1.5;			// Gain applied on Distance Displacement for path following controller
	k_theta = 2.8;				// Gain applied on Orientation Error for path following controller
	GtkWidget * w;
   	w = lookup_widget (GTK_WIDGET(togglebutton),"k_distance");
   	gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%.2f", k_distance));
   	w = lookup_widget (GTK_WIDGET(togglebutton),"k_theta");
   	gtk_entry_set_text((GtkEntry*) w,g_strdup_printf("%.2f", k_theta));
}
void
on_follow_path_released                (GtkButton       *button,
                                        gpointer         user_data)
{
	Node *temp_node;
	int i=0;
	temp_node = way_points;
	while(temp_node !=NULL)
	{
		//a = atan2(temp_node->next->location.y - temp_node->location.y  ,temp_node->next->location.x -temp_node->location.x);
		printf("\n Current Node is=%d and Location= X[%.3f],Y[%.3f] angle is=[%.3f]",++i,temp_node->location.x,temp_node->location.y,RTOD(temp_node->angle));
		fflush(stdout); 
		temp_node = temp_node->next;
	}
	if (!Planner->path)
	{
		add_text(GTK_WIDGET(button),g_strdup_printf("\n --->>> No Path to Follow <<<---"));
		return;
	}
	if(Follower)
	{
		Follower->path = Planner->path;
		Follower->kd = k_distance;
		Follower->kt = k_theta;
		Follower->ko = k_obstacle;
		Follower->tracking_distance = tracking_distance;
		Follower->log = 1;
	}
	Follower = new PathFollower(Planner->path,k_distance,k_theta,k_obstacle,tracking_distance,GTK_WIDGET(button),1,Planner,pixbuf);
	Follower->Connect(platform);
	Follower->stop = FALSE;
	Follower->FollowPath(Planner->path);
}
void
on_stop_following_released                (GtkButton       *button,
                                        gpointer         user_data)
{
	if(Follower)
		Follower->stop = TRUE;
};

void
on_window_exit                         (GtkObject       *object,
                                        gpointer         user_data)
{
if (Planner)
	{
	delete Planner;
	}
if (pixbuf)
	gdk_pixbuf_unref(pixbuf);
gtk_main_quit();
}


void
on_reg_grid_changed                    (GtkEditable     *editable,
                                        gpointer         user_data)
{
	reg_grid_distance = atof(gtk_editable_get_chars(editable,0,-1));
//	if(reg_grid_distance)
		//add_text (GTK_WIDGET(editable),g_strdup_printf("\n->Arc Width changed to:%.3f", reg_grid_distance));
}


void
on_dist_node_changed                   (GtkEditable     *editable,
                                        gpointer         user_data)
{
	node_connection_distance = atof(gtk_editable_get_chars(editable,0,-1));
//	if(node_connection_distance)
		//add_text (GTK_WIDGET(editable),g_strdup_printf("\n->Arc Width changed to:%.3f", node_connection_distance));
}
void *print_message_function( thread_args *ptr );

void
on_connect_to_player_clicked           (GtkButton       *button,
                                        gpointer         user_data)
{
	Planner->draw_tree();
}

void *print_message_function( thread_args *ptr )
{
	for (int i=0;i<10;i++)
	{
		gdk_threads_enter(); //Thread Lock
     		g_usleep(100000);
     		printf("Kd =%.3f \n", ptr->kd);
     		g_usleep(100000);
     		printf("Kt =%.3f \n", ptr->kt);
     		g_usleep(100000);
     		printf("Tracking Distance=%.3f \n", ptr->tracking_distance);
		gdk_flush();
		//g_idle_add();
		gdk_threads_leave(); // Thread Release
	}
	g_thread_exit(g_thread_self());
return NULL;
}

