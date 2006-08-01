#include <assert.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <unistd.h>
#include <sys/time.h>
// GUI settings
static int win,height,width;
static int gui_pause = 0;
static int viewport_width = 0;
static int viewport_height = 0;
static int show_samples = 1;
static double res,offsetx,offsety,mouse_offsetx,mouse_offsety;
// Mapping phases

// Local functions
static void process();

// Key callback
void win_key(unsigned char key, int x, int y)
{
  // Show the samples
  if (key == 'T' || key == 't')
  {
    show_samples = !show_samples;
    glutPostRedisplay();
  }

  // Pause
  else if (key == ' ')
  {
    if (!gui_pause)
      fprintf(stderr, "paused\n");
    else
      fprintf(stderr, "running\n");
    gui_pause = !gui_pause;
  }

  // Save the current map
  else if (key == 'W' || key == 'w')
  {
    //save();
  }
  
  return;
}



// Mouse callback
void win_mouse(int button, int state, int x, int y)
{
  if (state == GLUT_DOWN)
  {
    mouse_offsetx = 2 * x * res;
    mouse_offsety = 2 * y * res;      
  }
  else if (state == GLUT_UP)
  {
    offsetx += 2 * x * res - mouse_offsetx;
    offsety -= 2 * y * res - mouse_offsety;
    glutPostRedisplay();
  }

  return;
}
// Handle window reshape events
void win_reshape(int width, int height)
{
  glViewport(0, 0, width, height);
  viewport_width = width;
  viewport_height = height;
  return;
}
// Redraw the window
void win_redraw()
{
  double left, right, top, bot;
  glClearColor(0.7, 0.7, 0.7, 1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  left  = -res * width;
  right = +res * width;
  top   = -res * height;
  bot   = +res * height;

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(left, right, top, bot, -1, +10);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(offsetx,offsety, 0.0);

  // Draw the grid map
  // pmap_draw_map(pmap, opt_scale);

  // Draw the origin marker
  glColor3f(0, 0, 0);
  glBegin(GL_LINE_LOOP);
  glVertex3f(-1, -1, 0);
  glVertex3f(+1, -1, 0);
  glVertex3f(+1, +1, 0);
  glVertex3f(-1, +1, 0);
  glEnd();
  // draw the map
  glutSwapBuffers();
  return;
}


// Idle callback
void win_idle()
{
  if (!gui_pause)
  {
    process();
    glutPostRedisplay();
  }
  else
    usleep(100000);
  return;
}
// Trap SIGINTS
void signal_handle(int arg)
{
  // Save current state

  // Move on to fine phase or exit
  
  return;
};
// Run the GUI
int win_run(int w, int h , double resolution)
{
  width = w;
  height = h;
  res = resolution;
  // Register signal handlers
  assert(signal(SIGINT, signal_handle) != SIG_ERR);
  int argc;
  char **argv;
  printf("\n AM HERE 1"); fflush(stdout);
  glutInit(&argc, argv);
  printf("\n AM HERE 2"); fflush(stdout);
  // Create a window
  glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
  glutInitWindowSize( w + 16, h + 16);
  win = glutCreateWindow("Map Building Viewer");
  glutReshapeFunc(win_reshape);
  glutDisplayFunc(win_redraw);
  glutKeyboardFunc(win_key);
  glutMouseFunc(win_mouse);
  // Idle loop callback
  glutIdleFunc(win_idle);
  glutMainLoop();
  return 0;
}
// Do some appropriate form of processing
void process()
{
  cout<<"\nUpdating";
  return;
}


