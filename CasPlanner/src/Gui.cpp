#include "Gui.h"
#include "PlannerGui.cpp"
namespace GUI
{
Gui::Gui()
{
  int argc;
  char* argv[1];
  argc = 1;
  argv[0] = "blah";
	  // Make application
  FXApp application("Mobile Robotics","CAS PATH PLANNER");

  // Open the display
  application.init(argc,argv);

  // Make window
  new PlannerGui(&application);

  // Create the application's windows
  application.create(); 

  // Run the application
  application.run();
}

Gui::~Gui()
{
}

}
