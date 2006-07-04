#include "Navigator.h"

Navigator::Navigator()
{
}

Navigator::~Navigator()
{
}

int Navigator::config( ConfigFile *cf, int sectionid)
{
   	obst_avoid  =   cf->ReadString(sectionid, "obst_avoid", "non");
 	k_dist      =   cf->ReadFloat (sectionid, "k_dist", 1.8);
  	k_theta     =   cf->ReadFloat (sectionid, "k_theta", 2.5);
  	safety_dist =   cf->ReadFloat (sectionid, "safety_dist", 0.1);
 	return 1;
}

int Navigator::start()
{
    qDebug("-> Starting Robot Navigator."); 
    qDebug("*********************************************************************"); 	
  	qDebug("Navigation Parameters:"); 
  	qDebug("\t\t Obstacle Avoidance:\t%s", qPrintable(obst_avoid)); 
    qDebug("\t\t Controller Distance Gain:",k_dist); 
  	qDebug("\t\t Controller Theta    Gain:",k_theta);
  	qDebug("\t\t Safet Distance :",safety_dist);
    qDebug("*********************************************************************"); 
    qDebug("-> Robot Navigator Started."); 
    return 1;
}

int Navigator::stop()
{
  	return 1;
}