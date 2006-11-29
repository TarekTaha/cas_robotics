#ifndef SENSORS_H
#define SENSORS_H

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include <QWidget>
#include "robotmanager.h"
class Sensors: public QWidget 
{
    Q_OBJECT
    public:
        Sensors(QWidget *parent = 0,RobotManager *rob=0); 
        virtual int config()=0;
    public slots:
        virtual void updateData()=0; 
    protected:
        RobotManager *robotManager; 
    
}; 

#endif 

