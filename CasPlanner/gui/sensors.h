#ifndef SENSORS_H
#define SENSORS_H

#include <QWidget>
#include <CommManager.h>

class Sensors: public QWidget 
{
    Q_OBJECT
    public:
        Sensors(CommManager *commsMgr, QWidget *parent = 0); 
//        virtual int config(ConfigFile *cf, int sectionid)=0;
        virtual int config()=0;
    public slots:
        virtual void updateData()=0; 
    protected:
        CommManager *commsMgr; 
    
}; 

#endif 

