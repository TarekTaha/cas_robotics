#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <QObject>

class Controller : public QObject 
{
public:
	double k_dist,k_theta,safety_dist;
	Controller();
	virtual ~Controller();
};

#endif /*CONTROLLER_H_*/
