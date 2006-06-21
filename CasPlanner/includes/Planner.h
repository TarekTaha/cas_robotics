#ifndef PLANNER_H_
#define PLANNER_H_

namespace CasPlanner
{

class Planner
{
public:
	Planner();
	virtual ~Planner();
private :
	SearchSpace * searchSpace;
	TreeSearch  * astar;
};

}

#endif /*PLANNER_H_*/
