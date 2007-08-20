#include "mapskeleton.h"

MapSkeleton::MapSkeleton()
{
	verticies.resize(49);
	verticies[0].setLocation(-13.65,5.68);
	verticies[1].setLocation(-13.65,1.53);
	verticies[2].setLocation(-13.65,-1.4);
	verticies[3].setLocation(-13.65,-4.48);
	verticies[4].setLocation(-15.16,-4.48);
	verticies[5].setLocation(-15.11,-6.32);
	verticies[6].setLocation(-12.43,-4.48);
	verticies[7].setLocation(-12.43,-6.32);
	verticies[8].setLocation(-9.85,-4.48);
	verticies[9].setLocation(-9.9,-6.32);
	verticies[10].setLocation(-8.48,-4.48);
	verticies[11].setLocation(-7.17,-4.48);
	verticies[12].setLocation(-7.17,-6.32);
	verticies[13].setLocation(-5.52,-4.48);
	verticies[14].setLocation(-4.63,-4.48);
	verticies[15].setLocation(-4.58,-6.32);
	verticies[16].setLocation(-1.95,-4.48);
	verticies[17].setLocation(-1.9,-6.32);
	verticies[18].setLocation(0.64,-4.48);
	verticies[19].setLocation(0.78,-6.32);
	verticies[20].setLocation(1.72,-4.48);
	verticies[21].setLocation(3.27,-4.48);
	verticies[22].setLocation(3.36,-6.32);
	verticies[23].setLocation(4.96,-4.48);
	verticies[24].setLocation(9.42,-4.53);
	verticies[25].setLocation(9.61,-1.62);
	verticies[26].setLocation(4.96,-1.34);
	verticies[27].setLocation(4.96,1.48);
	verticies[28].setLocation(4.96,3.7);
	verticies[29].setLocation(4.96,7.36);
	verticies[30].setLocation(8.62,3.51);
	verticies[31].setLocation(8.44,6.23);
	verticies[32].setLocation(8.34,7.74);
	verticies[33].setLocation(11.3,6.18);
	verticies[34].setLocation(11.16,7.88);
	verticies[35].setLocation(11.12,4.45);
	verticies[36].setLocation(13.75,6.28);
	verticies[37].setLocation(13.75,7.88);
	verticies[38].setLocation(13.75,4.54);
	verticies[39].setLocation(1.62,1.53);
	verticies[40].setLocation(1.34,4.77);
	verticies[41].setLocation(1.2,-1.54);
	verticies[42].setLocation(-2,1.53);
	verticies[43].setLocation(-2.04,5.43);
	verticies[44].setLocation(-2.04,-1.52);
	verticies[45].setLocation(-5.52,1.48);
	verticies[46].setLocation(-5.33,-1.43);
	verticies[47].setLocation(-8.67,1.53);
	verticies[48].setLocation(-8.72,-1.2);
	/* Specify which verticies are considered destinations */	
	destIndexes.push_back(0);
	destIndexes.push_back(5);
	destIndexes.push_back(25);
	destIndexes.push_back(29);
	destIndexes.push_back(30);
	destIndexes.push_back(37);
}

MapSkeleton::~MapSkeleton()
{
	clear();
}

void MapSkeleton::clear()
{

}

int MapSkeleton::getCurrentSpatialState(Pose P)
{
	double dist,closest = Dist(P.p,verticies[0].location);
	int stateIndex=0,i;
	for(i=0; i < verticies.size(); i++)
	{
		if((dist=Dist(P.p,verticies[i].location))<closest)
		{
			closest = dist;
			stateIndex = i;
		}
	}
	return stateIndex;
}

void MapSkeleton::generateInnerSkeleton()
{
	Vertex v,vOpp;
}
