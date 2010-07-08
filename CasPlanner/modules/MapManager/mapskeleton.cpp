/***************************************************************************
 *   Copyright (C) 2006 - 2007 by                                          *
 *      Tarek Taha, CAS-UTS  <tataha@tarektaha.com>                        *
 *                                                                         *
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
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/
#include "mapskeleton.h"

MapSkeleton::MapSkeleton()
{
    numStates = 49;
    numDestinations = 6;
    verticies.resize(numStates);
    verticies[0].setLocation(-13.65,5.68); verticies[0].connect(1,D);
    verticies[1].setLocation(-13.65,1.53); verticies[1].connect(2,D); verticies[1].connect(47,R);verticies[1].connect(0,U);
    verticies[2].setLocation(-13.65,-1.4); verticies[2].connect(1,U); verticies[2].connect(3,D);
    verticies[3].setLocation(-13.65,-4.48);verticies[3].connect(2,U); verticies[3].connect(4,L); verticies[3].connect(6,R);
    verticies[4].setLocation(-15.16,-4.48);verticies[4].connect(3,R); verticies[4].connect(5,D);
    verticies[5].setLocation(-15.11,-6.32);verticies[5].connect(4,U);
    verticies[6].setLocation(-12.43,-4.48);verticies[6].connect(8,R); verticies[6].connect(3,L); verticies[6].connect(7,D);
    verticies[7].setLocation(-12.43,-6.32);verticies[7].connect(6,U);
    verticies[8].setLocation(-9.85,-4.48); verticies[8].connect(10,R);verticies[8].connect(6,L); verticies[8].connect(9,D);
    verticies[9].setLocation(-9.9,-6.32);  verticies[9].connect(8,U);
    verticies[10].setLocation(-8.48,-4.48);verticies[10].connect(11,R);verticies[10].connect(8,L); verticies[10].connect(48,U);
    verticies[11].setLocation(-7.17,-4.48);verticies[11].connect(13,R);verticies[11].connect(10,L);verticies[11].connect(12,D);
    verticies[12].setLocation(-7.17,-6.32);verticies[12].connect(11,U);
    verticies[13].setLocation(-5.52,-4.48);verticies[13].connect(14,R);verticies[13].connect(11,L);verticies[13].connect(46,U);
    verticies[14].setLocation(-4.63,-4.48);verticies[14].connect(16,R);verticies[14].connect(13,L);verticies[14].connect(15,D);
    verticies[15].setLocation(-4.58,-6.32);verticies[15].connect(14,U);
    verticies[16].setLocation(-1.95,-4.48);verticies[16].connect(18,R);verticies[16].connect(14,L);verticies[16].connect(17,D);
    verticies[17].setLocation(-1.9,-6.32); verticies[17].connect(16,U);
    verticies[18].setLocation(0.64,-4.48); verticies[18].connect(20,R);verticies[18].connect(16,L);verticies[18].connect(19,D);
    verticies[19].setLocation(0.78,-6.32); verticies[19].connect(18,U);
    verticies[20].setLocation(1.72,-4.48); verticies[20].connect(21,R);verticies[20].connect(18,L);verticies[20].connect(41,U);
    verticies[21].setLocation(3.27,-4.48); verticies[21].connect(23,R);verticies[21].connect(20,L);verticies[21].connect(22,D);
    verticies[22].setLocation(3.36,-6.32); verticies[22].connect(21,U);
    verticies[23].setLocation(4.96,-4.1);  verticies[23].connect(24,R);verticies[23].connect(21,L);verticies[23].connect(26,U);
    verticies[24].setLocation(9.1,-4.8);   verticies[24].connect(23,L);verticies[24].connect(25,U);
    verticies[25].setLocation(9.61,-1.62); verticies[25].connect(24,D);
    verticies[26].setLocation(4.96,-1.34); verticies[26].connect(23,D);verticies[26].connect(27,U);
    verticies[27].setLocation(4.7,1.48);   verticies[27].connect(26,D);verticies[27].connect(28,U);verticies[27].connect(39,L);
    verticies[28].setLocation(4.96,3.7);   verticies[28].connect(29,U);verticies[28].connect(30,R);verticies[28].connect(27,D);
    verticies[29].setLocation(4.96,7.36);  verticies[29].connect(28,D);
    verticies[30].setLocation(8.62,3.51);  verticies[30].connect(28,L);verticies[30].connect(31,U);
    verticies[31].setLocation(8.44,6.23);  verticies[31].connect(32,U);verticies[31].connect(30,D);
    verticies[32].setLocation(8.34,7.74);  verticies[32].connect(31,D);
    verticies[33].setLocation(11.3,6.18);  verticies[33].connect(31,L);verticies[33].connect(36,R);verticies[33].connect(34,U);verticies[33].connect(35,D);
    verticies[34].setLocation(11.16,7.88); verticies[34].connect(33,D);
    verticies[35].setLocation(11.12,4.45); verticies[35].connect(33,U);
    verticies[36].setLocation(13.75,6.28); verticies[36].connect(33,L);verticies[36].connect(37,U);verticies[36].connect(38,D);
    verticies[37].setLocation(13.75,7.88); verticies[37].connect(36,D);
    verticies[38].setLocation(13.75,4.54); verticies[38].connect(36,U);
    verticies[39].setLocation(1.62,1.53);  verticies[39].connect(27,R);verticies[39].connect(40,U);verticies[39].connect(41,D);verticies[39].connect(42,L);
    verticies[40].setLocation(1.34,4.77);  verticies[40].connect(39,D);
    verticies[41].setLocation(1.2,-1.54);  verticies[41].connect(39,U);verticies[41].connect(20,D);
    verticies[42].setLocation(-2,1.53);    verticies[42].connect(39,R);verticies[42].connect(43,U);verticies[42].connect(44,D);verticies[42].connect(45,L);
    verticies[43].setLocation(-2.04,5.43); verticies[43].connect(42,D);
    verticies[44].setLocation(-2.04,-1.52);verticies[44].connect(42,U);verticies[44].connect(16,D);
    verticies[45].setLocation(-5.52,1.48); verticies[45].connect(42,R);verticies[45].connect(47,L);verticies[45].connect(46,D);
    verticies[46].setLocation(-5.33,-1.43);verticies[46].connect(45,U);verticies[46].connect(13,D);
    verticies[47].setLocation(-8.67,1.53); verticies[47].connect(45,R);verticies[47].connect(1,L);verticies[47].connect(48,D);
    verticies[48].setLocation(-8.72,-1.2); verticies[48].connect(47,U);verticies[48].connect(10,D);
    /*
     * Specify which verticies are considered destinations
     * Order is very Important and it should be the same as
     * In the Pomdp Model Definition
     */
    destIndexes.push_back(0);
    destIndexes.push_back(5);
    destIndexes.push_back(43);
    destIndexes.push_back(29);
    destIndexes.push_back(25);
    destIndexes.push_back(37);
}

MapSkeleton::~MapSkeleton()
{
    clear();
}

void MapSkeleton::clear()
{

}

double MapSkeleton::getDist2SpatialState(Pose P,int stateIndex)
{
    return Dist(P.p,verticies[stateIndex].location);
}

int MapSkeleton::getDestIndex(int state)
{
    int i = destIndexes.indexOf(state);
    return i;
}

int MapSkeleton::getNumDestinations()
{
    return destIndexes.size();
}

int MapSkeleton::getNumVerticies()
{
    return verticies.size();
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
