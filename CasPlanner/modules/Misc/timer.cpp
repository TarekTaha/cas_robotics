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
#include "timer.h"

Timer::Timer()
{
	gettimeofday(&start_time,NULL);
}

double  Timer::usecElapsed() // return in usec
{
	gettimeofday(&end_time,NULL);
	time_diff = ((double)end_time.tv_sec*1e6   + (double)end_time.tv_usec) -  
	            ((double)start_time.tv_sec*1e6 + (double)start_time.tv_usec);
	return time_diff;
}

double Timer::msecElapsed() // return in usec
{
	return (usecElapsed()/1000.0f);
}
double Timer::secElapsed()
{
	return (usecElapsed()/double(1e6));;
}

Timer::~Timer()
{
}
void Timer::restart()
{
	gettimeofday(&start_time,NULL);
}
void Timer::msecSynch(double period)
{
	double time_elapsed = this->msecElapsed();
	if( time_elapsed < period*1e3)
		usleep((int)(period*1e3 -time_elapsed)); 
}
