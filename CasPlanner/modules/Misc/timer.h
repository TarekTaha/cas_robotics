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
#ifndef TIMER_H_
#define TIMER_H_
#include <sys/time.h>
#include <iostream>
class Timer
{
	private:
		struct timeval start_time,end_time;
		double time_diff;
	public:
		Timer();
		double msecElapsed(); // time elapsed in msec since last call
		double usecElapsed(); // time elapsed in usec since last call		
		double secElapsed();     // time elapsed in  sec since last call		
		void restart();       // resets timer
		virtual ~Timer();
		/* Synchronize the loop within a period
		 * To use this u will have to initialize the timer
		 * reset the timer at the beginning of the loop
		 * and call the Synch function at the end of the loop
		 */
		void msecSynch(double period); // period should be in msec
};

#endif /*TIMER_H_*/

