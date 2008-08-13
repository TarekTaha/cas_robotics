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
#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_
#include <stdint.h> 
#include <string.h> 
#include <string> 
#include <iostream>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#define BUFFERSIZE 1024
class Accelerometer
{
public:
	Accelerometer();
	virtual ~Accelerometer();
	int connectBT(uint8_t port, char MAC[18]);
	void readBT();
	int getX();
	int getY();
	int getZ();
	bool isConnected();
private:
	int x,y,z;
	struct sockaddr_rc addr;
	int s, status;
	char n95MAC[18];
	uint8_t port;
	char buffer[BUFFERSIZE];
	bool connected;
};

#endif /*ACCELEROMETER_H_*/
