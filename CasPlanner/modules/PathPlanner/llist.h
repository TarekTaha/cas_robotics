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
#ifndef LLIST_H_
#define LLIST_H_
#include <iostream>
#include <node.h>
using namespace std;
namespace CasPlanner
{

class LList
	{
		public :
			Node * 	Start;
		public :
			     	LList();
			     	~LList();
			void 	Add(Node *);
			bool 	Remove(Node *);
			void 	Print();
			void	Free();
			Node *  Find(Node *);
			void   	Next();
			void    Prev();
			Node *  GetHead();
	};

}

#endif /*LLIST_H_*/
