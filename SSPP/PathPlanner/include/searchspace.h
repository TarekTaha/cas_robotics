/***************************************************************************
 *   Copyright (C) 2007 by Tarek Taha                                      *
 *   tataha@eng.uts.edu.au                                                 *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef SEARCHSPACE_H_
#define SEARCHSPACE_H_

#include "searchspacenode.h"
namespace CasPlanner
{

class SearchSpace
{
	public:
		//! Pointer to the Search Space;
		SearchSpaceNode * searchSpace;	
		void freeSearchSpace();
		SearchSpaceNode * insertNode(Point loc);
		SearchSpaceNode * nodeExists(Point loc);
		bool              removeNode(Point loc);
		SearchSpace();
		virtual ~SearchSpace();
};

}

#endif /*SEARCHSPACE_H_*/
