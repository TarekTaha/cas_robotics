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
#include "mainwindow.h"
#include <qapplication.h>

int main( int argc, char ** argv ) 
{
	QApplication a( argc, argv );
    if(argc > 1)
    {
		MainWindow* main_win = new MainWindow(QApplication::arguments());
        main_win->setWindowTitle( "CAS Navigation System" );
        main_win->show();
        a.connect( &a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()) );
        return a.exec();
    }
    else 
    {
        qFatal("Usage: CasPlanner <cfgfile>\n"); 
    }
}
