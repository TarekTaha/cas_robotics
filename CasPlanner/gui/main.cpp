#include "mainwindow.h"
#include <qapplication.h>


int main( int argc, char ** argv ) 
{
    QApplication a( argc, argv );
    if(argc == 1)
    {
		MainWindow *main_win = new MainWindow(); 
		main_win->setWindowTitle("Testing Mode only"); 
		main_win->show(); 
		a.connect( &a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()) );
		return a.exec();
    }
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
        qFatal("Usage: CasPlannerGui <cfgfile>\n"); 
    }
}
