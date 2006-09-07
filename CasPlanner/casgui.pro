HEADERS +=  gui/commmanager.h\
			gui/comms.h\
			gui/configfile.h\
			gui/controller.h\
			gui/glrender.h\
			gui/interfaceprovider.h\
			gui/laserrender.h\
			gui/mainwindow.h\
			gui/mapviewer.h\	
			gui/mappainter.h\		
			gui/mapmanager.h\
			gui/navigationtab.h\
			gui/navigator.h\
			gui/ogrender.h\
			gui/playerinterface.h\
			gui/playground.h\
			gui/robothandler.h\
			gui/robotmanager.h\
			gui/robotrender.h\
			gui/sensors.h\
			gui/sensorsgui.h\
			gui/speedrender.h\
			gui/statusbar.h\
			gui/tabcontainer.h\
			includes/Astar.h\
			includes/C2DMatrix.h\
			includes/common.h\
			includes/defs.h\
			includes/ForceField.h\
			includes/geometry.h\
			includes/geometry2D.h\			
			includes/icp.h\			
			includes/LList.h\
			includes/nn.h\			
			includes/Node.h\
			includes/PathPlanner.h\
			includes/Robot.h\
			includes/SearchSpace.h\
			includes/SearchSpaceNode.h\
			includes/Transformations.h\
			includes/utils.h\
			includes/Vector2D.h\
			includes/wheelchairproxy.h

SOURCES +=  gui/commmanager.cpp\
			gui/configfile.cc\
			gui/controller.cpp\
			gui/laserrender.cpp\
			gui/main.cpp\
			gui/mainwindow.cpp\
			gui/mapviewer.cpp\			
			gui/mappainter.cpp\
			gui/mapmanager.cpp\
			gui/navigationtab.cpp\
			gui/navigator.cpp\
			gui/ogrender.cpp\			
			gui/playerinterface.cpp\
			gui/planningmanager.cpp\
			gui/playground.cpp\
			gui/robotmanager.cpp\
			gui/robotrender.cpp\
			gui/sensors.cpp\
			gui/sensorsgui.cpp\
			gui/speedrender.cpp\
			gui/statusbar.cpp\
			gui/tabcontainer.cpp\
			src/Astar.cpp\
		    src/ForceField.cpp\
		    src/geometry2D.cpp\		    
		    src/icp.cpp\		    
		    src/nn.cpp\		    
		    src/LList.cpp\
		    src/Node.cpp\
		    src/PathPlanner.cpp\
		    src/Robot.cpp\
		    src/SearchSpace.cpp\
		    src/SearchSpaceNode.cpp\
		    src/Vector2d.cpp
RESOURCES = resources/icons.qrc
QT += opengl
QMAKE_CFLAGS_RELEASE+= -g -O3 -o 
#QMAKE_CFLAGS_RELEASE+= -g -O3 -ffast-math -march=pentium-m -msse2 -mfpmath=sse 
#QMAKE_CXXFLAGS_RELEASE+= -g -O3 -ffast-math -march=pentium-m -msse2 -mfpmath=sse
#QMAKE_LFLAGS_RELEASE += -g -O3 -ffast-math -march=pentium-m -msse2 -mfpmath=sse
#QMAKE_CFLAGS_DEBUG +=-pg
#QMAKE_CXXFLAGS_DEBUG +=-pg
#QMAKE_LFLAGS_DEBUG+=-pg
#LIBS += -L/usr/local/lib $$system(pkg-config --cflags --libs gtk+-2.0 gthread-2.0 playerclient) -lGL -lGLU -lglut -ljpeg 
LIBS += -L/usr/local/lib $$system(pkg-config --cflags --libs gthread-2.0 playerc++ playercore) -lGL -lGLU -lglut -ljpeg 
INCLUDEPATH += includes gui /usr/include/gtk-2.0
INCLUDEPATH += /usr/lib/gtk-2.0/include
INCLUDEPATH += /usr/include/atk-1.0
INCLUDEPATH += /usr/include/cairo
INCLUDEPATH += /usr/include/pango-1.0
INCLUDEPATH += /usr/include/glib-2.0
INCLUDEPATH += /usr/lib/glib-2.0/include
TEMPLATE = app
CONFIG += release thread
TARGET = bin/CasPlanner
