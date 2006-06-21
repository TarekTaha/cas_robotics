HEADERS +=  gui/mainwindow.h\
			gui/statusbar.h\
			gui/tabcontainer.h\
			gui/ogrenderer.h\
			gui/sensortab.h\
			gui/navigationtab.h\
			gui/mapviewer.h\
			includes/C2DMatrix.h\
			includes/common.h\
			includes/defs.h\
			includes/ForceField.h\
			includes/geometry.h\
			includes/InvertedAABBox2D.h\
			includes/LList.h\
			includes/Map.h\
			includes/Node.h\
			includes/PathFollower.h\
			includes/PathPlanner.h\
			includes/Point.h\
			includes/Robot.h\
			includes/SearchSpaceNode.h\
			includes/Transformations.h\
			includes/utils.h\
			includes/Vector2D.h\
			includes/Wall2D.h\
			includes/WallIntersectionTests.h\
			includes/wheelchairproxy.h

SOURCES +=  gui/main.cpp\
		    gui/mainwindow.cpp\
		    gui/statusbar.cpp\
		    gui/tabcontainer.cpp\
		    gui/ogrenderer.cpp\
		    gui/sensortab.cpp\
		    gui/navigationtab.cpp\
		    gui/mapviewer.cpp\
		    src/ForceField.cpp\
		    src/LList.cpp\
		    src/Map.cpp\
		    src/Node.cpp\
		    src/PathFollower.cpp\
		    src/PathPlanner.cpp\
		    src/Point.cpp\
		    src/Robot.cpp\
		    src/SearchSpaceNode.cpp\
		    src/Vector2d.cpp
RESOURCES = gui/icons.qrc
QT += opengl
QMAKE_CFLAGS_RELEASE+= -g -O3 -ffast-math -march=pentium-m -msse2 -mfpmath=sse 
QMAKE_CXXFLAGS_RELEASE+= -g -O3 -ffast-math -march=pentium-m -msse2 -mfpmath=sse
QMAKE_LFLAGS_RELEASE += -g -O3 -ffast-math -march=pentium-m -msse2 -mfpmath=sse
QMAKE_CFLAGS_DEBUG +=-pg
QMAKE_CXXFLAGS_DEBUG +=-pg
QMAKE_LFLAGS_DEBUG+=-pg
LIBS += -L/usr/local/lib $$system(pkg-config --cflags --libs gtk+-2.0 gthread-2.0 player) -lGL -lGLU -lglut -ljpeg 
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
