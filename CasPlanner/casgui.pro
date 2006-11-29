HEADERS +=  modules/CommManager/commmanager.h\
			modules/CommManager/comms.h\
			modules/Parser/configfile.h\
			modules/Navigator/controller.h\
			modules/Rendering/glrender.h\
			modules/CommManager/interfaceprovider.h\
			modules/Rendering/laserrender.h\
			gui/mainwindow.h\
			modules/MapManager/map.h\
			modules/Rendering/mapviewer.h\	
			modules/MapManager/mapmanager.h\
			gui/navigationtab.h\
			modules/Navigator/navigator.h\
			modules/Rendering/ogrender.h\
			modules/CommManager/playerinterface.h\
			modules/PlanningManager/planningmanager.h\
			modules/PlayGround/playground.h\
			modules/PlayGround/robotmanager.h\
			modules/Rendering/robotrender.h\
			modules/Sensors/sensors.h\
			modules/TasksManager/tasksgui.h\
			modules/Rendering/speedrender.h\
			gui/statusbar.h\
			gui/tabcontainer.h\
			modules/PathPlanner/Astar.h\
			modules/GeometricTools/C2DMatrix.h\
			modules/Controller/common.h\
			modules/ObstacleAvoidance/ForceField.h\
			modules/GeometricTools/geometry.h\
			modules/ScanMatching/geometry2D.h\			
			modules/ScanMatching/icp.h\			
			modules/PathPlanner/LList.h\
			modules/ScanMatching/nn.h\			
			modules/PathPlanner/Node.h\
			modules/PathPlanner/PathPlanner.h\
			modules/PathPlanner/Robot.h\
			modules/PathPlanner/SearchSpace.h\
			modules/PathPlanner/SearchSpaceNode.h\
			modules/Misc/Timer.h\
			modules/GeometricTools/Transformations.h\
			modules/GeometricTools/utils.h\
			modules/GeometricTools/Vector2D.h\
			modules/Voronoi/mapskeleton.h\
#			modules/Voronoi/VoronoiDiagramGenerator.h\
			modules/Controller/wheelchairproxy.h

SOURCES +=  modules/CommManager/commmanager.cpp\
			modules/Parser/configfile.cc\
			modules/Navigator/controller.cpp\
			modules/Rendering/laserrender.cpp\
			gui/main.cpp\
			gui/mainwindow.cpp\
			modules/Rendering/mapviewer.cpp\			
			modules/MapManager/mapmanager.cpp\
			gui/navigationtab.cpp\
			modules/Navigator/navigator.cpp\
			modules/Rendering/ogrender.cpp\			
			modules/CommManager/playerinterface.cpp\
			modules/PlanningManager/planningmanager.cpp\
			modules/PlayGround/playground.cpp\
			modules/PlayGround/robotmanager.cpp\
			modules/Rendering/robotrender.cpp\
			modules/Sensors/sensors.cpp\
			modules/TasksManager/tasksgui.cpp\
			modules/Rendering/speedrender.cpp\
			gui/statusbar.cpp\
			gui/tabcontainer.cpp\
			modules/PathPlanner/Astar.cpp\
		    modules/ObstacleAvoidance/ForceField.cpp\
		    modules/ScanMatching/geometry2D.cpp\		    
		    modules/ScanMatching/icp.cpp\		    
		    modules/ScanMatching/nn.cpp\		    
		    modules/PathPlanner/LList.cpp\
		    modules/PathPlanner/Node.cpp\
		    modules/PathPlanner/PathPlanner.cpp\
		    modules/PathPlanner/Robot.cpp\
		    modules/PathPlanner/SearchSpace.cpp\
		    modules/PathPlanner/SearchSpaceNode.cpp\
		    modules/Misc/Timer.cpp\
  			modules/Voronoi/mapskeleton.cpp\
#			modules/Voronoi/VoronoiDiagramGenerator.cpp\
		    modules/GeometricTools/Vector2d.cpp
		    
RESOURCES = resources/icons.qrc
QT += opengl
QMAKE_CFLAGS_RELEASE+= -g -O3 -o 
QMAKE_CFLAGS_RELEASE+= $$system(soqt-config --cflags )
#QMAKE_CFLAGS_RELEASE+= -g -O3 -ffast-math -march=pentium-m -msse2 -mfpmath=sse 
#QMAKE_CXXFLAGS_RELEASE+= -g -O3 -ffast-math -march=pentium-m -msse2 -mfpmath=sse
#QMAKE_LFLAGS_RELEASE += -g -O3 -ffast-math -march=pentium-m -msse2 -mfpmath=sse
#QMAKE_CFLAGS_DEBUG +=-pg
#QMAKE_CXXFLAGS_DEBUG +=-pg
#QMAKE_LFLAGS_DEBUG+=-pg
# Player 1.6.5
	#LIBS += -L/usr/local/lib $$system(pkg-config --cflags --libs gtk+-2.0 gthread-2.0 playerclient) -lGL -lGLU -lglut -ljpeg 
# Player 2.0
LIBS += -L/usr/local/lib $$system(pkg-config --cflags --libs gthread-2.0 playerc++ playercore) -lGL -lGLU -lglut -ljpeg 
LIBS += -lCGAL

# Coin3d + SoQt until now they don't support QT4 so i have to ignore it
# LIBS += -lSoQt -lCoin -lbz2 -lz -lfreetype -lfontconfig -ldl -lpthread -lm -lXi

INCLUDEPATH += modules/PathPlanner modules/ObstacleAvoidance includes gui /usr/include/gtk-2.0
INCLUDEPATH += modules/ScanMatching modules/Misc modules/GeometricTools modules/Controller
INCLUDEPATH += modules/CommManager modules/MapManager modules/Navigator modules/Parser modules/PlanningManager
INCLUDEPATH += modules/PlayGround modules/Rendering modules/Sensors modules/Voronoi modules/TasksManager
INCLUDEPATH += /usr/lib/gtk-2.0/include
INCLUDEPATH += /usr/include/atk-1.0
INCLUDEPATH += /usr/include/cairo
INCLUDEPATH += /usr/include/pango-1.0
INCLUDEPATH += /usr/include/glib-2.0
INCLUDEPATH += /usr/include/Coin2
INCLUDEPATH += /usr/lib/glib-2.0/include
TEMPLATE = app
CONFIG += release thread
TARGET = bin/CasPlanner
