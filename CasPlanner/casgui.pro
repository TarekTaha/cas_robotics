
HEADERS += modules/CommManager/commmanager.h \
           modules/CommManager/comms.h \
           modules/Parser/configfile.h \
           modules/Navigator/controller.h \
           modules/Rendering/glrender.h \
           modules/CommManager/interfaceprovider.h \
           modules/Rendering/laserrender.h \
           gui/mainwindow.h \
           modules/MapManager/map.h \
           modules/Rendering/mapviewer.h \
           modules/MapManager/mapmanager.h \
           gui/navigationtab.h \
           modules/Navigator/navigator.h \
           modules/Rendering/ogrender.h \
           modules/CommManager/playerinterface.h \
           modules/PathPlanner/planningmanager.h \
           modules/PlayGround/playground.h \
           modules/PlayGround/robotmanager.h \
           modules/Rendering/robotrender.h \
           modules/Sensors/sensors.h \
           modules/TasksManager/hmm.h \           
           modules/TasksManager/logprobs.h \           
           modules/TasksManager/str2idmap.h \           
           modules/TasksManager/tables.h \           
           modules/TasksManager/task.h \
           modules/TasksManager/tasksgui.h \
           modules/Rendering/speedrender.h \
           gui/playgroundtab.h \
           gui/statusbar.h \
           gui/tabcontainer.h \
           modules/PathPlanner/astar.h \
           modules/GeometricTools/C2DMatrix.h \
           modules/Controller/common.h \
           modules/ObstacleAvoidance/forcefield.h \
           modules/GeometricTools/geometry.h \
           modules/ScanMatching/geometry2D.h \
           modules/ScanMatching/icp.h \
           modules/PathPlanner/llist.h \
           modules/ScanMatching/nn.h \
           modules/PathPlanner/node.h \
           modules/PathPlanner/pathplanner.h \
           modules/PathPlanner/robot.h \
           modules/PathPlanner/searchspace.h \
           modules/PathPlanner/searchspaceNode.h \
           modules/PathPlanner/voronoipathplanner.h \
           modules/Misc/timer.h \
           modules/GeometricTools/Transformations.h \
           modules/GeometricTools/utils.h \
           modules/GeometricTools/Vector2D.h \
           modules/Voronoi/mapskeleton.h \
           modules/Voronoi/mrfmodel.h \           
           modules/Voronoi/typedefs.h \                      
           modules/Voronoi/voronoidiagram.h \           
           modules/Controller/wheelchairproxy.h 
           
SOURCES += modules/CommManager/commmanager.cpp \
           modules/Parser/configfile.cc \
           modules/Navigator/controller.cpp \
           modules/Rendering/laserrender.cpp \
           gui/main.cpp \
           gui/mainwindow.cpp \
           modules/Rendering/mapviewer.cpp \
           modules/MapManager/mapmanager.cpp \
           gui/navigationtab.cpp \
           modules/Navigator/navigator.cpp \
           modules/Rendering/ogrender.cpp \
           modules/CommManager/playerinterface.cpp \
           modules/PathPlanner/planningmanager.cpp \
           modules/PlayGround/playground.cpp \
           modules/PlayGround/robotmanager.cpp \
           modules/Rendering/robotrender.cpp \
           modules/Sensors/sensors.cpp \
           modules/TasksManager/hmm.cpp \           
           modules/TasksManager/logprobs.cpp \           
           modules/TasksManager/tables.cpp \             
           modules/TasksManager/task.cpp \
           modules/TasksManager/tasksgui.cpp \
           modules/Rendering/speedrender.cpp \
           gui/playgroundtab.cpp \            
           gui/statusbar.cpp \
           gui/tabcontainer.cpp \
           modules/PathPlanner/astar.cpp \
           modules/ObstacleAvoidance/forcefield.cpp \
           modules/ScanMatching/geometry2D.cpp \
           modules/ScanMatching/icp.cpp \
           modules/ScanMatching/nn.cpp \
           modules/PathPlanner/llist.cpp \
           modules/PathPlanner/node.cpp \
           modules/PathPlanner/pathplanner.cpp \
           modules/PathPlanner/robot.cpp \
           modules/PathPlanner/searchspace.cpp \
           modules/PathPlanner/searchspacenode.cpp \
           modules/PathPlanner/voronoipathplanner.cpp \
           modules/Misc/timer.cpp \
           modules/Voronoi/mapskeleton.cpp \           
           modules/Voronoi/mrfmodel.cpp \                      
           modules/Voronoi/voronoidiagram.cpp \                      
           modules/GeometricTools/Vector2d.cpp 
           
INCLUDEPATH += 	/usr/local/Trolltech/Qt-4.2.2/include/QtCore \
				/usr/local/Trolltech/Qt-4.2.2/include/QtGui \
				/usr/local/high/include \		
				/usr/local/include/opencx \
				/usr/lib/gtk-2.0/include \
				/usr/include/atk-1.0 \
				/usr/include/cairo \
				/usr/include/pango-1.0 \
				/usr/include/glib-2.0 \
				/usr/include/Coin2 \
				/usr/lib/glib-2.0/include \
				/usr/include/gtk-2.0 \		
				modules/PathPlanner \
				modules/ObstacleAvoidance \
				modules/ScanMatching \
				modules/Misc \
				modules/GeometricTools \
				modules/Controller \
				modules/CommManager \
				modules/MapManager \
				modules/Navigator \
				modules/Parser \
				modules/PlanningManager \
				modules/PlayGround \
				modules/Rendering \
				modules/Sensors \
				modules/Voronoi \
				modules/TasksManager \
				gui
RESOURCES = resources/icons.qrc
QT += opengl
QMAKE_CFLAGS_RELEASE+= -g -O3 -o 
QMAKE_CXXFLAGS_RELEASE+= -g -O3 -o -ffast-math -march=pentium-m -msse2 -mfpmath=sse
#QMAKE_CFLAGS_RELEASE+= -g -O3 -ffast-math -march=pentium-m -msse2 -mfpmath=sse 
#QMAKE_LFLAGS_RELEASE += -g -O3 -ffast-math -march=pentium-m -msse2 -mfpmath=sse
LIBS += $$system(pkg-config --cflags --libs gthread-2.0 playerc++ playercore) -lCGAL
LIBS +=  -lhigh -lpnl /usr/local/lib/libcxcore.a
MOC_DIR = .tmp
OBJECTS_DIR = .tmp
RCC_DIR = .tmp
TARGET = CasPlanner
CONFIG += 	release \
			warn1_on \
			qt \
			opengl \
			thread \
			exceptions \
			stl
TEMPLATE = app
