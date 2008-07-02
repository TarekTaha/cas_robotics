# set project's name
PROJECT( CasPlanner )

SET( CasPlanner_MOC_HDRS
    ./modules/CommManager/commmanager.h 
    ./modules/CommManager/comms.h 
    ./modules/Rendering/glrender.h 
    ./modules/Rendering/laserrender.h 
    ./gui/hritab.h 
    ./gui/mainwindow.h 
    ./gui/navigationtab.h 
    ./gui/planningtab.h 
    ./gui/playgroundtab.h 
    ./gui/statusbar.h 
    ./gui/tabcontainer.h 
    ./modules/Rendering/mapviewer.h 
    ./modules/Navigator/navigator.h 
    ./modules/Rendering/ogrender.h 
    ./modules/CommManager/playerinterface.h 
    ./modules/PathPlanner/planningmanager.h 
    ./modules/PlayGround/playground.h 
    ./modules/PlayGround/robotmanager.h 
    ./modules/Rendering/robotrender.h 
    ./modules/Sensors/sensors.h
    ./modules/TasksManager/tasksgui.h 
)

SET( CasPlanner_SRCS
    ./modules/CommManager/commmanager.cpp 
    ./modules/CommonTools/bandmat.cpp 
    ./modules/CommonTools/cholesky.cpp 
    ./modules/CommonTools/evalue.cpp 
    ./modules/CommonTools/fft.cpp 
    ./modules/CommonTools/hholder.cpp 
    ./modules/CommonTools/jacobi.cpp 
    ./modules/CommonTools/myexcept.cpp 
    ./modules/CommonTools/newfft.cpp 
    ./modules/CommonTools/newmat1.cpp 
    ./modules/CommonTools/newmat2.cpp 
    ./modules/CommonTools/newmat3.cpp 
    ./modules/CommonTools/newmat4.cpp 
    ./modules/CommonTools/newmat5.cpp 
    ./modules/CommonTools/newmat6.cpp 
    ./modules/CommonTools/newmat7.cpp 
    ./modules/CommonTools/newmat8.cpp 
    ./modules/CommonTools/newmat9.cpp 
    ./modules/CommonTools/newmatex.cpp 
    ./modules/CommonTools/newmatnl.cpp 
    ./modules/CommonTools/newmatrm.cpp 
    ./modules/CommonTools/nm_misc.cpp 
    ./modules/CommonTools/solution.cpp 
    ./modules/CommonTools/sort.cpp 
    ./modules/CommonTools/submat.cpp 
    ./modules/CommonTools/svd.cpp 
    ./modules/Parser/configfile.cc 
    ./modules/Navigator/controller.cpp 
    ./modules/Rendering/laserrender.cpp 
    ./gui/hritab.cpp 
    ./gui/main.cpp 
    ./gui/mainwindow.cpp 
    ./gui/navigationtab.cpp 
    ./gui/planningtab.cpp 
    ./gui/playgroundtab.cpp 
    ./gui/statusbar.cpp 
    ./gui/tabcontainer.cpp 
    ./modules/Rendering/mapviewer.cpp 
    ./modules/MapManager/mapmanager.cpp 
    ./modules/Navigator/navigator.cpp 
    ./modules/Rendering/ogrender.cpp 
    ./modules/CommManager/playerinterface.cpp 
    ./modules/PathPlanner/planningmanager.cpp 
    ./modules/PlayGround/playground.cpp 
    ./modules/PlayGround/robotmanager.cpp 
    ./modules/Rendering/robotrender.cpp 
    ./modules/Sensors/sensors.cpp 
    ./modules/TasksManager/task.cpp 
    ./modules/TasksManager/tasksgui.cpp 
    ./modules/Rendering/speedrender.cpp 
    ./modules/PathPlanner/astar.cpp 
    ./modules/ObstacleAvoidance/forcefield.cpp 
    ./modules/ScanMatching/geometry2D.cpp 
    ./modules/ScanMatching/icp.cpp 
    ./modules/ScanMatching/nn.cpp 
    ./modules/PathPlanner/llist.cpp 
    ./modules/PathPlanner/node.cpp 
    ./modules/PathPlanner/pathplanner.cpp 
    ./modules/PathPlanner/robot.cpp 
    ./modules/PathPlanner/searchspace.cpp 
    ./modules/PathPlanner/searchspacenode.cpp 
    ./modules/PathPlanner/voronoipathplanner.cpp 
    ./modules/Misc/timer.cpp 
    ./modules/GeometricTools/Vector2d.cpp 
    ./modules/PomdpCore/CassandraModel.cc 
    ./modules/PomdpCore/CassandraParser.cc 
    ./modules/PomdpCore/decision-tree.c 
    ./modules/PomdpCore/imm-reward.c 
    ./modules/PomdpCore/mdp.c 
    ./modules/PomdpCore/Pomdp.cc 
    ./modules/PomdpCore/sparse-matrix.c 
    ./modules/PomdpCore/zmdpCommonTime.cc 
    ./modules/PomdpCore/zmdpCommonTypes.cc 
    ./modules/PomdpCore/zmdpConfig.cc 
    ./modules/PomdpCore/pomdp_spec.tab.cc 
    ./modules/PomdpCore/pomdp_spec.yy.cc 
    ./modules/PomdpCore/parse_err.c 
    ./modules/PomdpCore/parse_hash.c 
    ./modules/PomdpCore/BoundPairExec.cc 
    ./modules/PomdpCore/MDPExec.cc 
    ./modules/PomdpCore/MaxPlanesLowerBound.cc 
    ./modules/PomdpCore/BoundPairCore.cc 
    ./modules/PomdpCore/BoundPair.cc 
    ./modules/PomdpCore/MDPCache.cc 
    ./modules/PomdpCore/BlindLBInitializer.cc 
    ./modules/MapManager/mapskeleton.cpp 
    ./modules/MapManager/map.cpp 
    ./modules/HumanRobotInteraction/IntentionRecognizer.cpp 
    ./modules/HumanRobotInteraction/activityLogger.cpp 
    ./modules/Controller/wheelchairproxy.cpp 
    ./modules/Rendering/FreeType.cpp 
    ./modules/CommManager/accelerometer.cpp 
    ./modules/PathPlanner/socialplanner.cpp 
    ./modules/ExceptionHandling/casplannerexception.cpp 
    ./modules/PathPlanner/heuristic.cpp
)

# UI .ui files
SET(CasPlanner_UIS gui/hritab.ui )
# Resources files
SET(CasPlanner_RCS resources/icons.qrc)

# Optimization Level
SET(CMAKE_CXX_FLAGS "-O1 -o -ffast-math -march=pentium-m -msse2 -mfpmath=sse")

# CDT Error Parser cannot handle error messages that span more than one line
# which is the default gcc behavior. This is used to force gcc to generate single
# line error messages with no line wrapping
IF(CMAKE_COMPILER_IS_GNUCC)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fmessage-length=0")
ENDIF(CMAKE_COMPILER_IS_GNUCC)

IF(CMAKE_COMPILER_IS_GNUCXX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# enable warnings
ADD_DEFINITIONS( -Wall )

##################
#    QT STUFF    #
##################

ADD_DEFINITIONS( ${QT_DEFINITIONS} )
FIND_PACKAGE( Qt4 REQUIRED )
SET(QT_MT_REQUIRED true)
INCLUDE( ${QT_USE_FILE} )

QT4_ADD_RESOURCES( CasPlanner_RC_SRCS ${CasPlanner_RCS} )

# this will run uic on .ui files:
QT4_WRAP_UI( CasPlanner_UI_HDRS ${CasPlanner_UIS} )

# and finally this will run moc:
QT4_WRAP_CPP( CasPlanner_MOC_SRCS ${CasPlanner_MOC_HDRS} )

SET(QT_USE_QTGUI TRUE )
SET(QT_USE_QTOPENGL TRUE )

##################
#  OPENGL STUFF  #
##################
INCLUDE(${CMAKE_ROOT}/Modules/FindOpenGL.cmake)

##################
#  PLAYER STUFF  #
##################
INCLUDE( ${CMAKE_ROOT}/Modules/FindPkgConfig.cmake )
INCLUDE( ${CMAKE_ROOT}/Modules/UsePkgConfig.cmake )

IF(PKG_CONFIG_FOUND)
	pkg_check_modules(PLAYER REQUIRED playercore playerc++ playerc)
ELSE(PKG_CONFIG_FOUND)
	MESSAGE("Cannot find pkg-config")
ENDIF(PKG_CONFIG_FOUND)

IF(PLAYER_FOUND)
	#do something ?
ELSE(PLAYER_FOUND)
	MESSAGE("Cannot find playerstage version 2.0 or above")
ENDIF(PLAYER_FOUND)
  
# we need this to be able to include headers produced by uic in our code
# (CMAKE_BINARY_DIR holds a path to the build directory, while INCLUDE_DIRECTORIES() works just like INCLUDEPATH from qmake)
INCLUDE_DIRECTORIES( 
	${OPENGL_INCLUDE_DIR}
	${QT_INCLUDES}
	${QT_INCLUDE_DIR} 
	${QT_QTGUI_INCLUDE_DIR}
	${QT_QTCORE_INCLUDE_DIR}
    ${CMAKE_BINARY_DIR}
	${PLAYER_INCLUDE_DIRS}    
    ./modules/PathPlanner 
    ./modules/ObstacleAvoidance 
    ./modules/ScanMatching 
    ./modules/Misc 
    ./modules/GeometricTools 
    ./modules/Controller 
    ./modules/CommManager 
    ./modules/CommonTools/ 
    ./modules/MapManager 
    ./modules/Navigator 
    ./modules/Parser 
    ./modules/PlanningManager 
    ./modules/PlayGround 
    ./modules/Rendering 
    ./modules/Sensors 
    ./modules/TasksManager 
    ./modules/ExceptionHandling 
    ./gui 
    ./modules/PomdpCore 
    ./modules/HumanRobotInteraction 
    /usr/include/freetype2
)

# here we instruct CMake to build "CasPlanner" executable from all of the source files
ADD_EXECUTABLE( CasPlanner ${CasPlanner_SRCS} ${CasPlanner_MOC_SRCS} ${CasPlanner_RC_SRCS} ${CasPlanner_UI_HDRS} )

TARGET_LINK_LIBRARIES( CasPlanner 
    ${QT_QTGUI_LIBRARY} 
    ${QT_QTCORE_LIBRARY} 
    ${QT_QTOPENGL_LIBRARY}	
	${QT_LIBRARIES}
	${PLAYER_LIBRARIES}
	 freetype
	 bluetooth
)
