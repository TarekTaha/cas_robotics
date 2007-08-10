HEADERS += modules/CommManager/commmanager.h \
    modules/CommManager/comms.h \
    modules/CommonTools/controlw.h \
    modules/CommonTools/include.h \
    modules/CommonTools/myexcept.h \
    modules/CommonTools/newmatap.h \
    modules/CommonTools/newmatio.h \
    modules/CommonTools/newmatnl.h \
    modules/CommonTools/newmatrc.h \
    modules/CommonTools/newmatrm.h \
    modules/CommonTools/newmat.h \
    modules/CommonTools/precisio.h \
    modules/CommonTools/solution.h \
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
    modules/GeometricTools/Vector2D.h \ # modules/Voronoi/mapskeleton.h \
# modules/Voronoi/typedefs.h \
# modules/Voronoi/voronoidiagram.h \
    modules/Controller/wheelchairproxy.h \
    modules/PomdpCore/CassandraModel.h \
    modules/PomdpCore/CassandraParser.h \
    modules/PomdpCore/decision-tree.h \
    modules/PomdpCore/imm-reward.h \
    modules/PomdpCore/mdp.h \
    modules/PomdpCore/MatrixUtils.h \
    modules/PomdpCore/MDPModel.h \
    modules/PomdpCore/Pomdp.h \
    modules/PomdpCore/slaMatrixUtils.h \
    modules/PomdpCore/sla.h \
    modules/PomdpCore/sla_cassandra.h \
    modules/PomdpCore/sparse-matrix.h \
    modules/PomdpCore/zmdpCommonDefs.h \
    modules/PomdpCore/zmdpCommonTime.h \
    modules/PomdpCore/zmdpCommonTypes.h \
    modules/PomdpCore/zmdpConfig.h \
    modules/PomdpCore/pomdp_spec.tab.hh \
    modules/PomdpCore/parse_err.h \
    modules/PomdpCore/parse_hash.h \
    modules/PomdpCore/parse_constant.h
SOURCES += modules/CommManager/commmanager.cpp \
    modules/CommonTools/bandmat.cpp \
    modules/CommonTools/cholesky.cpp \
    modules/CommonTools/evalue.cpp \
    modules/CommonTools/fft.cpp \
    modules/CommonTools/hholder.cpp \
    modules/CommonTools/jacobi.cpp \
    modules/CommonTools/myexcept.cpp \
    modules/CommonTools/newfft.cpp \
    modules/CommonTools/newmat1.cpp \
    modules/CommonTools/newmat2.cpp \
    modules/CommonTools/newmat3.cpp \
    modules/CommonTools/newmat4.cpp \
    modules/CommonTools/newmat5.cpp \
    modules/CommonTools/newmat6.cpp \
    modules/CommonTools/newmat7.cpp \
    modules/CommonTools/newmat8.cpp \
    modules/CommonTools/newmat9.cpp \
    modules/CommonTools/newmatex.cpp \
    modules/CommonTools/newmatnl.cpp \
    modules/CommonTools/newmatrm.cpp \
    modules/CommonTools/nm_misc.cpp \
    modules/CommonTools/solution.cpp \
    modules/CommonTools/sort.cpp \
    modules/CommonTools/submat.cpp \
    modules/CommonTools/svd.cpp \
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
    modules/Misc/timer.cpp \ # modules/Voronoi/mapskeleton.cpp \
# modules/Voronoi/voronoidiagram.cpp \
    modules/GeometricTools/Vector2d.cpp \
    modules/PomdpCore/CassandraModel.cc \
    modules/PomdpCore/CassandraParser.cc \
    modules/PomdpCore/decision-tree.c \
    modules/PomdpCore/imm-reward.c \
    modules/PomdpCore/mdp.c \
    modules/PomdpCore/Pomdp.cc \
    modules/PomdpCore/sparse-matrix.c \
    modules/PomdpCore/zmdpCommonTime.cc \
    modules/PomdpCore/zmdpCommonTypes.cc \
    modules/PomdpCore/zmdpConfig.cc \
    modules/PomdpCore/pomdp_spec.tab.cc \
    modules/PomdpCore/pomdp_spec.yy.cc \
    modules/PomdpCore/parse_err.c \
    modules/PomdpCore/parse_hash.c
INCLUDEPATH += /usr/local/Trolltech/Qt-4.2.2/include/QtCore \
    /usr/local/Trolltech/Qt-4.2.2/include/QtGui \
    /usr/local/high/include \
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
    modules/CommonTools/ \
    modules/MapManager \
    modules/Navigator \
    modules/Parser \
    modules/PlanningManager \
    modules/PlayGround \
    modules/Rendering \
    modules/Sensors \ # modules/Voronoi \
    modules/TasksManager \
    gui \
    modules/PomdpCore
RESOURCES = resources/icons.qrc
QT += opengl
QMAKE_CFLAGS_RELEASE += -g \
    -O3 \
    -o
QMAKE_CXXFLAGS_RELEASE += -g \
    -O3 \
    -o \
    -ffast-math \
    -march=pentium-m \
    -msse2 \
    -mfpmath=sse \
    $$system(pkg-config --cflags gthread-2.0 playerc++ playercore)

# QMAKE_CFLAGS_RELEASE+= -g -O3 -ffast-math -march=pentium-m -msse2 -mfpmath=sse
# QMAKE_LFLAGS_RELEASE += -g -O3 -ffast-math -march=pentium-m -msse2 -mfpmath=sse
LIBS += $$system(pkg-config --cflags --libs gthread-2.0 playerc++ playercore) # -lCGAL
MOC_DIR = .tmp
OBJECTS_DIR = .tmp
RCC_DIR = .tmp
TARGET = CasPlanner
CONFIG += release \
    warn1_on \
    qt \
    opengl \
    thread \
    exceptions \
    stl

# CONFIG += no_keywords # so Qt won't #define any non-all-caps `keywords'
# INCLUDEPATH += . /usr/local/include/boost-1_33_1/
# macx:LIBS += /usr/lib/libboost_signals.a # ...your exact paths may vary
TEMPLATE = app
