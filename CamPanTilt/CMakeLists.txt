# set project's name
PROJECT( PanTilt )

cmake_minimum_required(VERSION 2.4)


SET( PanTilt_SRCS
    ./src/PanTilt.cpp
    ./src/test.cpp
)

# CDT Error Parser cannot handle error messages that span more than one line
# which is the default gcc behavior. This is used to force gcc to generate single
# line error messages with no line wrapping
IF(CMAKE_COMPILER_IS_GNUCC)
    SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -g -O0 -o -fmessage-length=0")
ENDIF(CMAKE_COMPILER_IS_GNUCC)

IF(CMAKE_COMPILER_IS_GNUCXX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -o -fmessage-length=0")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# Optimization Level
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math -msse2 -mfpmath=sse")

# enable warnings
ADD_DEFINITIONS( -Wall )

##################
#    QT STUFF    #
##################

ADD_DEFINITIONS( ${QT_DEFINITIONS} )
FIND_PACKAGE( Qt4 REQUIRED )

SET(QT_USE_QTGUI TRUE )
SET(QT_USE_QTOPENGL TRUE )
SET(QT_USE_XML TRUE )
SET(QT_USE_QTXMLPATTERNS TRUE)

INCLUDE( ${QT_USE_FILE} )

QT4_ADD_RESOURCES( PanTilt_RC_SRCS ${PanTilt_RCS} )

# this will run uic on .ui files:
QT4_WRAP_UI( PanTilt_UI_HDRS ${PanTilt_UIS} )

# and finally this will run moc:
QT4_WRAP_CPP( PanTilt_MOC_SRCS ${PanTilt_MOC_HDRS} )


##################
#  OPENGL STUFF  #
##################
INCLUDE(${CMAKE_ROOT}/Modules/FindOpenGL.cmake)
 
# we need this to be able to include headers produced by uic in our code
# (CMAKE_BINARY_DIR holds a path to the build directory, while INCLUDE_DIRECTORIES() works just like INCLUDEPATH from qmake)
INCLUDE_DIRECTORIES( 
    ${QT_INCLUDES} 
	${OPENGL_INCLUDE_DIR}
    ${CMAKE_BINARY_DIR}
	${PLAYER_INCLUDE_DIRS}    
    ./src
)

INCLUDE(${CMAKE_ROOT}/Modules/FindSDL.cmake)

# here we instruct CMake to build "testPanTilt" executable from all of the source files
ADD_EXECUTABLE( ../testPanTilt ${PanTilt_SRCS} ${PanTilt_MOC_SRCS} ${PanTilt_RC_SRCS} ${PanTilt_UI_HDRS} )

TARGET_LINK_LIBRARIES( ../testPanTilt 
    ${QT_QTGUI_LIBRARY} 
    ${QT_QTCORE_LIBRARY} 
    ${QT_QTOPENGL_LIBRARY}
    ${QT_QTXML_LIBRARY}	
    ${QT_LIBRARIES}
    ${SDL_LIBRARY}
)
