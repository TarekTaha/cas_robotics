
SET( SIFT_HDRS
  imgfeatures.h  
  kdtree.h  
  minpq.h  
  sift.h  
  utils.h  
  xform.h
)

SET( SIFT_SRCS
  imgfeatures.cpp  
  kdtree.cpp  
  minpq.cpp  
  sift.cpp  
  utils.cpp  
  xform.cpp
)

IF(CMAKE_COMPILER_IS_GNUCC)
    SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -g -O0 -o -fmessage-length=0")
ENDIF(CMAKE_COMPILER_IS_GNUCC)

IF(CMAKE_COMPILER_IS_GNUCXX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -o -fmessage-length=0")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# Optimization Level
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math -march=pentium-m -msse2 -mfpmath=sse")

# enable warnings
ADD_DEFINITIONS( -Wall )

INCLUDE( ${CMAKE_ROOT}/Modules/FindPkgConfig.cmake )
INCLUDE( ${CMAKE_ROOT}/Modules/UsePkgConfig.cmake )

IF(PKG_CONFIG_FOUND)
	pkg_check_modules(OPENCV REQUIRED opencv)
ELSE(PKG_CONFIG_FOUND)
	MESSAGE("Cannot find pkg-config")
ENDIF(PKG_CONFIG_FOUND)

IF(PKG_CONFIG_FOUND)
        pkg_check_modules(GTK REQUIRED gtk+-2.0)
ELSE(PKG_CONFIG_FOUND)
        MESSAGE("Cannot find pkg-config")
ENDIF(PKG_CONFIG_FOUND)

IF(GTK_FOUND)
	#do something ?
ELSE(GTK_FOUND)
	MESSAGE("Cannot find opencv version 2.1 or above")
ENDIF(GTK_FOUND)

INCLUDE_DIRECTORIES( 
    ${CMAKE_BINARY_DIR}
    ${GTK_INCLUDE_DIRS}
    ${OPENCV_INCLUDE_DIRS}
    .
)

ADD_LIBRARY( sift STATIC ${SIFT_SRCS} ${SIFT_HDRS} )

TARGET_LINK_LIBRARIES( sift
    ${OPENCV_LIBRARIES}
    ${GTK_LIBRARIES}
)

INSTALL(FILES
        imgfeatures.h
        kdtree.h
        minpq.h
        sift.h
        utils.h
        xform.h
        DESTINATION
        /usr/local/include/visionclassification
        )
