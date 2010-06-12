project(haartraining)

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

IF(OPENCV_FOUND)
	#do something ?
ELSE(OPENCV_FOUND)
	MESSAGE("Cannot find opencv version 2.1 or above")
ENDIF(OPENCV_FOUND)

INCLUDE_DIRECTORIES( 
    ${CMAKE_BINARY_DIR}
    ${OPENCV_INCLUDE_DIRS}
    .
)

# -----------------------------------------------------------
#  Library
# -----------------------------------------------------------
SET(cvhaartraining_lib_src
    _cvcommon.h
    cvclassifier.h
    _cvhaartraining.h
    cvhaartraining.h
    cvboost.cpp
    cvcommon.cpp
    cvhaarclassifier.cpp
    cvhaartraining.cpp
    cvsamples.cpp
    )

ADD_LIBRARY(cvhaartraining STATIC ${cvhaartraining_lib_src})

INSTALL(FILES
        _cvcommon.h
        cvclassifier.h
        _cvhaartraining.h
        cvhaartraining.h
        DESTINATION
        /usr/local/include/visionclassification
        )
