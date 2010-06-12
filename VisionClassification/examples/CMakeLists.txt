MACRO(new_executable exec)
        ADD_EXECUTABLE(${exec} ${exec}.cpp)
        TARGET_LINK_LIBRARIES(${exec}
                              ${OPENCV_LIBRARIES}
                              ${Boost_FILESYSTEM_LIBRARY}
                              ${Boost_SYSTEM_LIBRARY}
                              visualClassification
                             )
        INSTALL(PROGRAMS ${exec} DESTINATION bin)
ENDMACRO(new_executable exec)

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

FIND_PACKAGE(Boost COMPONENTS system filesystem REQUIRED)

new_executable(backgroundSubtraction)
new_executable(brightnessClassification)
new_executable(textOCR)
new_executable(trainFindObject)

INCLUDE_DIRECTORIES( 
    ${CMAKE_BINARY_DIR}
    ${OPENCV_INCLUDE_DIRS}
    ${Boost_FILESYSTEM_DIRS}
    ${Boost_SYSTEM_DIRS}
    ../src/BackgroundClassification
    ../src/BrightnessClassification
    ../src/common
    ../src/SIFT
    ../src/SiftClassification
    ../src/ShapeClassification
    ../src/ClassifierCore
    ../src/ColorClassification
    ../src/CVHaar
    ../src/Gesture
    ../src/GestureClassification
    ../src/MotionClassification
    ../src/TesseractClassification
    ../src/Training
    ../src/VideoHelpers
     /usr/include/tesseract
)

