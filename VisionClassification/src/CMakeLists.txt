
SET( VC_HDRS
  BackgroundClassification/BackgroundSubtraction.h
  BrightnessClassification/BrightnessClassifier.h
  CamshiftClassification/CamshiftClassifier.h
  ClassifierCore/Classifier.h
  ClassifierCore/ClassifierOutputData.h
  ClassifierCore/ClassifierTester.h
  ColorClassification/ColorClassifier.h
  Gesture/FlowTracker.h
  Gesture/OneDollar.h
  Gesture/SimpleFlowTracker.h
  GestureClassification/GestureClassifier.h
  HaarClassification/HaarClassifier.h
  TesseractClassification/TesseractClassifier.h
  Training/TrainingSample.h
  Training/TrainingSet.h
  MotionClassification/MotionClassifier.h
  ShapeClassification/ShapeClassifier.h
  SiftClassification/SiftClassifier.h
  SurfClassification/SurfClassifier.h
  VideoHelpers/VideoLoader.h
  common/common.h
  common/Graphics.h
  common/constants.h
  common/precomp.h
)

SET( VC_SRCS
  BackgroundClassification/BackgroundSubtraction.cpp
  BrightnessClassification/BrightnessClassifier.cpp
  CamshiftClassification/CamshiftClassifier.cpp
  ClassifierCore/Classifier.cpp
  ClassifierCore/ClassifierOutputData.cpp
  ClassifierCore/ClassifierTester.cpp
  ColorClassification/ColorClassifier.cpp
  Gesture/FlowTracker.cpp
  Gesture/OneDollar.cpp
  Gesture/SimpleFlowTracker.cpp
  GestureClassification/GestureClassifier.cpp
  HaarClassification/HaarClassifier.cpp
  TesseractClassification/TesseractClassifier.cpp
  Training/TrainingSample.cpp
  Training/TrainingSet.cpp
  MotionClassification/MotionClassifier.cpp
  ShapeClassification/ShapeClassifier.cpp
  SiftClassification/SiftClassifier.cpp
  SurfClassification/SurfClassifier.cpp
  VideoHelpers/VideoLoader.cpp
  common/Graphics.cpp
  common/precomp.cpp
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

IF(OPENCV_FOUND)
	#do something ?
ELSE(OPENCV_FOUND)
	MESSAGE("Cannot find opencv version 2.1 or above")
ENDIF(OPENCV_FOUND)

FIND_PACKAGE(Boost COMPONENTS system filesystem REQUIRED)
#FIND_PACKAGE(Tesseract COMPONENTS tesseract REQUIRED)

INCLUDE_DIRECTORIES( 
    ${CMAKE_BINARY_DIR}
    ${OPENCV_INCLUDE_DIRS}
    ${Boost_FILESYSTEM_DIRS}
    ${Boost_SYSTEM_DIRS}
  common
  SIFT
  ClassifierCore
  CVHaar
  Gesture
  GestureClassification
  MotionClassification
  TesseractClassification
  Training
  VideoHelpers
   /usr/include/tesseract
)



ADD_LIBRARY( visualClassification SHARED ${VC_SRCS})

TARGET_LINK_LIBRARIES( visualClassification  
    ${OPENCV_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    sift
    tesseract_full
    cvhaartraining
)
