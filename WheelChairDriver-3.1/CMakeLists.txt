cmake_minimum_required(VERSION 2.6)

SET (WheelChairDriverSrcs
	SerialCom.cpp
	wheelchair.cpp
	WheelchairDriver.cpp
	WheelEncoder.cpp
)

find_package(PkgConfig REQUIRED)

pkg_check_modules(PLAYERSTAGE REQUIRED playercore playerc playerinterface playerdrivers playerc++ playercommon)
IF(PLAYERSTAGE_FOUND)
        #do something ?
ELSE(PLAYERSTAGE_FOUND)
        MESSAGE("Cannot find PlayerStage version 3.0 or above")
ENDIF(PLAYERSTAGE_FOUND)

ADD_LIBRARY(WheelChairDriver SHARED ${WheelChairDriverSrcs})

Message(" player stage: ${PLAYERSTAGE_LIBRARIES}")

TARGET_LINK_LIBRARIES(WheelChairDriver
  ${PLAYERSTAGE_LIBRARIES}
)

INCLUDE_DIRECTORIES(
    ${PLAYERSTAGE_INCLUDE_DIRS}
    ${CMAKE_BINARY_DIR}
  .
)
