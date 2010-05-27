#
# Test CMake version
#
cmake_minimum_required( VERSION 2.6 FATAL_ERROR )

#
# project name
#
project( VisionClassification )

#
# project version
#
set( VC_PROJECT_VERSION_MAJOR "1" )
set( VC_PROJECT_VERSION_MINOR "0" )
set( VC_PROJECT_VERSION_PATCH "0" )

set( VC_PROJECT_VERSION
    ${VC_PROJECT_VERSION_MAJOR}.${VC_PROJECT_VERSION_MINOR}.${VC_PROJECT_VERSION_PATCH} )

#
# Give feedback on custom entries
#
message( STATUS "Setting project name to ${PROJECT_NAME}" )
message( STATUS "Setting project version to ${VC_PROJECT_VERSION}" )

#
# Set CMake policies
# For help on policy CMPxxxx: $ cmake --help-policy CMPxxxx
# 
if( COMMAND cmake_policy )
    if( POLICY CMP0011 )
        cmake_policy(SET CMP0011 NEW )
    endif( POLICY CMP0011 )
endif( COMMAND cmake_policy )

add_subdirectory( src/SIFT )
add_subdirectory( src )
