#specify minimum cmake version required
cmake_minimum_required(VERSION 2.8)

#set project name and programming language
project(Soft_neck_Soft_sensor LANGUAGES CXX)

#set c++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

aux_source_directory(main SRC_LIST)

#include libraries directory
add_subdirectory(lib)

INCLUDE_DIRECTORIES(${SUBDIR_INCLUDE_DIRECTORIES})

foreach( testsourcefile ${SRC_LIST} )
    # I used a simple string replace, to cut off .cpp.
    string( REGEX REPLACE "[A-z]+[/]" "" testname ${testsourcefile} )
    string( REPLACE ".cpp" "" testname ${testname} )

    add_executable( ${testname} ${testsourcefile} )
    message ("Including executable: " ${testname})
    # Make sure YourLib is linked to each app
    target_link_libraries( ${testname} ${SUBDIR_LINK_NAMES} )
endforeach( testsourcefile ${APP_SOURCES} )
