##---------------------------------------------------------------------------
## Author:      Andreas Ziegler
##---------------------------------------------------------------------------
 
# define minimum cmake version
cmake_minimum_required(VERSION 2.8)

# Our project is called 'Stretcher' this is how it will be called in
# visual studio, and in our makefiles.
project( SP1 )

set(CMAKE_BINARY_DIR "${CMAKE_SOURCE_DIR}/build")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# Location where cmake first looks for cmake modules.
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}" "${CMAKE_SOURCE_DIR}/cmake/Modules")

find_package( OpenCV REQUIRED )

add_executable( SP1 test.cpp )
target_link_libraries( SP1 ${OpenCV_LIBS} )
