cmake_minimum_required(VERSION 3.7)

#===============================================================================
# MY PROJECT
#===============================================================================

# enable C++11
enable_language(C)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 11)

# create project
project(SparGlove)

# set binary output location to conventional "bin" (optional, but recommended)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

# include directories
include_directories(
    "include"              # your include directory
    "/Git/MEL/include" # MEL's include directory
)

# indicate where MEL.lib is
link_directories("/Git/MEL/build/Release")

# remove or uncomment if linking MEL statically
# add_definitions(-DMEL_STATIC)

# create application
add_executable(spar_glove
               src/main.cpp)

# link MEL
target_link_libraries(spar_glove MEL)
