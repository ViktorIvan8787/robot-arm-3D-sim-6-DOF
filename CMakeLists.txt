set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_minimum_required(VERSION 3.14)
project(MyRaylibApp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_TAG        5.0
)
FetchContent_MakeAvailable(raylib)

add_executable(simulation 
  simulation.cpp
  src/kinematics.cpp
)

target_link_libraries(simulation PRIVATE raylib)