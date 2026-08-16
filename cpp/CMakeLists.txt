find_package(raylib QUIET)

if(NOT raylib_FOUND)
    include(FetchContent)

    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

    FetchContent_Declare(
        raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG 5.5
        GIT_SHALLOW TRUE
    )

    FetchContent_MakeAvailable(raylib)
endif()

add_library(robot_arm_core
    src/kinematics.cpp
    src/pathways.cpp
)

target_include_directories(robot_arm_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(robot_arm_core PUBLIC cxx_std_17)
target_link_libraries(robot_arm_core PUBLIC raylib)

add_executable(robot_arm_simulator
    apps/simulation.cpp
)

target_link_libraries(robot_arm_simulator PRIVATE robot_arm_core)

if(MSVC)
    target_compile_options(robot_arm_core PRIVATE /W4)
    target_compile_options(robot_arm_simulator PRIVATE /W4)
else()
    target_compile_options(robot_arm_core PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(robot_arm_simulator PRIVATE -Wall -Wextra -Wpedantic)
endif()
