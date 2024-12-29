cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 23)

project(scp)

add_subdirectory(src/olc)
add_subdirectory(src/threadpool)

add_compile_options(-march=native)
add_compile_options(-Wall -Wextra)

add_compile_options()
add_link_options()

set(SOURCES
    src/Main.cpp
    src/ControlNetwork.cpp
    src/Vec2.cpp
    src/PhysicsSim.cpp
    src/MainWindow.cpp
    src/TrainingSim.cpp
)

add_executable(scp ${SOURCES})

target_link_libraries(scp PRIVATE olcpge)
target_link_libraries(scp PRIVATE threadpool)

if (LINUX OR APPLE)
    target_link_libraries(scp PRIVATE -lX11 -lGL -lpthread -lpng -pg)
endif()