set(CMAKE_CXX_STANDARD 23)


add_library(threadpool ThreadPool.cpp)
target_include_directories(threadpool PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
