add_executable(dij Dijkstra.cpp d_main.cpp)
link_directories(${OpenCV_LIBRARY_DIRS})
target_link_libraries(dij ${OpenCV_LIBS})
