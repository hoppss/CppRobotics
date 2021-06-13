add_executable(dij Dijkstra.cpp)
link_directories(${OpenCV_LIBRARY_DIRS})
target_link_libraries(dij ${OpenCV_LIBS})
