add_executable(mydijkstra Dijkstra.cpp)
link_directories(${OpenCV_LIBRARY_DIRS})
target_link_libraries(mydijkstra ${OpenCV_LIBS})


add_executable(myastar AStar.cpp)
target_link_libraries(myastar ${OpenCV_LIBS})