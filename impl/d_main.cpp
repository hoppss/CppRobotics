#include "Dijkstra.h"



int main() {

  Dijkstra d;
  d.setCostmap();

  d.createGraph();

  Eigen::Vector2i start(10, 10);
  Eigen::Vector2i end(50, 40);

  std::vector<Eigen::Vector2i> path;
  d.pathFinding(start, end, path);
}