#include "Node.h"
#include <opencv2/opencv.hpp>




class Dijkstra {
public:

  Dijkstra();
  ~Dijkstra();


  void setCostmap();
  void createGraph();

  bool pathFinding(const Eigen::Vector2i& start, const Eigen::Vector2i& end, std::vector<Eigen::Vector2i>& path);



  cv::Mat costmap_; // save map data, 255 is free, 0 is occupied
  size_t width_;
  size_t height_;


private:

  Graph  graph_;
  NodeQueue  open_list_;
  std::vector<Node> close_list_;
};