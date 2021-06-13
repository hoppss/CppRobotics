#include "Node.h"
#include "motion_model.h"

#include <opencv2/opencv.hpp>

#include <string>

#define sqrt2 1.414


class Dijkstra {
public:

  Dijkstra();
  ~Dijkstra();


  void setCostmap();
  void createGraph();

  void getNeighbors(NodePtr current, std::vector<NodePtr>& neighbors, std::vector<double>& costs);

  void initMotionModel();
  double getHeuristic(NodePtr from, NodePtr to);


  bool pathFinding(const Eigen::Vector2i& start, const Eigen::Vector2i& end, std::vector<Eigen::Vector2i>& path);

  bool tracebackPath(NodePtr start, NodePtr end, std::vector<Eigen::Vector2i>& path);
  void setDebug(bool debug) { debug_ = debug;}
  void visualize(int ms, NodePtr p = nullptr);


private:

  cv::Mat costmap_; // save map data, 255 is free, 0 is occupied
  int width_;
  int height_;
  int ns_;

  bool debug_;
  std::string name_;

  MotionModel model_;

  Graph  graph_;
  NodeQueue  open_list_;
  std::vector<NodePtr> close_list_;
  std::vector<Eigen::Vector2d>  motion_model_;
};