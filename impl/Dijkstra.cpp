#include "Dijkstra.h"

Dijkstra::Dijkstra(){}


Dijkstra::~Dijkstra(){}

void Dijkstra::setCostmap() {
  // Mat(rows, cols, type)
  costmap_  = cv::Mat(48, 64, CV_8UC1, cv::Scalar(255)).clone();

  width_ = costmap_.cols;
  height_ = costmap_.rows;


  std::cout << "width_ " << width_ << ", height " << height_ << std::endl;


  // set border to obstacle
  for(int i = 0; i < width_; ++i) {

    //at(i,j), i is y coord; j is x coord
    costmap_.at<uchar>(0, i) = 0;   // 第0行
    costmap_.at<uchar>(height_ - 1, 0) = 0; //最后一行
  }


  for(int j=0; j < height_; ++j) {

    costmap_.at<uchar>(j, 0) = 0;                //最左边行
    costmap_.at<uchar>(j, width_-1) = 0;  //最后边行
  }

  // set obs
  // 左边竖线
  for(int k =0; k < 41; ++k) {
    costmap_.at<uchar>(k, 20) = 0;
  }


  // 右边竖线
  for(int v=7; v < 48; ++v) {
    costmap_.at<uchar>(v, 40) = 0;
  }

  cv::imwrite("/home/limao/dij0.png", costmap_);


  // auto ellipse = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
  // cv::Mat dilated;
  // cv::dilate(costmap_, dilated, ellipse );
  // cv::imwrite("/home/limao/dilate0.png", dilated);
}


void Dijkstra::createGraph()
{
  for(int j =0; j < height_; ++j) {

    for(int i=0; i < width_; ++i) {
        std::cout << "i" << i << " j" << j << std::endl;

        graph_.push_back(new Node(Eigen::Vector2i(i, j), costmap_.at<uchar>(j, i)));
    }
  }

  // y_index * width_  + x_index = one-dim-index
  // std::cout << "graph size: "  << graph_.size() << ", 400-200: "<< static_cast<double>(graph_[width_ * 400 + 200]->cost_) << std::endl;
  // std::cout << graph_[width_ * 400 + 200]->coord_i_(0) << " " <<  graph_[width_ * 400 + 200]->coord_i_(1) << std::endl;
  // std::cout << "graph 400-200 cost " << static_cast<double>(costmap_.at<uchar>(400, 200)) << std::endl;
}


bool Dijkstra::pathFinding(const Eigen::Vector2i& start, const Eigen::Vector2i& end, std::vector<Eigen::Vector2i>& path) {
    NodePtr start_ptr = graph_[start(1) * width_ + start(0)];

    NodePtr end_ptr = graph_[end(1) * width_ + end(0)];


    std::cout << static_cast<int>(start_ptr->cost_) << std::endl;
    std::cout << static_cast<int>(end_ptr->cost_) << std::endl;


    while(!open_list_.empty())  open_list_.pop();

    open_list_.push(start_ptr);

    size_t iteration_count = 0;

    while(!open_list_.empty()) {

      std::cout << "-";

    }


    return true;
}