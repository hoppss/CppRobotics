#include "Dijkstra.h"
#include <thread>

Dijkstra::Dijkstra(){
  debug_  = true;
  name_= "dijkstra";
  cv::namedWindow(name_, CV_WINDOW_NORMAL);


  model_  = MotionModel::VON_NEUMANN;
  // model_  = MotionModel::MOORE;
}


Dijkstra::~Dijkstra(){
  for(auto it = graph_.begin(); it != graph_.end(); ++it) {
    if(*it != nullptr){
      delete (*it);
      (*it) = nullptr;
    }
  }

  motion_model_.clear();

  std::cout << "deconstructor" << std::endl;
}

void Dijkstra::setCostmap() {
  // Mat(rows, cols, type)
  costmap_  = cv::Mat(48, 64, CV_8UC1, cv::Scalar(255)).clone();

  width_ = costmap_.cols;
  height_ = costmap_.rows;
  ns_ = width_ * height_;


  std::cout << "width_ " << width_ << ", height " << height_ << ", ns " << ns_ << std::endl;


  // set border to obstacle, 0 is occupied, 254 is free
  for(int i = 0; i < width_; ++i) {

    //at(i,j), i is y coord; j is x coord
    costmap_.at<uchar>(0, i) = 0;   // 第0行
    costmap_.at<uchar>(height_ - 1, i) = 0; //最后一行
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

  visualize(10000);

  initMotionModel();  // init map firstly, then init motionModel;

  cv::imwrite("/home/limao/dij0.png", costmap_);


  // auto ellipse = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
  // cv::Mat dilated;
  // cv::dilate(costmap_, dilated, ellipse );
  // cv::imwrite("/home/limao/dilate0.png", dilated);
}

// 这一版本都是指针操作，为了节省开销，也不知道节省多少
void Dijkstra::createGraph()
{
  for(int j =0; j < height_; ++j) {

    for(int i=0; i < width_; ++i) {
        // std::cout << "i" << i << " j " << j << std::endl;

        graph_.push_back(new Node( i+j*width_, Eigen::Vector2i(i,j), costmap_.at<uchar>(j, i)));
    }
  }

  // y_index * width_  + x_index = one-dim-index
  // std::cout << "graph size: "  << graph_.size() << ", 400-200: "<< static_cast<double>(graph_[width_ * 400 + 200]->cost_) << std::endl;
  // std::cout << graph_[width_ * 400 + 200]->coord_i_(0) << " " <<  graph_[width_ * 400 + 200]->coord_i_(1) << std::endl;
  // std::cout << "graph 400-200 cost " << static_cast<double>(costmap_.at<uchar>(400, 200)) << std::endl;
}

void Dijkstra::initMotionModel(){
  motion_model_.emplace_back(1.0, 1.0);  // 右边
  motion_model_.emplace_back(-1.0, 1.0); // 左边
  motion_model_.emplace_back(static_cast<double>(width_), 1.0); // 下
  motion_model_.emplace_back(static_cast<double>(-width_), 1.0);  // 上
  // for(const auto & i : motion_model_) std::cout << i << std::endl;

  if(model_ == MotionModel::MOORE) {
    std::cout << "MOORE 8-neighbors" << std::endl;
    motion_model_.emplace_back(static_cast<double>(-width_ - 1), sqrt2);  // 左上
    motion_model_.emplace_back(static_cast<double>(-width_ + 1), sqrt2); // 右上
    motion_model_.emplace_back(static_cast<double>(width_ -1), sqrt2);  // 左下
    motion_model_.emplace_back(static_cast<double>(width_+1), sqrt2); // 右下
  }

}


void Dijkstra::getNeighbors(NodePtr current, std::vector<NodePtr>& neighbors, std::vector<double>& costs){

  for(auto it = motion_model_.begin(); it != motion_model_.end(); ++it) {
    int i =current->getIndex() + static_cast<int>((*it)(0));
    std::cout << "getNeighbors " << i << " motion: " << (*it)(0) << ", cost " << (*it)(1) << std::endl;
     if( i < ns_ &&  graph_[i]->cost_) {
       neighbors.push_back(graph_[i]);
       costs.push_back((*it)(1));
     } else {
       std::cout << "ignore index: " << i << std::endl;
     }
  }
}

double Dijkstra::getHeuristic(NodePtr from, NodePtr to) {

  double dx = static_cast<double>(to->getX() -  from->getX());
  double dy = static_cast<double>(to->getY() -  from->getY());

  return std::sqrt( dx * dx + dy * dy);
}



bool Dijkstra::pathFinding(const Eigen::Vector2i& start, const Eigen::Vector2i& end, std::vector<Eigen::Vector2i>& path) {
    NodePtr start_ptr = graph_[start(1) * width_ + start(0)];
    NodePtr end_ptr = graph_[end(1) * width_ + end(0)];

    // 初始阶段， 只有起点的代价是0， 其余都是inf
    start_ptr->g_score_ = start_ptr->f_score_ = 0;

    bool find_path = false;

    std::cout <<"start: " << start_ptr->getIndex() << ", cost: " << static_cast<int>(start_ptr->cost_) << std::endl;
    std::cout << "end:  " << end_ptr->getIndex() << ", cost: " << static_cast<int>(end_ptr->cost_) << std::endl;


    while(!open_list_.empty())  open_list_.pop();

    start_ptr->type_ = LISTYPE::OPENLIST;
    open_list_.push(start_ptr);


    size_t cnt = 0;    // iteration cnt sum

    while(!open_list_.empty()) {


      // 1. 从open_list pop, f or g 最小的节点

      NodePtr current = open_list_.top();
      open_list_.pop();

      // 2. 标记该节点被扩展（已经找到起点到该点最优， 访问过的）
      close_list_.push_back(current);
      current->type_ = LISTYPE::CLOSELIST;   // expanded, while it also is visited;

      // 3. 判断该点是不是终点

      if(current->coord_i_ == end_ptr->coord_i_) {
        find_path = true;
        std::cout << "FOUND GOAL" << std::endl;
        break;
      }

      // 4. 找到最近邻， 加入openlist
      std::vector<NodePtr>  neighbors;
      std::vector<double> costs;
      getNeighbors(current, neighbors, costs);

      std::cout << "------" << cnt << ", neighbors: " <<  neighbors.size()<< std::endl;

      for(int i=0; i<neighbors.size();++i){
        std::cout << "current: " << current->getIndex() << ", neighbor: " << neighbors[i]->getIndex() << std::endl;


        // 对于所有不在close_list 中的邻居进行扩展
        if(neighbors[i]->type_ == LISTYPE::UNKNOW) {
          neighbors[i]->f_score_ = current->f_score_ + 1.0;
          neighbors[i]->type_ = LISTYPE::OPENLIST;
          neighbors[i]->parent_ = current;
          open_list_.push(neighbors[i]);

        } else if (neighbors[i]->type_ == LISTYPE::OPENLIST) {
          // 如果邻居已经在open list中， 则检查是否从当前点更近
          if(neighbors[i]->f_score_ > current->f_score_ + 1.0) {
            neighbors[i]->f_score_ = current->f_score_ + 1.0;
          }
        } else {
          std::cout << "? neighbor type: " << (int)(neighbors[i]->type_) << std::endl;
        }
      }

      ++cnt;

      visualize(20, current);
      // std::this_thread::sleep_for(std::chrono::milliseconds(400));
    }

    tracebackPath(start_ptr, end_ptr, path);

    return true;
}

bool Dijkstra::tracebackPath(NodePtr start, NodePtr end, std::vector<Eigen::Vector2i>& path) {
   NodePtr p = end;

   while(p->parent_) {
     path.emplace_back(p->getX(), p->getY());
     p = p->parent_;
   }

   std::cout << "FIND PATH, len " << path.size() << std::endl;
   cv::Mat copy = costmap_.clone();
   for(int i=0; i < path.size(); ++i){
      copy.at<uchar>(path[i](1), path[i](0)) = 30;
   }
   cv::imshow(name_, copy);
   cv::waitKey(10000);
   return true;
}

void Dijkstra::visualize(int ms, NodePtr p) {

  cv::Mat copy = costmap_.clone();

  for(auto it = close_list_.begin(); it != close_list_.end(); ++it) {
     copy.at<uchar>((*it)->getY(), (*it)->getX()) = 180;
  }

  if(p) {
    copy.at<uchar>(p->getY(), p->getX()) = 0;
  }
  cv::imshow(name_, copy);
  cv::waitKey(ms);
}


int main() {

  Dijkstra d;
  d.setCostmap();

  d.createGraph();

  Eigen::Vector2i start(10, 10);
  Eigen::Vector2i end(50, 40);

  std::vector<Eigen::Vector2i> path;
  d.pathFinding(start, end, path);
}