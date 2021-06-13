#ifndef _NODE_H_
#define _NODE_H_

#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <memory>
#include <queue>
#include <limits>
#include <utility>
#include <functional>

// #define inf 1e9


enum class LISTYPE
{
  UNKNOW=0,
  OPENLIST,
  CLOSELIST,
};



class Node;              // declaration
typedef Node* NodePtr;

class Node {
public:


  Node(int i, Eigen::Vector2i coord,unsigned char cost) {
    index_ = i;
    coord_i_ = coord;
    cost_ = cost;

    parent_ = nullptr;

    type_ = LISTYPE::UNKNOW;   // initial is known, if CLOSELIST indicate visited

    f_score_ = g_score_ = std::numeric_limits<float>::max();
  }


  int getX() {
    return coord_i_(0);
  }

  int getY() {
    return coord_i_(1);
  }

  int getIndex() {
    return index_;
  }

  int index_;
  unsigned char cost_;      // cost in grid cell from costmap or mat

  Eigen::Vector2i  coord_i_;  // 0 is x, 1 is y index
  // Eigen::Vector2d  coord_;  // coord in global frame

  NodePtr  parent_;         // parent_ which indicate where came frome


  LISTYPE type_;            // mark Node current location type


  double g_score_;          // cost from start to current
  double h_score_;          // heurist cost from current and goal
  double f_score_;         // all cost f = g + h

};



// 小顶堆
class NodeComparator
{
public:
  bool operator()(const NodePtr & a, const NodePtr & b) const
  {
    return a->f_score_ > b->f_score_;
  }
};


typedef std::vector<NodePtr> Graph;

typedef std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> NodeQueue;



#endif