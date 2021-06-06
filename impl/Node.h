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


  Node(Eigen::Vector2i coord_i, unsigned char cost) {
    // index_ = index;
    cost_ = cost;
    coord_i_ = coord_i;

    parent_ = nullptr;

    type_ = LISTYPE::UNKNOW;   // initial is known, if CLOSELIST indicate visited

    f_score_ = g_score_ = std::numeric_limits<float>::max();
  }

  // int index_;
  Eigen::Vector2i  coord_i_;  // 0 is x, 1 is y index
  unsigned char cost_;      // cost in grid cell from costmap or mat

  //Eigen::Vector2d  coord_;  // coord in global frame

  NodePtr  parent_;         // parent_ which indicate where came frome


  LISTYPE type_;


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