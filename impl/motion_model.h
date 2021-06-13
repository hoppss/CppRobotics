#ifndef __MOTIONMODEL_H_
#define __MOTIONMODEL_H_

#include <string>

enum class MotionModel
{
  UNKNOWN = 0,
  VON_NEUMANN = 1,
  MOORE = 2,
  DUBIN = 3,
  REEDS_SHEPP = 4,
};

// VON_NEUMANN: 4-neighbor
// MOORE: 8-neighbor

inline std::string toString(const MotionModel & n)
{
  switch (n) {
    case MotionModel::VON_NEUMANN:
      return "Von Neumann";
    case MotionModel::MOORE:
      return "Moore";
    case MotionModel::DUBIN:
      return "Dubin";
    case MotionModel::REEDS_SHEPP:
      return "Reeds-Shepp";
    default:
      return "Unknown";
  }
}

inline MotionModel fromString(const std::string & n)
{
  if (n == "VON_NEUMANN") {
    return MotionModel::VON_NEUMANN;
  } else if (n == "MOORE") {
    return MotionModel::MOORE;
  } else if (n == "DUBIN") {
    return MotionModel::DUBIN;
  } else if (n == "REEDS_SHEPP") {
    return MotionModel::REEDS_SHEPP;
  } else {
    return MotionModel::UNKNOWN;
  }
}


#endif
