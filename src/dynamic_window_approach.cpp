/*************************************************************************
        > File Name: main.cpp
        > Author: TAI Lei
        > Mail: ltai@ust.hk
        > Created Time: Thu Mar  7 19:39:14 2019
 ************************************************************************/

#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define PI 3.141592653

using State = std::array<float, 5>;  // x，y,theta, vx, vw; 五状态向量
using Traj = std::vector<State>;     // trajectory or path

using Point = std::array<float, 2>;

using Obstacle = std::vector<Point>;  // environment map abstract

using Window = std::array<float, 4>;   // two-dims control-state window
using Control = std::array<float, 2>;  // vx, vw

// config, or its kinematic model
class Config {
 public:
  float max_speed = 1.0;
  float min_speed = -0.5;
  float max_yawrate = 40.0 * PI / 180.0;

  float max_accel = 0.2;
  float robot_radius = 1.0;
  float max_dyawrate = 40.0 * PI / 180.0;

  float v_reso = 0.01;                    // window-x [ v_min: v_reso:  v_max]
  float yawrate_reso = 0.1 * PI / 180.0;  // window-w [w_min: w_reso: w_max ]

  float dt = 0.1;  // sim_time granularity

  float predict_time = 3.0;  // forward simulation and get traj to be evaluated

  float to_goal_cost_gain = 1.0;  // weighted factor for evalution funciton
  float speed_cost_gain = 1.0;
};

// forward simulation
State motion(State x, Control u, float dt) {
  x[2] += u[1] * dt;
  x[0] += u[0] * std::cos(x[2]) * dt;
  x[1] += u[0] * std::sin(x[2]) * dt;
  x[3] = u[0];
  x[4] = u[1];
  return x;
};

// windows for control state
Window calc_dynamic_window(State x, Config config) {
  return {
      {std::max((x[3] - config.max_accel * config.dt), config.min_speed),
       std::min((x[3] + config.max_accel * config.dt), config.max_speed),
       std::max((x[4] - config.max_dyawrate * config.dt), -config.max_yawrate),
       std::min((x[4] + config.max_dyawrate * config.dt), config.max_yawrate)}};
};

Traj calc_trajectory(State x, float v, float y, Config config) {
  Traj traj;
  traj.push_back(x);
  float time = 0.0;
  while (time <= config.predict_time) {
    x = motion(x, std::array<float, 2>{{v, y}}, config.dt);
    traj.push_back(x);
    time += config.dt;
  }
  return traj;
};

// 距离障碍物距离， 碰撞检测用, cost 越小越好
float calc_obstacle_cost(Traj traj, Obstacle ob, Config config) {
  // calc obstacle cost inf: collistion, 0:free
  int skip_n = 2;
  float minr = std::numeric_limits<float>::max();

  for (unsigned int ii = 0; ii < traj.size(); ii += skip_n) {
    for (unsigned int i = 0; i < ob.size(); i++) {
      float ox = ob[i][0];
      float oy = ob[i][1];
      float dx = traj[ii][0] - ox;
      float dy = traj[ii][1] - oy;

      float r = std::sqrt(dx * dx + dy * dy);
      if (r <= config.robot_radius) {
        return std::numeric_limits<float>::max();
      }

      if (minr >= r) {
        minr = r;
      }
    }
  }

  return 1.0 / minr;
};

// traj 前向模拟最后一点和目标点， 两个向量之间夹角， 投影cos
float calc_to_goal_cost(Traj traj, Point goal, Config config) {
  float goal_magnitude = std::sqrt(goal[0] * goal[0] + goal[1] * goal[1]);
  float traj_magnitude =
      std::sqrt(std::pow(traj.back()[0], 2) + std::pow(traj.back()[1], 2));
  float dot_product = (goal[0] * traj.back()[0]) + (goal[1] * traj.back()[1]);
  float error = dot_product / (goal_magnitude * traj_magnitude);
  float error_angle = std::acos(error);
  float cost = config.to_goal_cost_gain * error_angle;

  return cost;
};

// algrithm core: DWA
// 1. 根据当前速度u，和其窗口，对控制状态进行采样并生成轨迹
// 2. 对轨迹进行评价，选出最好的轨迹
// 3. 返回最好的轨迹，返回最好轨迹的控制
Traj calc_final_input(State x, Control& u, Window dw, Config config, Point goal,
                      std::vector<std::array<float, 2>> ob) {
  float min_cost = 10000.0;
  Control min_u = u;
  min_u[0] = 0.0;
  Traj best_traj;

  // evalucate all trajectory with sampled input in dynamic window
  for (float v = dw[0]; v <= dw[1]; v += config.v_reso) {
    for (float y = dw[2]; y <= dw[3]; y += config.yawrate_reso) {
      Traj traj = calc_trajectory(x, v, y, config);

      float to_goal_cost = calc_to_goal_cost(traj, goal, config);

      // 速度越大， cost 越低， forward prefer ?
      float speed_cost =
          config.speed_cost_gain * (config.max_speed - traj.back()[3]);

      float ob_cost = calc_obstacle_cost(traj, ob, config);

      float final_cost = to_goal_cost + speed_cost + ob_cost;

      if (min_cost >= final_cost) {
        min_cost = final_cost;
        min_u = Control{{v, y}};
        best_traj = traj;
      }
    }
  }
  u = min_u;
  return best_traj;
};

// 最外层算法入口， 返回轨迹为了可视化把
Traj dwa_control(State x, Control& u, Config config, Point goal, Obstacle ob) {
  // 1. 基于当前u，生成速度窗口
  // 2. 基于当前速度u，当前位置x，当前窗口，进行dwa速度分辨率采样-> 生成轨迹 ->
  // \ 评价轨迹 -> 选出最好轨迹及其控制u -> control
  Window dw = calc_dynamic_window(x, config);
  Traj traj = calc_final_input(x, u, dw, config, goal, ob);

  return traj;
};

// 可视化，放大，要不然看不清
cv::Point2i cv_offset(float x, float y, int image_width = 1000,
                      int image_height = 1000) {
  cv::Point2i output;
  output.x = int(x * 50);
  output.y = int(y * 50);
  return output;
};

int main() {
  // 起点终点， x始终表示当前位置点
  State x({{0.0, 0.0, PI / 8.0, 0.0, 0.0}});
  Point goal({{10.0, 10.0}});

  // 点障碍物
  Obstacle ob({{{0, 2}},
               {{4.0, 2.0}},
               {{5.0, 4.0}},
               {{5.0, 5.0}},
               {{5.0, 6.0}},
               {{5.0, 9.0}},
               {{8.0, 9.0}},
               {{7.0, 9.0}},
               {{12.0, 12.0}}});

  Control u({{0.0, 0.0}});
  Config config;
  Traj gloal_traj;
  gloal_traj.push_back(x);

  bool terminal = false;

  cv::namedWindow("dwa", cv::WINDOW_AUTOSIZE);

  int count = 0;

  for (int i = 0; i < 1000 && !terminal; i++) {
    Traj ltraj = dwa_control(x, u, config, goal, ob);
    x = motion(x, u, config.dt);
    gloal_traj.push_back(x);

    // visualization - backgroud 初始化为白色背景
    cv::Mat bg(1000, 1000, CV_8UC3, cv::Scalar(255, 255, 255));
    // visualization - 蓝色圆圈 为目标点
    cv::circle(bg, cv_offset(goal[0], goal[1], bg.cols, bg.rows), 30,
               cv::Scalar(255, 0, 0), 5);
    // visulizaiton - 障碍物画圆， -1 实心填充
    for (unsigned int j = 0; j < ob.size(); j++) {
      cv::circle(bg, cv_offset(ob[j][0], ob[j][1], bg.cols, bg.rows), 20,
                 cv::Scalar(0, 0, 0), -1);
    }
    // visualzation - 当前最优轨迹可视化，绿色
    for (unsigned int j = 0; j < ltraj.size(); j++) {
      cv::circle(bg, cv_offset(ltraj[j][0], ltraj[j][1], bg.cols, bg.rows), 7,
                 cv::Scalar(0, 255, 0), -1);
    }
    // visualization - 圈出当前位置点，红色
    cv::circle(bg, cv_offset(x[0], x[1], bg.cols, bg.rows), 30,
               cv::Scalar(0, 0, 255), 5);

    // visualization - 直线箭头， 柄： 当前位置x， 箭头：当前速度值*三角映射到xY
    cv::arrowedLine(bg, cv_offset(x[0], x[1], bg.cols, bg.rows),
                    cv_offset(x[0] + std::cos(x[2]), x[1] + std::sin(x[2]),
                              bg.cols, bg.rows),
                    cv::Scalar(0, 0, 0), 7);
    // log, output current speed
    std::cout << "vx: " << x[3] << ", v2: " << x[4] << std::endl;

    // isGoalReached judge
    if (std::sqrt(std::pow((x[0] - goal[0]), 2) +
                  std::pow((x[1] - goal[1]), 2)) <= config.robot_radius) {
      terminal = true;
      for (unsigned int j = 0; j < gloal_traj.size(); j++) {
        cv::circle(
            bg, cv_offset(gloal_traj[j][0], gloal_traj[j][1], bg.cols, bg.rows),
            7, cv::Scalar(0, 0, 255), -1);
      }
    }

    cv::imshow("dwa", bg);

    if (terminal)
      cv::waitKey(3000);
    else
      cv::waitKey(30);

    // std::string int_count = std::to_string(count);
    // cv::imwrite("./pngs/"+std::string(5-int_count.length(),
    // '0').append(int_count)+".png", bg);

    count++;
  }
}

/**
 * 1. vx 增长到最大后，一直保持在max 附近
 * 2. vw 是有效的
 * */
