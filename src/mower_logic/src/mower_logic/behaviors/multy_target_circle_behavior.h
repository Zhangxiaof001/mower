
#ifndef MULTY_TARGET_CIRCLE_BEHAVIOR_H
#define MULTY_TARGET_CIRCLE_BEHAVIOR_H

#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <mower_map/GetDockingPointSrv.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Transform.h>

#include "Behavior.h"
#include "DockingBehavior.h"
#include "IdleBehavior.h"
#include "geometry_msgs/PoseStamped.h"
#include "mower_msgs/Status.h"
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
class  MultyTargetCircle : public Behavior {
 public:
  static MultyTargetCircle INSTANCE;
  static MultyTargetCircle RETRY_INSTANCE;

  // 构造函数
//   MultyTargetCircle();

 private:
  int status = 0;
  bool thread_exit_ =false;
  ros::Subscriber  multy_target_sub_;
  ros::NodeHandle*ph_;
  Behavior* nextBehavior;  // 下一个状态，在构造函数中赋值
  // 是否需要gps
  bool gpsRequired;

  bool waitForGPS();

 public:
  std::string state_name() override;

  Behavior* execute() override;

  void enter() override;

  void exit() override;

  void reset() override;

  bool needs_gps() override;

  bool mower_enabled() override;

  void command_home() override;

  void command_start() override;

  void command_s1() override;

  void command_s2() override;

  bool redirect_joystick() override;
};
#endif