#include "multy_target_circle_behavior.h"
#include "chrono"
#include <thread>
#include "IdleBehavior.h"
extern actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> *mbfClient;

extern ros::NodeHandle *n;
geometry_msgs::PoseStamped target_points_;
MultyTargetCircle MultyTargetCircle::INSTANCE;

bool kRecivedPointFlag = false;
void PoseStampedCallBack(geometry_msgs::PoseStampedConstPtr msg) {
  kRecivedPointFlag = true;
  target_points_ = *msg;
}
void MultyTargetCircle::enter() {
  ph_ = n;
  multy_target_sub_ = ph_->subscribe<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 10, PoseStampedCallBack);

  ROS_INFO("multy target enter");
}
Behavior *MultyTargetCircle::execute() {
  while (!thread_exit_) {
    if (status == 0) {
      if (!kRecivedPointFlag) {
        std::this_thread::sleep_for(std::chrono::seconds(100));
        continue;
      } else {
        kRecivedPointFlag = false;
        status = 1;
      }
    } else if (status == 1) {
      mbf_msgs::MoveBaseGoal moveBaseGoal;
      moveBaseGoal.target_pose = target_points_;
      moveBaseGoal.controller = "FTCPlanner";
      auto result = mbfClient->sendGoalAndWait(moveBaseGoal);
      if (result.state_ = result.SUCCEEDED) {
        status = 0;
        //     actionlib_msgs::GoalStatusArray goal_staus_array_pub =
        //         ph_->advertise<actionlib_msgs::GoalStatusArray>("move_base/status");
        //    goal_staus_array_pub.
      }
    }
  }
  return &IdleBehavior::INSTANCE;
}

std::string MultyTargetCircle::state_name() { return "  MultyTargetCircle"; }

void MultyTargetCircle::exit() {}

// 初始化尝试次数
void MultyTargetCircle::reset() {}

// 获取是否需要gps
bool MultyTargetCircle::needs_gps() {
    // we only need GPS if we're in approach mode
}
// 获取是否支持割草
bool MultyTargetCircle::mower_enabled() {
  // No mower during docking
  return false;
}
void MultyTargetCircle::command_home() {}

void MultyTargetCircle::command_start() {}

void MultyTargetCircle::command_s1() { thread_exit_ = true; }

void MultyTargetCircle::command_s2() {}

bool MultyTargetCircle::redirect_joystick() { return false; }




