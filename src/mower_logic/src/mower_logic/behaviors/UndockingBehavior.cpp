// Created by Clemens Elflein on 2/21/22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//
#include "UndockingBehavior.h"

extern ros::ServiceClient dockingPointClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *mbfClientExePath;
extern nav_msgs::Odometry last_odom;

extern void stop();

extern bool setGPS(bool enabled);

UndockingBehavior UndockingBehavior::INSTANCE(&MowingBehavior::INSTANCE);
UndockingBehavior UndockingBehavior::RETRY_INSTANCE(&DockingBehavior::INSTANCE);

// 获取当前状态名
std::string UndockingBehavior::state_name() {
    return "UNDOCKING";
}

Behavior *UndockingBehavior::execute() {

    // get robot's current pose from odometry.
    // 解算偏航角yaw
    nav_msgs::Odometry odom = last_odom;
    tf2::Quaternion quat;
    tf2::fromMsg(odom.pose.pose.orientation, quat);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);


    mbf_msgs::ExePathGoal exePathGoal;

    nav_msgs::Path path;


    // 从当前位置，后退2m
    int undock_point_count = config.undock_distance * 10.0;  // undock_distance is 2
    ROS_INFO("undock_point_count : %d", undock_point_count);
    for (int i = 0; i < undock_point_count; i++) {
        geometry_msgs::PoseStamped docking_pose_stamped_front;
        docking_pose_stamped_front.pose = odom.pose.pose;
        docking_pose_stamped_front.header = odom.header;  // 仿真中原始坐标系为base_link
        docking_pose_stamped_front.header.frame_id = "map";
        // 反方向取，一共后退2m
        docking_pose_stamped_front.pose.position.x -= cos(yaw) * (i / 10.0);
        docking_pose_stamped_front.pose.position.y -= sin(yaw) * (i / 10.0);
        ROS_INFO("docking path : (x:%f, y:%f))",docking_pose_stamped_front.pose.position.x, docking_pose_stamped_front.pose.position.y);
        path.poses.push_back(docking_pose_stamped_front);
    }

    exePathGoal.path = path;
    exePathGoal.angle_tolerance = 1.0 * (M_PI / 180.0);
    exePathGoal.dist_tolerance = 0.1;
    exePathGoal.tolerance_from_action = true;
    exePathGoal.controller = "DockingFTCPlanner";

    auto result = mbfClientExePath->sendGoalAndWait(exePathGoal);

    bool success = result.state_ == actionlib::SimpleClientGoalState::SUCCEEDED;

    // stop the bot for now
    stop();

    if (!success) {
        ROS_ERROR_STREAM("Error during undock");
        return nullptr;
    }

    // 退出成功，等待gps信号
    ROS_INFO_STREAM("Undock success. Waiting for GPS.");
    // bool hasGps = waitForGPS();

    // if (!hasGps) {
    //     ROS_ERROR_STREAM("Could not get GPS.");
    //     return nullptr;
    // }

    // TODO return mow area
    return nextBehavior;
    // return &DockingBehavior::INSTANCE;

}

void UndockingBehavior::enter() {
  reset();

  // Get the docking pose in map
  // 获取对接点
  mower_map::GetDockingPointSrv get_docking_point_srv;
  dockingPointClient.call(get_docking_point_srv);
  docking_pose_stamped.pose = get_docking_point_srv.response.docking_pose;
  docking_pose_stamped.header.frame_id = "map";
  docking_pose_stamped.header.stamp = ros::Time::now();
  ROS_INFO("Get the docking pose in map : (x:%f, y:%f, z:%f)",
           docking_pose_stamped.pose.position.x,
           docking_pose_stamped.pose.position.y,
           docking_pose_stamped.pose.position.z);
}

void UndockingBehavior::exit() {

}

// 设置为不需要gps
void UndockingBehavior::reset() {
    gpsRequired = false;
}

// 返回是否需要gps
bool UndockingBehavior::needs_gps() {
    return gpsRequired;
}

bool UndockingBehavior::mower_enabled() {
    // No mower during docking
    return false;
}

// 等待gps
bool UndockingBehavior::waitForGPS() {
    gpsRequired = false;
    setGPS(true);
    ros::Rate odom_rate(1.0);
    // 一直等待gps
    while (ros::ok()) {
        if (last_odom.pose.covariance[0] < 0.05) {
            ROS_INFO("Got good gps, let's go");
            break;
        } else {
            ROS_INFO_STREAM("waiting for gps. current covariance: " << last_odom.pose.covariance[0]);
            odom_rate.sleep();
        }
    }
    // 如果节点挂掉，返回false
    if (!ros::ok()) {
        return false;
    }

    // wait additional time for odometry filters to converge
    ros::Rate r(ros::Duration(config.gps_wait_time, 0));
    r.sleep();

    gpsRequired = true;

    return true;
}

UndockingBehavior::UndockingBehavior(Behavior* next) {
    this->nextBehavior = next;
}

void UndockingBehavior::command_home() {

}

void UndockingBehavior::command_start() {

}

void UndockingBehavior::command_s1() {

}

void UndockingBehavior::command_s2() {

}

bool UndockingBehavior::redirect_joystick() {
    return false;
}
