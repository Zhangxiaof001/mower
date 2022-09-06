#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include "local_planner/LocalPlannerSrv.h"

#include "apf.h"
#include "vfh.h"
#include "local_planner_utils.h"

using namespace std;


    class LocalPlanner
    {
    public:
        LocalPlanner(ros::NodeHandle &nh);
        ros::NodeHandle local_planner_nh;

    private:
        // 参数
        int uav_id;
        int algorithm_mode;
        int map_input_source;
        double max_planning_vel;
        double fly_height;
        double safe_distance;
        bool sim_mode;
        bool map_groundtruth;
        string local_pcl_topic_name;
        double goal_distance;

        // 订阅无人机状态、目标点、传感器数据（生成地图）
        ros::Subscriber goal_sub;
        ros::Subscriber uav_state_sub;
        ros::Subscriber uav_control_state_sub;
        ros::Subscriber local_point_cloud_sub;

        // 发布控制指令
        ros::Publisher uav_cmd_pub;
        ros::Publisher rviz_vel_pub;
        ros::Publisher reach_goal_pub;
        ros::Timer mainloop_timer;
        ros::Timer control_timer;

        // 局部避障算法 算子
        local_planner_alg::Ptr local_alg_ptr;

        // prometheus_msgs::UAVState uav_state;      // 无人机状态
        // prometheus_msgs::UAVControlState uav_control_state;
        nav_msgs::Odometry uav_odom;
        // prometheus_msgs::UAVCommand uav_command; 
        geometry_msgs::Twist cmd_vel;
        ros::ServiceServer local_planner_srv;

        double distance_to_goal;
        // 规划器状态
        bool odom_ready;
        bool drone_ready;
        bool sensor_ready;
        bool goal_ready;
        bool path_ready;
        bool is_safety;
        bool path_ok;
        std_msgs::Bool reach_goal;
        std::vector<geometry_msgs::Pose> global_path_;
        int index_ = 0;
        bool rotating_to_goal_ = true;

        // 规划初始状态及终端状态
        Eigen::Vector3d uav_pos;  // 无人机位置
        Eigen::Vector3d uav_vel;  // 无人机速度
        Eigen::Quaterniond uav_quat; // 无人机四元数
        double uav_yaw;
        double goal_yaw;
        // 规划终端状态
        Eigen::Vector3d goal_pos, goal_vel;

        int planner_state;
        Eigen::Vector3d desired_vel;
        float desired_yaw;

        // 五种状态机
        enum EXEC_STATE
        {
            WAIT_GOAL,
            PLANNING,
            TRACKING,
            LANDING,
        };
        EXEC_STATE exec_state;

        sensor_msgs::PointCloud2ConstPtr local_map_ptr_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr;
        pcl::PointCloud<pcl::PointXYZ> latest_local_pcl_;

        double Normalize(double angle1, double angle2);
        bool ReceiveGoal(local_planner::LocalPlannerSrvRequest& req, local_planner::LocalPlannerSrvResponse& res);
        void goal_cb(const geometry_msgs::PoseStampedConstPtr &msg);
        // void uav_control_state_cb(const prometheus_msgs::UAVControlState::ConstPtr &msg);
        // void uav_state_cb(const prometheus_msgs::UAVStateConstPtr &msg);
        void odom_cb(const nav_msgs::Odometry::ConstPtr &msg);
        void pcl_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
        void laserscan_cb(const sensor_msgs::LaserScanConstPtr &msg);
        void mainloop_cb(const ros::TimerEvent &e);
        void control_cb(const ros::TimerEvent &e);
        void debug_info();
    };


#endif
