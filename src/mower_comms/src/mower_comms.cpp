#include "ros/ros.h"
#include "mower_msgs/HighLevelControlSrv.h"

// 高等级用户控制服务
ros::ServiceClient highLevelClient;

int main(int argc, char **argv) {
    ros::init(argc, argv, "mower_comms");

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    // 高等级用户控制服务
    highLevelClient = n.serviceClient<mower_msgs::HighLevelControlSrv>(
            "mower_service/high_level_control");

    ROS_INFO("Success launch mower comms node");
    ros::spin();

    return 0;
}
