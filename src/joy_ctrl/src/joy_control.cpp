#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

using namespace std;

class JoyControl {
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
public:
    //线速度 单位: m/s; 角速度 单位: rad/s
    double linear_speed_limit=1.0, angular_speed_limit=2.0;
    string robot_simulation;
    string control_topic;
    void callback(const sensor_msgs::Joy::ConstPtr &Joy);
    
    // 构造函数
    JoyControl() {
      // turtlebot3
      control_topic = "/cmd_vel";
      linear_speed_limit = 1.0;
      angular_speed_limit = 1.5;
      
      // cout << "robot_simulation: " << robot_simulation << "  turtle_topic: " << turtle_topic << endl;
      pub = n.advertise<geometry_msgs::Twist>(control_topic, 1);
      sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &JoyControl::callback, this);
    }
};

void JoyControl::callback(const sensor_msgs::Joy::ConstPtr &Joy) {
//    cout << "axes("<<Joy->axes.size()<<"): [";
//    for (int i = 0; i < Joy->axes.size(); i++) {
//        cout << Joy->axes.at(i) << " ,";
//    }
//    cout << "]" << endl << "buttons("<<Joy->buttons.size()<<"): [";
//    for (int i = 0; i < Joy->buttons.size(); i++) {
//        cout << Joy->buttons.at(i) << " ,";
//    }
//    cout << "]" << endl;
    geometry_msgs::Twist twist;
    twist.linear.x = Joy->axes[1] * linear_speed_limit;
    twist.angular.z = Joy->axes[3] * angular_speed_limit;
    ROS_INFO("linear: %.3f angular: %.3f", twist.linear.x, twist.angular.z);
    pub.publish(twist);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "JoyControl");
    JoyControl joy_control;
    ros::spin();
    return 0;
}


