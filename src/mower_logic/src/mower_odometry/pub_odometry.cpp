#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "nav_msgs/Odometry.h"

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "pub_odometry");
  ros::NodeHandle node;

  tf::TransformListener listener;
  ros::Rate rate(100);
  ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("/mower/odom", 100);

  listener.waitForTransform("/odom", "/base_link", ros::Time(0),
                            ros::Duration(10.0));

  while (ros::ok()) {
    tf::StampedTransform transform;
    try {
      listener.lookupTransform("/odom", "/base_link", ros::Time(0),
                               transform);
      double x = transform.getOrigin().x();
      double y = transform.getOrigin().y();
      nav_msgs::Odometry odom;
      odom.header.stamp= transform.stamp_;
      odom.pose.pose.position.x = transform.getOrigin().x();
      odom.pose.pose.position.y = transform.getOrigin().y();
      odom.pose.pose.position.z = 0;
      tf::quaternionTFToMsg(transform.getRotation(), odom.pose.pose.orientation);
      odom_pub.publish(odom);
      // cout << "Current position: (" << x << "," << y << ")" << endl;
    } catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
    }
    rate.sleep();
  }

  return 0;
}