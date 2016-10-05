#include <iostream>
#include <stdlib.h>
#include <random>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <boost/thread/thread.hpp>
#include <turtlesim/Spawn.h>
#include <turtlesim/Pose.h> // x, y, theta
#include <turtlesim/TeleportAbsolute.h>
#include <std_srvs/Empty.h>

// Creating a class for a Spinner driver
class SpinnerDriver {
private:
  // Nothing outside of this class should even need to see this
  ros::NodeHandle n;
  ros::Publisher velPub;
  geometry_msgs::Twist command;

public:
  // Something others may be able to see

  // Creating a constructor that takes a ROS node handle and generates a
  // publisher
  SpinnerDriver(ros::NodeHandle& nh) {
    n = nh;
    velPub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  }

  void lead() {
    std::cout << "Will now try to lead the way for my follower!\n";
    ros::Duration(0.5).sleep();

    while (n.ok()) {
      command.angular.z = 0.1;
      velPub.publish(command);
      ros::Duration(1.1).sleep();
    }
  }
};

// Creating a that will attempt to follow the Spinner
class FollowerDriver {
private:
  // Nothing outside of this class should even need to see this
  ros::NodeHandle n;
  ros::Publisher velPub;
  ros::Subscriber spinPose;
  ros::Subscriber myPose;
  turtlesim::Pose currentPose;
  geometry_msgs::Twist command;
  float theta;

public:
  // Creating a constructor that takes a ROS node handle and generates a
  // publisher
  void poseCallback(const turtlesim::Pose &msg)
  {
    theta = msg.theta;
  }
  void pose2Callback(const turtlesim::Pose &msg)
  {
    currentPose.theta = msg.theta;
    currentPose.x = msg.x;
    currentPose.y = msg.y;
  }

  FollowerDriver(ros::NodeHandle& nh) {
    n = nh;
    velPub = n.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 1);
    spinPose = n.subscribe("/turtle1/pose", 10, &FollowerDriver::poseCallback, this);
    myPose = n.subscribe("/turtle2/pose", 10, &FollowerDriver::pose2Callback, this);
  }

  bool follow() {
    std::cout << "Will now try to walk past Spinner.\n";
    ros::Duration(0.5).sleep();
    float vel;

    while (n.ok()) {
      vel = 0.025*std::sqrt((currentPose.x-9.5)^2+(currentPose.y-5.5)^2);
      float angle = std::atan();
      velPub.publish(command);
      ros::Duration(1.1).sleep();
    }
    return true;
  }
};

int main(int argc, char** argv) {
  // init the ROS node
  ros::init(argc, argv, "SpinnerFollower");
  ros::NodeHandle nh;

  // Creates and runs the drunken turtle
  FollowerDriver follower(nh);
  SpinnerDriver Spinner(nh);

  ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle =
  nh.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);
  ros::ServiceClient teleport_turtle =
  nh.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
  turtlesim::TeleportAbsolute srv2;
  srv2.request.x = 5.0;
  srv2.request.y = 5.5;  
  srv2.request.theta = 0;
  teleport_turtle.call(srv2);
  ros::ServiceClient teleport_turtle2 =
  nh.serviceClient<turtlesim::TeleportAbsolute>("turtle2/teleport_absolute");
  turtlesim::TeleportAbsolute srv3;
  srv3.request.x = 0.5;
  srv3.request.y = 5.5;  
  srv3.request.theta = 0;
  teleport_turtle2.call(srv3);

  ros::service::waitForService("clear");
  ros::ServiceClient clear =  nh.serviceClient<std_srvs::Empty>("clear");
  std_srvs::Empty empt;
  clear.call(empt);

  boost::thread thread_b(&SpinnerDriver::lead, Spinner);
  follower.follow();
}
