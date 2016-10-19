#include <iostream>
#include <stdlib.h>
#include <random>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <boost/thread/thread.hpp>
#include <turtlesim/Spawn.h>
#include <turtlesim/Pose.h>  // x, y, theta
#include <turtlesim/TeleportAbsolute.h>
#include <std_srvs/Empty.h>
#include <math.h>

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
      command.angular.z = 0.075;
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
  void poseCallback(const turtlesim::Pose& msg) { theta = msg.theta; }
  void pose2Callback(const turtlesim::Pose& msg) {
    currentPose.theta = msg.theta;
    currentPose.x = msg.x;
    currentPose.y = msg.y;
  }

  FollowerDriver(ros::NodeHandle& nh) {
    n = nh;
    velPub = n.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 1);
    spinPose =
        n.subscribe("/turtle1/pose", 10, &FollowerDriver::poseCallback, this);
    myPose =
        n.subscribe("/turtle2/pose", 10, &FollowerDriver::pose2Callback, this);
  }

  bool follow() {
    std::cout << "Will now try to walk past Spinner.\n";
    ros::Duration(0.5).sleep();
    float vel, angle, control, height, width, velAdj, angAdj;
    height = 0.0;
    width = 0.0;
    velAdj = 1.0;
    angAdj = 1.0;

    ros::spinOnce();

    while (n.ok()) {
      // Implementing Forcefield around the spinner, eggshell size
      // Begin turning away if it is too close
      width = -4.5;
      velAdj = 1.0;
      angAdj = 1.0;
      height = sin(theta) * 0.5 + 0.75;

      // If passed the turtle, resume normal travel
      if (currentPose.x - 5.0 > 0.0) {
        // std::cout << "Passed"
        //           << "\n";
        width = 0.0;
        velAdj = 1.0;
        angAdj = 1.0;
        height = 0.0;
      }

      vel = 0.05 * velAdj *
            std::sqrt(std::pow(currentPose.x - (9.5 + width), 2) +
                      std::pow(currentPose.y - (5.5 + height), 2));
      if (vel <= 0.4 && fabs(currentPose.x - 9.5) < 0.25) {
        std::cout << "I made it!";
        return true;
      } else if (vel <= 0.04)
        vel = 0.4;
      angle =
          atan2((5.5 + height - currentPose.y), (9.5 + width - currentPose.x));
      control = 0.3 * angAdj * (angle - currentPose.theta);
      // std::cout << height << " " << theta << "\n";
      command.linear.x = vel;
      command.angular.z = control;

      velPub.publish(command);
      ros::spinOnce();
      ros::Duration(0.5).sleep();
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
  ros::ServiceClient add_turtle = nh.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);
  ros::ServiceClient teleport_turtle =
      nh.serviceClient<turtlesim::TeleportAbsolute>(
          "turtle1/teleport_absolute");
  turtlesim::TeleportAbsolute srv2;
  srv2.request.x = 5.0;
  srv2.request.y = 5.5;
  srv2.request.theta = 0.0;
  teleport_turtle.call(srv2);
  ros::ServiceClient teleport_turtle2 =
      nh.serviceClient<turtlesim::TeleportAbsolute>(
          "turtle2/teleport_absolute");
  turtlesim::TeleportAbsolute srv3;
  srv3.request.x = 0.5;
  srv3.request.y = 5.5;
  srv3.request.theta = 0.0;
  teleport_turtle2.call(srv3);

  ros::service::waitForService("clear");
  ros::ServiceClient clear = nh.serviceClient<std_srvs::Empty>("clear");
  std_srvs::Empty empt;
  clear.call(empt);

  boost::thread thread_b(&SpinnerDriver::lead, Spinner);
  follower.follow();
}
