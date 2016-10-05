#include <iostream>
#include <stdlib.h>
#include <random>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <boost/thread/thread.hpp>

// Creating a class for a leader driver
class LeaderDriver {
 private:
  // Nothing outside of this class should even need to see this
  ros::NodeHandle n;
  ros::Publisher velPub;
  geometry_msgs::Twist command;

 public:
  // Something others may be able to see
  geometry_msgs::Point goal;
  geometry_msgs::Point start;

  // Creating a constructor that takes a ROS node handle and generates a
  // publisher
  LeaderDriver(ros::NodeHandle& nh) {
    n = nh;
    velPub =
        n.advertise<geometry_msgs::Twist>("/turtlesim1/turtle1/cmd_vel", 1);
    goal.x = 10.0;
    goal.y = 10.0;
  }

  void lead() {
    std::cout << "Will now try to lead the way for my follower!\n";
    ros::Duration(0.5).sleep();

    // This is the new standard random number generator in c++11
    std::mt19937 gen(time(NULL));
    std::uniform_real_distribution<double> prob(0, 2);

    // Step size is the maximum distance the robot will travel per command
    double step = (goal.x + goal.x + goal.z) / 80;
    double countAng = 0.0;

    while (n.ok()) {
      // Try to move towards goal from current position.
      if (prob(gen) / 2 <= exp(2) / 10) {
        command.linear.x = step * (prob(gen) - 1);
        // command.linear.y = step * (prob(gen) - 1);
        // command.linear.z = step * (prob(gen) - 1);
        command.angular.z = 3.14159265359 * (prob(gen) - 1);
      } else {
        command.linear.x = step;
        // command.linear.y = step * (goal.y - currentPos.y);
        // command.linear.z = step * (goal.z - currentPos.z);
        command.angular.z = 0;
      }
      if (prob(gen) / 2 <= 2) {
        std::cout << "Woohoo! Found another Beer! BAC " << 2 << std::endl;
      }

      // Actually publishes the command to the turtle
      velPub.publish(command);
      ros::Duration(1.1).sleep();
    }
  }
};

// Creating a that will attempt to follow the leader
class FollowerDriver {
 private:
  // Nothing outside of this class should even need to see this
  ros::NodeHandle n;
  ros::Publisher velPub;
  geometry_msgs::Twist command;

 public:
  // Creating a constructor that takes a ROS node handle and generates a
  // publisher
  FollowerDriver(ros::NodeHandle& nh) {
    n = nh;
    velPub =
        n.advertise<geometry_msgs::Twist>("/turtlesim2/turtle1/cmd_vel", 1);
  }

  bool follow() {
    std::cout << "Will now try to follow the leader.\n";
    ros::Duration(0.5).sleep();

    // This is the new standard random number generator in c++11
    std::mt19937 gen(time(NULL));
    std::uniform_real_distribution<double> prob(0, 2);

    // Step size is the maximum distance the robot will travel per command
    double step = 80;
    double countAng = 0.0;

    while (n.ok()) {
      // Try to move towards goal from current position.
      if (prob(gen) / 2 <= exp(2) / 10) {
        command.linear.x = step * (prob(gen) - 1);
        // command.linear.y = step * (prob(gen) - 1);
        // command.linear.z = step * (prob(gen) - 1);
        command.angular.z = 3.14159265359 * (prob(gen) - 1);
      } else {
        command.linear.x = step;
        // command.linear.y = step * (goal.y - currentPos.y);
        // command.linear.z = step * (goal.z - currentPos.z);
        command.angular.z = 0;
      }
      if (prob(gen) / 2 <= 2) {
        std::cout << "Woohoo! Found another Beer! BAC " << 2 << std::endl;
      }

      // Actually publishes the command to the turtle
      velPub.publish(command);
      ros::Duration(1.1).sleep();
    }
    return true;
  }
};

int main(int argc, char** argv) {
  // init the ROS node
  ros::init(argc, argv, "LeaderFollower");
  ros::NodeHandle nh;

  // Creates and runs the drunken turtle
  FollowerDriver follower(nh);
  LeaderDriver leader(nh);

  boost::thread thread_b(&LeaderDriver::lead, leader);
  follower.follow();
}
