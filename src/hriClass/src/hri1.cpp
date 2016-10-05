#include <iostream>
#include <stdlib.h>
#include <random>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

// Creating a class for drunk driver so that I can create other types later
class DrunkDriver {
 private:
  // Nothing outside of this class should even need to see this
  ros::NodeHandle n;
  ros::Publisher velPub;
  geometry_msgs::Twist command;

 public:
  // Something others may be able to see
  geometry_msgs::Point goal;
  geometry_msgs::Point start;
  double bac = 0.09;
  double newBeerProb = 0.05;

  // Creating a constructor that takes a ROS node handle and generates a publisher
  DrunkDriver(ros::NodeHandle& nh) {
    n = nh;
    velPub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
    goal.x = 10.0;
    goal.y = 10.0;
    start.x = start.y = start.z = 0;
  }

  // Actually controls the drunken behavior
  // Will try to walk forward until they stumble due to drunkeness
  // When they stumble, they will turn a random direction and try to keep walking straight
  // There is also a low probabablity, newBeerProb, that the turtle's BAC will rise
  // I tried to set goal points, etc. but it would need a lot more work, listening to 
  // messages published from the turtlebot, as well as a lot of math haha
  bool drunken() {
    std::cout << "Will now try to drive to a goal 10 meters forward.\n";
    std::cout << "Notice: I am currently Drunk with BAC= " << bac << "\n";
    ros::Duration(0.5).sleep();

    // This is the new standard random number generator in c++11
    std::mt19937 gen(time(NULL));
    std::uniform_real_distribution<double> prob(0, 2);

    // Step size is the maximum distance the robot will travel per command
    double step = (goal.x + goal.x + goal.z) / 80;
    double countAng = 0.0;
    
    while (n.ok()) {
      // Try to move towards goal from current position.
      if (prob(gen) / 2 <= exp(bac)/10) {
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
      if (prob(gen) / 2 <= newBeerProb) {
        bac += 0.015;
        std::cout << "Woohoo! Found another Beer! BAC " << bac  << std::endl;
      }

      // if (goal.x - currentPos.x <= 0.5)
      //   if (goal.y - currentPos.y <= 0.5)
      //     if (goal.z - currentPos.z <= 0.5) {
      //       command.linear.x = 0;
      //       command.linear.y = 0;
      //       command.linear.z = 0;
      //       command.angular.z = 0;
      //       velPub.publish(command);
      //       std::cout << "\'Slosh\', I'm here!" << std::endl;
      //       break;
      //     }

      // Actually publishes the command to the turtle
      velPub.publish(command);
      ros::Duration(1.1).sleep();
    }
    return true;
  }
};

int main(int argc, char** argv) {
  // init the ROS node
  ros::init(argc, argv, "DrunkDriver");
  ros::NodeHandle nh;

  // Creates and runs the drunken turtle
  DrunkDriver driver(nh);
  driver.drunken();
}
