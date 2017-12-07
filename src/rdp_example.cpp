#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <stdlib.h>
#include "douglas_peucker.hpp"

// color =1 -> red , color = 2 -> green
ros::Publisher long_pub;
ros::Publisher short_pub;
void displayCurve(std::vector<Eigen::Vector3d> curve, int color);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rdp_example");
  ros::NodeHandle node;
  long_pub= node.advertise<nav_msgs::Path>("rdp/long", 5);
  short_pub= node.advertise<nav_msgs::Path>("rdp/short", 5);
  ros::Duration(1.0).sleep();

  //////
  std::vector<int> v1;
  std::vector<int> v2;
  std::vector<int> v3;
  std::vector<int> v= { 1, 2, 3, 4 };
  v1.insert(v1.begin(), v.begin(), v.begin() + 2);
  v2.insert(v2.begin(), v.begin() + 2, v.end());
  v3.insert(v3.begin(), v.begin(), v.end());
  for(int i= 0; i < v1.size(); i++)
    std::cout << v1[i];
  std::cout << std::endl;
  for(int i= 0; i < v2.size(); i++)
    std::cout << v2[i];
  std::cout << std::endl;
  for(int i= 0; i < v3.size(); i++)
    std::cout << v3[i];
  //////

  // create a curve containing numerous points, then show it in rviz
  std::srand(int(ros::Time::now().toSec()));

  const int point_num= 30;
  float step= 10.0f / point_num;
  float turb= 1.0;
  std::vector<Eigen::Vector3d> curve;
  for(int i= 0; i <= point_num; i++)
  {
    Eigen::Vector3d p;
    p(0)= -5.0 + step * i + turb * std::rand() / float(RAND_MAX);
    p(1)= -5.0 + step * i + turb * std::rand() / float(RAND_MAX);
    p(2)= turb * std::rand() / float(RAND_MAX);

    curve.push_back(p);
  }

  displayCurve(curve, 1);

  // use RDP to simplify it, and show in rviz
  RDPCurveSimplifier simplifier;
  float epsilon= 0.7;

  double t1= ros::Time::now().toSec();
  simplifier.simplify(curve, epsilon);
  double t= ros::Time::now().toSec() - t1;
  std::cout << "time" << t << std::endl;

  displayCurve(curve, 2);

  return 0;
}

void displayCurve(std::vector<Eigen::Vector3d> curve, int color)
{
  nav_msgs::Path path;
  path.header.frame_id= "world";
  path.header.stamp= ros::Time::now();

  for(int i= 0; i < curve.size(); i++)
  {
    // publish these point
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id= "world";
    pose.pose.position.x= curve[i](0);
    pose.pose.position.y= curve[i](1);
    pose.pose.position.z= curve[i](2);
    pose.pose.orientation.w= 1;
    pose.pose.orientation.x= 0;
    pose.pose.orientation.y= 0;
    pose.pose.orientation.z= 0;

    path.poses.push_back(pose);
  }
  if(color == 1)
    long_pub.publish(path);

  else if(color == 2)
    short_pub.publish(path);
}