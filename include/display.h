#ifndef _DISPLAY_H
#define _DISPLAY_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Eigen>
#include <vector>

using std::vector;

Eigen::VectorXd my_time;
int point_num;

ros::Publisher setpoint_pub;
ros::Publisher traj_pub;
ros::Publisher traj_point_pub;

void getPositionFromCoeff(Eigen::Vector3d &pos, Eigen::MatrixXd coeff, int index, double time);

// visualize initial waypoint
void visualizeSetPoints(vector<Eigen::Vector3d> points)
{
  // send them to rviz
  for(int i = 0; i < points.size(); ++i)
  {
    visualization_msgs::Marker p;
    p.header.frame_id = "world";
    p.header.stamp = ros::Time::now();
    p.id = i;

    p.type = visualization_msgs::Marker::SPHERE;
    p.action = visualization_msgs::Marker::ADD;

    p.pose.position.x = points[i][0];
    p.pose.position.y = points[i][1];
    p.pose.position.z = points[i][2];
    p.pose.orientation.w = 1;
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;

    p.scale.x = p.scale.y = p.scale.z = 0.2;

    p.color.a = p.color.r = 1.0;
    p.color.g = p.color.b = 0.0;

    p.lifetime = ros::Duration(20.0);

    setpoint_pub.publish(p);
    ros::Duration(0.001).sleep();

    // ROS_INFO_STREAM("publish set point");
  }
}

// use coefficient of polynomials to draw the trajectory
void displayTrajectory(Eigen::MatrixXd coeff, bool animate)
{
  nav_msgs::Path path;
  path.header.frame_id = "world";
  path.header.stamp = ros::Time::now();

  // publish the whole trajectory
  double _t;
  for(int s = 0; s < (point_num - 1); s++)
  {
    // show optimized set point
    visualization_msgs::Marker p;
    p.header.frame_id = "world";
    p.header.stamp = ros::Time::now();
    p.id = s + point_num;

    p.type = visualization_msgs::Marker::SPHERE;
    p.action = visualization_msgs::Marker::ADD;

    Eigen::Vector3d pos(3);
    getPositionFromCoeff(pos, coeff, s, 0);

    p.pose.position.x = pos(0);
    p.pose.position.y = pos(1);
    p.pose.position.z = pos(2);
    p.pose.orientation.w = 1;
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;

    p.scale.x = p.scale.y = p.scale.z = 0.2;

    p.color.a = p.color.r = p.color.g = 1.0;
    p.color.b = 0.0;

    p.lifetime = ros::Duration(7.0);

    setpoint_pub.publish(p);

    // show path
    _t = my_time(s);
    for(float t = 0; t < _t; t += 0.2)
    {
      Eigen::Vector3d pos(3);
      getPositionFromCoeff(pos, coeff, s, t);

      // publish these point
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "world";

      pose.pose.position.x = pos(0);
      pose.pose.position.y = pos(1);
      pose.pose.position.z = pos(2);

      pose.pose.orientation.w = 1;
      pose.pose.orientation.x = 0;
      pose.pose.orientation.y = 0;
      pose.pose.orientation.z = 0;

      path.poses.push_back(pose);
    }
  }
  traj_pub.publish(path);

  if(!animate)
    return;
  // then publish point on trajectory one by one
  for(int s = 0; s < (point_num - 1); s++)
  {
    _t = my_time(s);
    for(float t = 0; t < _t; t += 0.05)
    {
      visualization_msgs::Marker p;
      p.header.frame_id = "world";
      p.header.stamp = ros::Time::now();
      p.id = 1;

      p.type = visualization_msgs::Marker::SPHERE;
      p.action = visualization_msgs::Marker::ADD;

      Eigen::Vector3d pos(3);
      getPositionFromCoeff(pos, coeff, s, t);

      p.pose.position.x = pos(0);
      p.pose.position.y = pos(1);
      p.pose.position.z = pos(2);
      p.pose.orientation.w = 1;
      p.pose.orientation.x = 0;
      p.pose.orientation.y = 0;
      p.pose.orientation.z = 0;

      p.scale.x = p.scale.y = p.scale.z = 0.3;

      p.color.a = p.color.r = p.color.g = 1.0;
      p.color.b = 0.0;

      p.lifetime = ros::Duration(0.05);
      traj_point_pub.publish(p);
      ros::Duration(0.01).sleep();
    }
  }
}

void getPositionFromCoeff(Eigen::Vector3d &pos, Eigen::MatrixXd coeff, int index, double time)
{
  int s = index;
  double t = time;
  float x = coeff(s, 0) + coeff(s, 1) * t + coeff(s, 2) * pow(t, 2) + coeff(s, 3) * pow(t, 3) +
            coeff(s, 4) * pow(t, 4) + coeff(s, 5) * pow(t, 5);
  float y = coeff(s, 6) + coeff(s, 7) * t + coeff(s, 8) * pow(t, 2) + coeff(s, 9) * pow(t, 3) +
            coeff(s, 10) * pow(t, 4) + coeff(s, 11) * pow(t, 5);
  float z = coeff(s, 12) + coeff(s, 13) * t + coeff(s, 14) * pow(t, 2) + coeff(s, 15) * pow(t, 3) +
            coeff(s, 16) * pow(t, 4) + coeff(s, 17) * pow(t, 5);

  pos(0) = x;
  pos(1) = y;
  pos(2) = z;
}

#endif