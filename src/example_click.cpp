#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <stdlib.h>
#include "display.h"
#include "grad_traj_optimizer.h"

using namespace std;

ros::Subscriber waypoint_sub;
void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

bool waypoint_enough = false;
vector<Eigen::Vector3d> way_points;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "random");
  ros::NodeHandle node;

  // -------------------visulize endpoints and trajectory---------------------
  setpoint_pub = node.advertise<visualization_msgs::Marker>("trajopt/setpoint", 10);
  traj_point_pub = node.advertise<visualization_msgs::Marker>("trajopt/traj_point", 10);
  traj_pub = node.advertise<nav_msgs::Path>("trajopt/init_traj", 5);
  ros::Publisher visualization_pub =
      node.advertise<visualization_msgs::Marker>("sdf_tools_tutorial_visualization", 1, true);

  waypoint_sub = node.subscribe("/move_base_simple/goal", 5, waypointCallback);  // 2D Nav Goal
  // waypoint_sub= node.subscribe("/goal", 5, waypointCallback);                   // 3D Nav Goal

  srand(ros::Time::now().toSec());
  ros::Duration(0.5).sleep();

  ros::param::get("/traj_opti_node1/point_num", point_num);

  //---------------------create a map using sdf_tools-----------------------------

  // sdf collision map parameter
  const double resolution = 0.1;
  const double x_size = 20.0;
  const double z_size = 5.0;
  double y_size = 20.0;
  Eigen::Translation3d origin_translation(-10.0, -10.0, 0.0);
  Eigen::Quaterniond origin_rotation(1.0, 0.0, 0.0, 0.0);
  const Eigen::Isometry3d origin_transform = origin_translation * origin_rotation;
  const std ::string frame = "world";

  // create map
  sdf_tools ::COLLISION_CELL cell;
  cell.occupancy = 0.0;
  cell.component = 0;
  const sdf_tools ::COLLISION_CELL oob_cell = cell;
  sdf_tools ::CollisionMapGrid collision_map(origin_transform, frame, resolution, x_size, y_size,
                                             z_size, oob_cell);

  // add some obstacle randomly
  sdf_tools::COLLISION_CELL obstacle_cell(1.0);

  int obs_num = 50;
  vector<Eigen::Vector3d> obstacles;
  cout << "----------------------Add obstacles to map!---------------" << endl;
  int fail_num = 0;
  for(int i = 0; i < obs_num;)
  {
    // randomly create a obstacle point
    Eigen::Vector3d pt;
    pt(0) = -3.0 + 6.0 * rand() / double(RAND_MAX);
    pt(1) = -3.0 + 6.0 * rand() / double(RAND_MAX);
    pt(2) = 2.0;

    // ensure that each obstacle is far enough from others
    if(i == 0)
    {
      obstacles.push_back(pt);
      ++i;
    }
    else
    {
      double min_dist = 1000.0;
      double dist_thresh = 1.85;
      for(int j = 0; j < obstacles.size(); ++j)
      {
        double dist = (obstacles[j] - pt).norm();
        if(dist < min_dist)
          min_dist = dist;
      }

      if(min_dist > dist_thresh)
      {
        obstacles.push_back(pt);
        ++i;
        fail_num = 0;
      }
      else
      {
        ++fail_num;
      }
    }
    if(fail_num > 10000)
    {
      break;
    }
  }

  cout << "----------------------Obstacles generated!----------------------" << endl;

  // add the generated obstacles into collision map
  const int th = 2;
  for(float z = 0; z < 3.5; z += resolution)
    for(int i = 0; i < obstacles.size(); ++i)
    {
      for(int m = -th; m <= th; m++)
        for(int n = -th; n <= th; n++)
        {
          collision_map.Set(obstacles[i](0) + m * resolution, obstacles[i](1) + n * resolution, z,
                            obstacle_cell);
        }
    }

  // visualize the collision map
  std_msgs::ColorRGBA collision_color;
  collision_color.r = 0.0;
  collision_color.g = 0.0;
  collision_color.b = 1.0;
  collision_color.a = 0.8;

  std_msgs::ColorRGBA free_color, unknown_color;
  unknown_color.a = free_color.a = 0.0;

  visualization_msgs::Marker collision_map_marker =
      collision_map.ExportForDisplay(collision_color, free_color, unknown_color);
  collision_map_marker.ns = "collision_map";
  collision_map_marker.id = 1;

  visualization_pub.publish(collision_map_marker);

  // Build the signed distance field
  float oob_value = INFINITY;
  std::pair<sdf_tools::SignedDistanceField, std::pair<double, double>> sdf_with_extrema =
      collision_map.ExtractSignedDistanceField(oob_value);

  sdf_tools::SignedDistanceField sdf = sdf_with_extrema.first;
  cout << "----------------------Signed distance field build!----------------------" << endl;

  //-----------------------------Wait for user to click waypoint--------------------
  cout << "----------------------Please click some way_points----------------------- " << endl;
  while(ros::ok() && !waypoint_enough)
  {
    ros::spinOnce();
    ros::Duration(0.5).sleep();
  }

  // ----------------------------main optimization procedure--------------------------
  GradTrajOptimizer grad_traj_opt(node, way_points);
  grad_traj_opt.setSignedDistanceField(&sdf, resolution);

  Eigen::MatrixXd coeff;
  grad_traj_opt.getCoefficient(coeff);
  grad_traj_opt.getSegmentTime(my_time);
  displayTrajectory(coeff, false);

  // first step optimization
  grad_traj_opt.optimizeTrajectory(OPT_FIRST_STEP);
  grad_traj_opt.getCoefficient(coeff);
  displayTrajectory(coeff, false);

  //  second step optimization
  grad_traj_opt.optimizeTrajectory(OPT_SECOND_STEP);
  grad_traj_opt.getCoefficient(coeff);
  displayTrajectory(coeff, true);

  return 0;
}

void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  if(msg->pose.position.z < -0.01 || msg->pose.position.z > 4.0)
  {
    ROS_WARN("z should be between 0.0 and 4.0!!");
    return;
  }

  Eigen::Vector3d pt;
  pt(0) = msg->pose.position.x;
  pt(1) = msg->pose.position.y;
  // pt(2)= msg->pose.position.z;
  pt(2) = 2.0;

  way_points.push_back(pt);

  visualizeSetPoints(way_points);
  cout << "------------------New waypoint added!------------------" << endl;

  if(way_points.size() == point_num)
  {
    cout << "------------------Waypoint enough!------------------" << endl;
    waypoint_enough = true;
  }
  else
  {
    cout << "------------------" << (point_num - way_points.size()) << " more way_points required"
         << endl;
  }
}
