#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <stdlib.h>
#include "display.h"
#include "grad_traj_optimizer.h"

using namespace std;

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

  srand(ros::Time::now().toSec());
  ros::Duration(0.5).sleep();

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

  // add obstacles set in launch file
  sdf_tools::COLLISION_CELL obstacle_cell(1.0);
  vector<Eigen::Vector2d> obstacles;
  int obs_num = 0;

  ros::param::get("/traj_opti_node1/obstacle_num", obs_num);
  for(int i = 0; i < obs_num; ++i)
  {
    Eigen::Vector2d obs;
    ros::param::get("/traj_opti_node1/obstacle_x_" + to_string(i + 1), obs(0));
    ros::param::get("/traj_opti_node1/obstacle_y_" + to_string(i + 1), obs(1));
    obstacles.push_back(obs);
  }

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

  //-----------------------------add waypoints-----------------------------------------
  ros::param::get("/traj_opti_node1/waypoint_num", point_num);
  vector<Eigen::Vector3d> way_points;
  for(int i = 0; i < point_num; ++i)
  {
    Eigen::Vector3d wp;
    ros::param::get("/traj_opti_node1/waypoint_x_" + to_string(i + 1), wp(0));
    ros::param::get("/traj_opti_node1/waypoint_y_" + to_string(i + 1), wp(1));
    ros::param::get("/traj_opti_node1/waypoint_z_" + to_string(i + 1), wp(2));
    way_points.push_back(wp);
  }

  visualizeSetPoints(way_points);

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
