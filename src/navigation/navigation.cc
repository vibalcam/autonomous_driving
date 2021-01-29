//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using Eigen::Rotation2Df;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

DEFINE_double(cp1_distance, 2.5, "Distance to travel for 1D TOC (cp1)");
DEFINE_double(cp3_curvature, 0.5, "Curvature for arc path (cp3)");

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

// Maximum speed in m/s
const int MAX_SPEED = 1;
// Maximum acceleration/deceleration in m/s^2
const int MAX_ACCELERATION = 3;
// Time between decisions (s)
const float DELTA_T = 0.05;

float dist_covered = 0;
bool isFirst = true;

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
    // Update the current navigation target
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
    // Update the current estimate of the robot's position in the map reference frame.
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
    if(!isFirst) {
      Rotation2Df r1(-robot_angle_);
      Vector2f loc_rel = r1 * (loc - robot_loc_);
      dist_covered += loc_rel.norm();
    }
    isFirst = false;
    
    // Update the robot's position in the odometry reference frame.
    robot_loc_ = loc;
    robot_angle_ = angle;
    // Update the current estimate of the robot's velocity
    robot_vel_ = vel;
    std::cout << "Velocity: " << vel << std::endl;
    robot_omega_ = ang_vel;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
    // This function will be called when the LiDAR sensor on the robot has a new scan.
    // Here cloud is an array of points observed by the laser sensor, in the sensor's reference frame
    // This information can be used to detect obstacles in the robot's path.
}

void Navigation::Run() {
  // Called every timestep. This will be the main entrypoint of the navigation code, and is responsible for publishing appropriate navitation commands.
  AckermannCurvatureDriveMsg msg;
  float velocity = robot_vel_.norm();
  float dist_left = FLAGS_cp1_distance - dist_covered;
  std::cout << "Distance_left: " << dist_left << std::endl;
  // Check if there is space to accelerate
  if(velocity < MAX_SPEED && dist_left > MAX_ACCELERATION * pow(DELTA_T, 2) / 2 + velocity * DELTA_T + pow(MAX_SPEED, 2) / MAX_ACCELERATION / 2) {
    msg.velocity = MAX_SPEED;
  } else if(dist_left < velocity * DELTA_T + pow(velocity, 2) / MAX_ACCELERATION / 2) {   // If there is no space to keep at the same speed, stop
    msg.velocity = 0;
  }
  
  drive_pub_.publish(msg);
}

}  // namespace navigation
