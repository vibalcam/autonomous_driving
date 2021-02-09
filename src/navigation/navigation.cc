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
#include <math.h>

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
const bool IS_SIMULATION = true;

// Maximum speed in m/s
const int MAX_SPEED = 1;
// Maximum acceleration/deceleration in m/s^2
const int MAX_ACCELERATION = 3;
// Time between decisions (s)
const float DELTA_T = 0.05;

// System latency in s
// const float LATENCY = 0.15;
const int FWD_PREDICT_PERIODS = 3;  // latency takes n instructions

// Location of the LIDAR with respect to base_link (BASE = LIDAR + v)
const Vector2f LOCATION_LIDAR(0.2,0);
// Obstacle margin in m
const float OBSTACLE_MARGIN = 0.1;

// Dimensions of car in m
// const float WIDTH = 0.281;
// const float HEIGHT = 0.206;
const float FRONT_BASE = 0.4;
// const float FRONT_BASE = 0.4295;
const float REER_BASE = -0.1055;
const float SIDE_ABS_BASE = 0.1405; // absolute value, both sides (real measure 0.133)

// Dimensions of car including margin in m
const float FRONT_MARGIN_BASE = FRONT_BASE + OBSTACLE_MARGIN;
const float REER_MARGIN_BASE = REER_BASE - OBSTACLE_MARGIN;
const float SIDE_ABS_MARGIN_BASE = SIDE_ABS_BASE + OBSTACLE_MARGIN;

// Point cloud from the LIDAR in the sensor's reference frame
vector<Vector2f> pointCloud;
float distCovered = 0;
bool isFirst = true;
float futureVelocities[FWD_PREDICT_PERIODS];

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
      distCovered += loc_rel.norm();
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
  pointCloud = cloud;
}

void visualizeCarDimensions(VisualizationMsg& viz_msg) {
  // Visualize car dimensions
  visualization::DrawLine(
    Vector2f(REER_BASE,SIDE_ABS_BASE), Vector2f(REER_BASE,-SIDE_ABS_BASE), 0xFF0000, viz_msg);
  visualization::DrawLine(
    Vector2f(FRONT_BASE,SIDE_ABS_BASE), Vector2f(FRONT_BASE,-SIDE_ABS_BASE), 0xFF0000, viz_msg);
  visualization::DrawLine(
    Vector2f(FRONT_BASE,SIDE_ABS_BASE), Vector2f(REER_BASE,SIDE_ABS_BASE), 0xFF0000, viz_msg);
  visualization::DrawLine(
    Vector2f(FRONT_BASE,-SIDE_ABS_BASE), Vector2f(REER_BASE,-SIDE_ABS_BASE), 0xFF0000, viz_msg);
  
  // Visualize car margins
  visualization::DrawLine(
    Vector2f(REER_MARGIN_BASE,SIDE_ABS_MARGIN_BASE), Vector2f(REER_MARGIN_BASE,-SIDE_ABS_MARGIN_BASE), 0xf57d05, viz_msg);
  visualization::DrawLine(
    Vector2f(FRONT_MARGIN_BASE,SIDE_ABS_MARGIN_BASE), Vector2f(FRONT_MARGIN_BASE,-SIDE_ABS_MARGIN_BASE), 0xf57d05, viz_msg);
  visualization::DrawLine(
    Vector2f(FRONT_MARGIN_BASE,SIDE_ABS_MARGIN_BASE), Vector2f(REER_MARGIN_BASE,SIDE_ABS_MARGIN_BASE), 0xf57d05, viz_msg);
  visualization::DrawLine(
    Vector2f(FRONT_MARGIN_BASE,-SIDE_ABS_MARGIN_BASE), Vector2f(REER_MARGIN_BASE,-SIDE_ABS_MARGIN_BASE), 0xf57d05, viz_msg);

    // Visualize current front
  visualization::DrawLine(Vector2f(0,0), Vector2f(2,0), 0x0400ff, viz_msg);
}

void Navigation::Run() {
  // Called every timestep. This will be the main entrypoint of the navigation code, and is responsible for publishing appropriate navitation commands.
  // Clear Visualizations
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  visualizeCarDimensions(local_viz_msg_);


  // // todo Calculations for r
  // float r1 = r - SIDE_ABS_MARGIN_BASE;
  // float r2 = sqrt(pow(r+SIDE_ABS_MARGIN_BASE,2) + pow(FRONT_MARGIN_BASE,2));
  // float distToCenter = (p-c).norm();
  // // check if dist to center is between r1 and r2
  // // use atan2(y,x), consider if point behind if we will hit it
  // float theta = atan2(p.x(),r-p.y());
  // float omega = atan2(FRONT_MARGIN_BASE,r-SIDE_ABS_MARGIN_BASE);
  // float f = r * (theta-omega);



  float velocity = robot_vel_.norm();
  // float dist_left = FLAGS_cp1_distance - distCovered;
  float dist_left = 10;

  // Check if there are any obstacles ahead
  float x, absY;
  for(Vector2f v : pointCloud) {
    v += LOCATION_LIDAR;  // Change to base_link
    // visualization::DrawCross(v, 0.05, 0xfc766f, local_viz_msg_);

    x = v.x() - FRONT_MARGIN_BASE;  // Check x value from obstacle
    absY = abs(v.y()) - SIDE_ABS_MARGIN_BASE; // Check y value from obstacle
    // std::cout << "Viewed: " << x << std::endl;
    if(absY <= 0 && dist_left > x) {  // if going to hit obstacle before reaching goal
      dist_left = x;
      // std::cout << "Change: " << dist_left << " for " << (x + OBSTACLE_MARGIN) << std::endl;
    }
  }
  // Visualize distance left
  std::cout << "Distance_left: " << (dist_left + OBSTACLE_MARGIN) << std::endl;
  // visualization::DrawCross(Vector2f(dist_left, 0), 0.2, 0x0dff00, local_viz_msg_);
  
  // Forward predict to take into account latency
  if(!IS_SIMULATION) {
    for(int k=0; k < FWD_PREDICT_PERIODS; k++) {
      // Distance covered in this period: (v2-v1)/2*t+v1*t=((v2-v1)/2+v1)*t
      dist_left -= ((futureVelocities[k] - velocity) / 2 + velocity) * DELTA_T;
      velocity = futureVelocities[k];

      if(k != FWD_PREDICT_PERIODS-1)
        futureVelocities[k] = futureVelocities[k+1]; 
    }
  }

  // Check if there is space to accelerate
  float speedUp = velocity + DELTA_T * MAX_ACCELERATION;
  if(speedUp > MAX_SPEED)
    speedUp = MAX_SPEED;
  if(dist_left > MAX_ACCELERATION * pow(DELTA_T, 2) / 2 + velocity * DELTA_T + pow(speedUp, 2) / MAX_ACCELERATION / 2) {
    velocity = speedUp;

    drive_msg_.velocity = velocity;
    drive_msg_.curvature = 0;
    drive_pub_.publish(drive_msg_);
  } else if(dist_left < velocity * DELTA_T + pow(velocity, 2) / MAX_ACCELERATION / 2) {   // If there is no space to keep at the same speed, stop
    velocity -= DELTA_T * MAX_ACCELERATION;
    if(velocity < 0) {
      velocity = 0;
    }

    drive_msg_.velocity = velocity;
    drive_msg_.curvature = 0;
    drive_pub_.publish(drive_msg_);
  }

  // Save commands
  futureVelocities[FWD_PREDICT_PERIODS-1] = velocity;
  // Publish visualizations
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
}

}  // namespace navigation
