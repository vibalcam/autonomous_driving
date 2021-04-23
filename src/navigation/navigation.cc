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
#include "vector_map/vector_map.h"

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

// STATIC VALUES

// Maximum speed in m/s
const float MAX_SPEED = 1;
// Maximum acceleration/deceleration in m/s^2
const float MAX_ACCELERATION = 3;
// Time between decisions (s)
const float DELTA_T = 0.05;
// System latency in s
// const float LATENCY = 0.15;
const int FWD_PREDICT_PERIODS = 4;  // latency takes n instructions
// const int FWD_PREDICT_PERIODS = 2;  // latency takes n instructions
// Location of the LIDAR with respect to base_link (BASE = LIDAR + v)
const Vector2f LOCATION_LIDAR(0.2,0);
// Max curvature of turning
const float MAX_ABS_CURVATURE = 1;
// Obstacle margin in m
const float OBSTACLE_MARGIN = 0.1;

// CONFIGURABLE VALUES

// Min curvature of turning (otherwise considered as driving straight)
const float MIN_CURVATURE = 0.01;
// Delta curvature of turning being explored
const float DELTA_CURVATURE = 0.01;
// const float DELTA_CURVATURE = 0.5;
// Max curvature change to optimize clearance
const float CLEARANCE_CURVATURE = 0.4;
// const float CLEARANCE_CURVATURE = 0.2;
// Maximum clearance to take into account
const float MAX_CLEARANCE = 0.5;
const float MIN_CLEARANCE = 0.3;

// Weight to calculate best path option
// const float W_CLEARANCE = 15;
// Weight to calculate best path option
const float W_DIST_GOAL = 0.1;

// Dimensions of car in m
// const float WIDTH = 0.281;
// const float HEIGHT = 0.206;
// const float FRONT_BASE = 0.4;
const float FRONT_BASE = 0.4295;
const float REER_BASE = -0.1055;
const float SIDE_ABS_BASE = 0.1405; // absolute value, both sides (real measure 0.133)

// Dimensions of car including margin in m
const float FRONT_MARGIN_BASE = FRONT_BASE + OBSTACLE_MARGIN;
const float REER_MARGIN_BASE = REER_BASE - OBSTACLE_MARGIN;
const float SIDE_ABS_MARGIN_BASE = SIDE_ABS_BASE + OBSTACLE_MARGIN;

// Point cloud from the LIDAR in the sensor's reference frame
vector<Vector2f> pointCloud;
// Map
vector_map::VectorMap map;
float distCovered = 0;
bool isFirst = true;
float futureVelocities[FWD_PREDICT_PERIODS];
float futureCurvatures[FWD_PREDICT_PERIODS];
Vector2f futurePosition(0,0);

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

  map = vector_map::VectorMap(map_file);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
    // Update the current navigation target
    nav_goal_loc_ = loc;
    nav_goal_angle_ = angle;
    nav_complete_ = false;  //todo update tu true when arrived
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
    // Update the current estimate of the robot's position in the map reference frame.
    robot_loc_ = loc;
    robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
    if(!isFirst) {
      Rotation2Df r1(-odom_angle_);
      Vector2f loc_rel = r1 * (loc - odom_loc_);
      distCovered += loc_rel.norm();
    }
    isFirst = false;
    
    // Update the robot's position in the odometry reference frame.
    odom_loc_ = loc;
    odom_angle_ = angle;
    // Update the current estimate of the robot's velocity
    robot_vel_ = vel;
    std::cout << "Velocity: " << vel << std::endl;
    robot_omega_ = ang_vel;
}

float atan2Positive(float y, float x) {
  float a = atan2(y,x);
  return (a>=0) ? a : a + 2*M_PI;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  // This function will be called when the LiDAR sensor on the robot has a new scan.
  // Here cloud is an array of points observed by the laser sensor, in the sensor's reference frame
  // This information can be used to detect obstacles in the robot's path.
  pointCloud = cloud;
}

void calculateGlobalPlanner() {
    /**
     * todo
     * preguntar si hay que correrlo en cada timestep
     */
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
  visualization::DrawLine(Vector2f(0,0), Vector2f(1,0), 0x0400ff, viz_msg);
}

/* 
  Returns the location after moving a certain distance with a certain curvature
  The distance must be positive
*/
Vector2f predictLocation(float& distance, float& curvature) {
  if(abs(curvature) > MIN_CURVATURE) { // turning
    float r = 1.0f / curvature;
    float ang = distance / r;
    // If r and ang are negative, sin(-ang)=-sin(ang) and cos(-ang)=cos(ang); also works
    return Vector2f(sin(ang) * r, r - cos(ang) * r); 
  } else {  // driving straight
    return Vector2f(distance, 0);
  }
}

/* 
  Returns the maximum free path given a radius and the goal.
  When turning, theta is returned
      otherwise, distance is returned
*/
float calculateMaxOptimumFreePath(float& curvature, Vector2f& goal) {
  if(abs(curvature) > MIN_CURVATURE) {
    float r = 1.0f / curvature;
    Vector2f radius(0,r);
    Vector2f v = goal - radius;
    radius = -1 * radius / radius.norm();
    v = v / v.norm();
    // since radius.x() is always 0
    float theta = atan2Positive(-radius.y()*v.x(), radius.y()*v.y());
    // std::cout << "Theta Max: " << (theta / M_PI*180) << std::endl;
    return curvature > 0 ? theta : 2 * M_PI - theta;
  } else {
    return goal.x();
  }
}

float calculateMaxClearance(float velocity) {
  return MAX_CLEARANCE - (MAX_SPEED-velocity) / MAX_SPEED * (MAX_CLEARANCE - MIN_CLEARANCE);
}

// Calculates the free path length given the curvature
float calculateFreePath(float curvature, Vector2f& goal) {
  float freePath, dist_left;

  if(abs(curvature) > MIN_CURVATURE) { // if turning
    // Radius of turning
    float r = 1.0f / abs(curvature);
    float signR = signbit(curvature) * -2 + 1; // from (0,1) (1 if negative) to (-1,1), 1 if turning left
    float r1 = r - SIDE_ABS_MARGIN_BASE;
    float r2 = sqrt(pow(r+SIDE_ABS_MARGIN_BASE,2) + pow(FRONT_MARGIN_BASE,2));
    float omega = atan2Positive(FRONT_MARGIN_BASE,r-SIDE_ABS_MARGIN_BASE);
    float maxTheta = calculateMaxOptimumFreePath(curvature, goal);  // makes sure we are not driving away from target
    dist_left = r * maxTheta;
    // std::cout << "dist: " << dist_left << std::endl;

    // check if points are in the turning arc
    float distToCenter,theta;
    // todo add points behind base link for free path
    for(Vector2f v : pointCloud) {
      v += LOCATION_LIDAR - futurePosition;  // Change to base_link
      // visualization::DrawCross(v, 0.04, 0xfc766f, local_viz_msg_);
      v.y() *= signR;  // Change y value deppending on direction of turning
      distToCenter = (v-Vector2f(0,r)).norm();

      // Calculate free path length
      if(distToCenter >= r1 && distToCenter <= r2) {  // check if dist to center is between r1 and r2
        // std::cout << "Viewed: " << v << std::endl;
        theta = atan2Positive(v.x(),r-v.y());
        if(theta > maxTheta)
          theta = maxTheta;
        freePath = r * (theta-omega);
        if(dist_left > freePath)
          dist_left = freePath;
      }
    }
  } else {  // if driving straight
    float x, absYtoCar;
    float maxFreePath = calculateMaxOptimumFreePath(curvature, goal);  // makes sure we are not driving away from target
    dist_left = maxFreePath;
    for(Vector2f v : pointCloud) {
      v += LOCATION_LIDAR - futurePosition;  // Change to base_link
      // visualization::DrawCross(v, 0.04, 0xfc766f, local_viz_msg_);

      // Calculate free path length
      x = v.x() - FRONT_MARGIN_BASE;  // Check x value from obstacle
      absYtoCar = abs(v.y()) - SIDE_ABS_MARGIN_BASE; // Check y value from obstacle
      // std::cout << "Viewed: " << x << std::endl;
      if(absYtoCar <= 0) {  // if going to hit obstacle before reaching goal
        freePath = x < maxFreePath ? x : maxFreePath;
        if(dist_left > freePath)
          dist_left = freePath;
      }
    }
  }
  return dist_left;
}

// Calculates the clearance given a curvature and free path length
float calculateClearance(float& curvature, float& freePath, float velocity) {
  float clearance = calculateMaxClearance(velocity);

  if(abs(curvature) > MIN_CURVATURE) { // if turning
    // Radius of turning
    float r = 1.0f / abs(curvature);
    float signR = signbit(curvature) * -2 + 1; // from (0,1) (1 if negative) to (-1,1), 1 if turning left
    float r1 = r - SIDE_ABS_MARGIN_BASE;
    float r2 = sqrt(pow(r+SIDE_ABS_MARGIN_BASE,2) + pow(FRONT_MARGIN_BASE,2));
    // max theta is theta from free path + dimensions of car (omega)
    float maxTheta = freePath/r + atan2Positive(FRONT_MARGIN_BASE,r-SIDE_ABS_MARGIN_BASE);  // makes sure we are not driving away from target

    // check if points are in the turning arc
    float distToCenter,theta, distClearance;
    // todo add points behind base link for clearance
    for(Vector2f v : pointCloud) {
      v += LOCATION_LIDAR - futurePosition;  // Change to base_link
      v.y() *= signR;  // Change y value deppending on direction of turning
      distToCenter = (v-Vector2f(0,r)).norm();
      theta = atan2Positive(v.x(),r-v.y());

      // Calculate clearance
      if(theta > 0 && theta < maxTheta) {
        if((distClearance = distToCenter - r2) > 0 && distClearance < clearance)
          clearance = distClearance;
        else if((distClearance = r1 - distToCenter) > 0 && distClearance < clearance)
          clearance = distClearance;
      }
    }
  } else {  // if driving straight
    float absYtoCar;
    for(Vector2f v : pointCloud) {
      v += LOCATION_LIDAR - futurePosition;  // Change to base_link
      absYtoCar = abs(v.y()) - SIDE_ABS_MARGIN_BASE; // Check y value from obstacle

      // Calculate clearance
      if(v.x() > 0 && v.x() < freePath + FRONT_MARGIN_BASE && clearance > absYtoCar) {
        clearance = absYtoCar;
      }
    }
  }
  return clearance;
}

// Calculates the distance to the goal after driving a certain distance with a certain curvature
float calculateDistToGoal(float& distance, float& curvature, Vector2f& goal) {
  return (goal - predictLocation(distance, curvature)).norm();
}

void calculatePlan(float& dist_left, float& curvature, Vector2f& goal, float& velocity) {
  dist_left = 0;
  curvature = 0;
  float bestScore = -FLT_MAX;
  float freePath, clearance, distToGoal, score;
  float cur_curvature = MAX_ABS_CURVATURE;
  while(cur_curvature >= -MAX_ABS_CURVATURE) {
    freePath = calculateFreePath(cur_curvature, goal);
    // clearance = calculateClearance(cur_curvature, freePath, velocity);
    distToGoal = calculateDistToGoal(freePath, cur_curvature, goal);

    // calculate score for this option
    // score = freePath + W_CLEARANCE * clearance - W_DIST_GOAL * distToGoal;
    // use narrow openings ignoring first dist to goal and then look for close by solutions
    score = freePath - W_DIST_GOAL * distToGoal;
    if(score > bestScore || (score == bestScore && abs(cur_curvature) < abs(curvature))) {
      curvature = cur_curvature;
      dist_left = freePath;
      bestScore = score;
    }
    cur_curvature -= DELTA_CURVATURE;
  }

  // Optimize for clearance
  float goalNorm = goal.norm();
  float min_curv = curvature - CLEARANCE_CURVATURE;
  if(min_curv < -MAX_ABS_CURVATURE)
    min_curv = -MAX_ABS_CURVATURE;
  cur_curvature = curvature + CLEARANCE_CURVATURE;
  if(cur_curvature > MAX_ABS_CURVATURE)
    cur_curvature = MAX_ABS_CURVATURE;
  bestScore = 0;
  while(cur_curvature >= min_curv) {
    freePath = calculateFreePath(cur_curvature, goal);
    clearance = calculateClearance(cur_curvature, goalNorm, velocity);
    // score = freePath + W_CLEARANCE / goal.norm() * clearance;
    if(clearance > bestScore || (clearance == bestScore && abs(cur_curvature) < abs(curvature))) {
      bestScore = clearance;
      curvature = cur_curvature;
      dist_left = freePath;
    }
    cur_curvature -= DELTA_CURVATURE;
  }
}

Vector2f doLatencyCompensation(float& velocity, float angVel) {
  // // Forward predict to take into account latency
  // for(int k=0; k < FWD_PREDICT_PERIODS; k++) {
  //   // Distance covered in this period: (v2-v1)/2*t+v1*t=((v2-v1)/2+v1)*t
  //   dist_left -= ((futureVelocities[k] - velocity) / 2 + velocity) * DELTA_T;
  //   velocity = futureVelocities[k];

  //   if(k != FWD_PREDICT_PERIODS-1)
  //     futureVelocities[k] = futureVelocities[k+1]; 
  // }

  Vector2f forwardPredict(0,0);
  float dist;
  // Forward predict to take into account latency
  for(int k=0; k < FWD_PREDICT_PERIODS; k++) {
    // Distance covered in this period: (v2-v1)/2*t+v1*t=((v2-v1)/2+v1)*t
    dist = ((futureVelocities[k] - velocity) / 2 + velocity) * DELTA_T;
    forwardPredict += predictLocation(dist, futureCurvatures[k]);

    velocity = futureVelocities[k];
    if(k != FWD_PREDICT_PERIODS-1) {
      futureVelocities[k] = futureVelocities[k+1]; 
      futureCurvatures[k] = futureCurvatures[k+1];
    }
  }

  // Vector2f forwardPredict(0,0);
  // Vector2f v(0,0);
  // float ang = angVel * DELTA_T;
  // v = Vector2f(velocity * cos(ang), velocity * sin(ang));

  // Vector2f futVel(0,0);
  // float angVelFut = 0;
  // float r;
  // // Forward predict to take into account latency
  // for(int k=0; k < FWD_PREDICT_PERIODS; k++) {
  //   if(abs(futureCurvatures[k]) > MIN_CURVATURE) { // turning
  //     r = 1.0f / futureCurvatures[k];
  //     angVelFut = futureVelocities[k] / r;
  //     ang = angVelFut * DELTA_T;
      
  //     futVel = Vector2f(futureVelocities[k] * cos(ang), futureVelocities[k] * sin(ang));
  //   } else {  // driving straight
  //     futVel = Vector2f(futureVelocities[k],0);
  //   }

  //   // Distance covered in this period: (v2-v1)/2*t+v1*t=((v2-v1)/2+v1)*t
  //   forwardPredict += ((futVel - v) / 2 + v) * DELTA_T;

  //   v = futVel;
  //   if(k != FWD_PREDICT_PERIODS-1) {
  //     futureVelocities[k] = futureVelocities[k+1]; 
  //     futureCurvatures[k] = futureCurvatures[k+1];
  //   }
  // }
  return forwardPredict;
}

// Drives the car using 1D TOC and updates velocity 
void drive(float& dist_left, float& velocity, float& curvature) {
  // Check if there is space to accelerate
  float speedUp = velocity + DELTA_T * MAX_ACCELERATION;
  if(speedUp > MAX_SPEED)
    speedUp = MAX_SPEED;
  if(dist_left > MAX_ACCELERATION * pow(DELTA_T, 2) / 2 + velocity * DELTA_T + pow(speedUp, 2) / MAX_ACCELERATION / 2) {
    velocity = speedUp;
  } else if(dist_left < velocity * DELTA_T + pow(velocity, 2) / MAX_ACCELERATION / 2) {   // If there is no space to keep at the same speed, stop
    velocity -= DELTA_T * MAX_ACCELERATION;
    if(velocity < 0) {
      velocity = 0;
    }
  }
  drive_msg_.velocity = velocity;
  drive_msg_.curvature = abs(curvature) > MIN_CURVATURE ? curvature : 0;
  drive_pub_.publish(drive_msg_);
}

void Navigation::Run() {
  // Called every timestep. This will be the main entrypoint of the navigation code, and is responsible for publishing appropriate navitation commands.
  // Clear Visualizations
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);
  visualizeCarDimensions(local_viz_msg_);

  float velocity = robot_vel_.norm();
  // float curvature = FLAGS_cp3_curvature;
  // float dist_left = FLAGS_cp1_distance - distCovered;
  calculateGlobalPlanner();
  // todo set goal to appropiate goal
  Vector2f goal(3,0);
  // nav_goal_angle_ = 0;
  visualization::DrawCross(goal, 0.1, 0xff0324, local_viz_msg_);

  // Do latency compensation
  if(!IS_SIMULATION) {
    futurePosition = doLatencyCompensation(velocity, robot_omega_);
  }
  // goal -= futurePosition;

  // Plan path
  float dist_left, curvature;
  calculatePlan(dist_left, curvature, goal, velocity);
  visualization::DrawPathOption(curvature,dist_left,SIDE_ABS_MARGIN_BASE,local_viz_msg_);

  std::cout << "Distance_left: " << (dist_left + OBSTACLE_MARGIN) << std::endl;

  // Drive distance left
  drive(dist_left, velocity, curvature);

  // Save commands
  futureVelocities[FWD_PREDICT_PERIODS-1] = velocity;
  futureCurvatures[FWD_PREDICT_PERIODS-1] = curvature;
  // Publish visualizations
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
}

}  // namespace navigation