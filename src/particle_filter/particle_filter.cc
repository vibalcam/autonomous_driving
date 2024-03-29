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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::Line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using Eigen::Rotation2Df;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");

namespace particle_filter {
// Number of particles
const int N_PARTICLES = 30;

// Values for the estimation of gaussian error
const float K1 = 0.2; 
const float K2 = 0.2; 
const float K3 = 0.6; 
const float K4 = 0.2; 
// Initialization randomness values
const float INIT_VAR_LOC = 0.25;
const float INIT_VAR_ANG = M_PI * 10.0 / 180.0;

// TODO make generic for navigation and particle filter
// Location of the LIDAR with respect to base_link (BASE = LIDAR + v)
const Vector2f LOCATION_LIDAR(0.2,0);

// Values for the probability density function
const float VAR_LIKELIHOOD = pow(0.1,2);
const float GAMMA_LIKELIHOOD = 0.2;
const float D_SHORT = 0.35;
const float P_D_SHORT = pow(D_SHORT, 2) / VAR_LIKELIHOOD;
const float D_LONG = 0.55;
const float P_D_LONG = pow(D_LONG, 2) / VAR_LIKELIHOOD;
const int N_SKIP_RAYS = 12;
// const float D_UPDATE = 0;
// float dist_left_update = 0;
// bool update_available = false;
const int N_UPDATES_RESAMPLE = 10;
int n_updates = 0;

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

float getValidAng(float ang) {
  if(abs(ang) > M_PI) {
    int div = (int) (ang / (2*M_PI) + 1);
    ang -= 2 * M_PI * div;
  }
  return ang;
  // return math_util::AngleMod(ang);
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  // scan.resize(num_ranges);
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  // for (size_t i = 0; i < scan.size(); ++i) {
  //   scan[i] = Vector2f(0, 0);
  // }

  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:
  // for (size_t i = 0; i < map_.lines.size(); ++i) {
    /*
    const Line2f map_line = map_.lines[i];
    // The Line2f class has helper functions that will be useful.
    // You can create a new line segment instance as follows, for :
    Line2f my_line(1, 2, 3, 4); // Line segment from (1,2) to (3.4).
    // Access the end points using `.p0` and `.p1` members:
    printf("P0: %f, %f P1: %f,%f\n", 
           my_line.p0.x(),
           my_line.p0.y(),
           my_line.p1.x(),
           my_line.p1.y());

    // Check for intersections:
    bool intersects = map_line.Intersects(my_line);
    // You can also simultaneously check for intersection, and return the point
    // of intersection:
    Vector2f intersection_point; // Return variable
    intersects = map_line.Intersection(my_line, &intersection_point);
    if (intersects) {
      printf("Intersects at %f,%f\n", 
             intersection_point.x(),
             intersection_point.y());
    } else {
      printf("No intersection\n");
    }
    */
  // }

  scan.resize(num_ranges);
  Vector2f min_intersection_point;
  Vector2f LOCATION_LIDAR_MAP = loc + LOCATION_LIDAR;
  float cos_v,sin_v, sq_norm, min_sq_norm;
  float theta = getValidAng(angle + angle_min);
  float delta_theta = (angle_max-angle_min) / num_ranges;

  for (size_t i = 0; i < scan.size(); ++i) {
    min_intersection_point = LOCATION_LIDAR_MAP + Vector2f(range_max+1,0);
    min_sq_norm = pow(range_max+1,2);
    cos_v = cos(theta);
    sin_v = sin(theta);
    Line2f my_line(LOCATION_LIDAR_MAP.x() + cos_v * range_min,LOCATION_LIDAR_MAP.y() + sin_v * range_min,
                    LOCATION_LIDAR_MAP.x() + cos_v * range_max, LOCATION_LIDAR_MAP.y() + sin_v * range_max);

    // Iterate through all the lines in the map
    for (size_t j = 0; j < map_.lines.size(); ++j) {
      Vector2f intersection_point;
      if (map_.lines[j].Intersection(my_line, &intersection_point)) {
        sq_norm = (intersection_point - LOCATION_LIDAR_MAP).squaredNorm();
        if(sq_norm < pow(range_max,2) && sq_norm > pow(range_min,2) &&
            sq_norm < min_sq_norm) {
          min_intersection_point = intersection_point;
          min_sq_norm = sq_norm;
        }
      } 
    }

    scan[i] = min_intersection_point;
    theta = getValidAng(theta + delta_theta);
  }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.

  // vector<Vector2f> scan_ptr;
  // GetPredictedPointCloud(p_ptr->loc, p_ptr->angle, ranges.size(), range_min, 
  //     range_max, angle_min, angle_max,&scan_ptr);
  vector<float> scan_ptr;
  // p_ptr->loc + LOCATION_LIDAR
  map_.GetPredictedScan(p_ptr->loc, range_min, range_max, 
      getValidAng(angle_min + p_ptr->angle), getValidAng(angle_max + p_ptr->angle), 
      ranges.size(), &scan_ptr);
  
  double w = 0;
  double predicted, observed;
  // double predicted;
  // Vector2f location_lidar_map = p_ptr->loc + LOCATION_LIDAR;
  for(size_t k = 0; k < ranges.size(); k+=N_SKIP_RAYS) {
    observed = ranges[k];
    // predicted = (scan_ptr[k] - location_lidar_map).norm();
    predicted = scan_ptr[k];
    if(observed < range_min || observed > range_max) {
      // w += 10000;   // 0 probability, so in log likelihood -inf
      continue;
    } else if(observed < predicted - D_SHORT) {
      w += P_D_SHORT;
    } else if(observed > predicted + D_LONG) {
      w += P_D_LONG;
    } else {
      w += pow(observed-predicted, 2) / VAR_LIKELIHOOD;
    }

    // predicted = (scan_ptr[k] - location_lidar_map).norm();
    // w += pow(ranges[k]-predicted, 2) / VAR_LIKELIHOOD;
    // w += pow(ranges[k]-scan_ptr[k], 2) / VAR_LIKELIHOOD;
    // std::cout << "predicted" << predicted << std::endl;
    // std::cout << "range" << ranges[k] << std::endl;
    // std::cout << "range-predicted" << (ranges[k]-predicted) << std::endl;
  }
  // log likelihoods
  (*p_ptr).weight = -1 * GAMMA_LIKELIHOOD * w;
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles';
  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:
  // float x = rng_.UniformRandom(0, 1);
  // printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
  //        x);

  // Low-Variance Resampling 03/31

  // vector<Particle> newParticles;
  // newParticles.resize(N_PARTICLES);
  // // W sum of all weights w_i
  // double sum = 0;
  // double w;
  // for(size_t k = 0; k < particles_.size(); ++k) {
  //   w = exp(particles_[k].weight);
  //   particles_[k].weight = w;
  //   sum += w;
  // }
  // // draw N particles
  // float x;
  // for(int k=0; k<N_PARTICLES-1; k+=2) {
  //   x = rng_.UniformRandom(0, sum);
  //   w = 0;
  //   // for each particle weight wi
  //   for(Particle p:particles_) {
  //     w += p.weight;
  //     if(w > x) {
  //       Particle newParticle = {p.loc, p.angle, p.weight};
  //       newParticles[k] = newParticle;
  //       break;
  //     }
  //   }
  //   w = sum;
  //   for(Particle p:particles_) {
  //     w -= p.weight;
  //     if(w < x) {
  //       Particle newParticle = {p.loc, p.angle, p.weight};
  //       newParticles[k+1] = newParticle;
  //       break;
  //     }
  //   }
  // }
  // particles_ = newParticles;

  vector<Particle> newParticles;
  newParticles.resize(N_PARTICLES);
  // W sum of all weights w_i
  double sum = 0;
  double w;
  for(size_t k = 0; k < particles_.size(); ++k) {
    w = exp(particles_[k].weight);
    particles_[k].weight = w;
    sum += w;
  }
  // draw N particles
  float x = rng_.UniformRandom(0, sum);
  float dist = sum / N_PARTICLES;
  for(int k=0; k<N_PARTICLES; k++) {
    w = 0;
    // for each particle weight wi
    for(Particle p:particles_) {
      w += p.weight;
      if(w >= x) {
        Particle newParticle = {p.loc, p.angle, p.weight};
        newParticles[k] = newParticle;
        break;
      }
    }
    x += dist;
    if(x > sum) {
      x -= sum;
    }
  }
  particles_ = newParticles;

  // vector<Particle> newParticles;
  // newParticles.resize(N_PARTICLES);
  // // W sum of all weights w_i
  // double sum = 0;
  // double w;
  // for(size_t k = 0; k < particles_.size(); ++k) {
  //   w = exp(particles_[k].weight);
  //   particles_[k].weight = w;
  //   sum += w;
  // }
  // // draw N particles
  // float x;
  // for(int k=0; k<N_PARTICLES; k++) {
  //   x = rng_.UniformRandom(0, sum);
  //   w = 0;
  //   // for each particle weight wi
  //   for(Particle p:particles_) {
  //     w += p.weight;
  //     if(w > x) {
  //       Particle newParticle = {p.loc, p.angle, p.weight};
  //       newParticles[k] = newParticle;
  //       break;
  //     }
  //   }
  // }
  // particles_ = newParticles;
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.

  int size = particles_.size();
  if(size <= 0)
    return;

  double w_max;
  for(int k=0; k<size; k++) {
    Update(ranges,range_min,range_max,
        angle_min, angle_max, &(particles_[k]));
        
    if(k==0 || particles_[k].weight > w_max) {
      w_max = particles_[k].weight;
    }
  }
  n_updates++;

  // Normalize with max
  for(int k=0; k<size; k++) {
    particles_[k].weight -= w_max;
  }

  // Resample
  if(n_updates >= N_UPDATES_RESAMPLE) {
    Resample();
    n_updates = 0;
  }
}

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.

  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
  // float x = rng_.Gaussian(0.0, 2.0);
  // printf("Random number drawn from Gaussian distribution with 0 mean and "
  //        "standard deviation of 2 : %f\n", x);

  if(odom_initialized_) {
    // Car reference frame
    Rotation2Df rOdo(-prev_odom_angle_);
    Vector2f loc_car = rOdo * (odom_loc - prev_odom_loc_);

    float epsilonX, epsilonY, epsilonTheta, deltaAngle;
    Vector2f loc_delta, newLoc;
    // bool intersects;
    int size = particles_.size();
    for(int k=0; k<size; k++) {
      // Map reference frame
      Rotation2Df rMap(particles_[k].angle);
      loc_delta = rMap * loc_car;
      deltaAngle = getValidAng(odom_angle - prev_odom_angle_);

      // Gaussian for random error
      epsilonX = rng_.Gaussian(0.0, K1 * loc_delta.norm() + K2 * abs(deltaAngle));
      epsilonY = rng_.Gaussian(0.0, K1 * loc_delta.norm() + K2 * abs(deltaAngle));
      epsilonTheta = rng_.Gaussian(0.0, K3 * loc_delta.norm() + K4 * abs(deltaAngle));

      // Update particle
      particles_[k].angle = getValidAng(particles_[k].angle + deltaAngle + epsilonTheta);
      particles_[k].loc += loc_delta + Vector2f(epsilonX, epsilonY);
      // Only update the particle if it is not hitting a wall
      // newLoc = particles_[k].loc + loc_delta + Vector2f(epsilonX, epsilonY);
      // if(!map_.Intersects(particles_[k].loc, newLoc)) {
      //   particles_[k].loc = newLoc;
      // } else { // Set this particle same as the next one
      //   if(k > 0)
      //     particles_[k] = particles_[k-1];
      //   else
      //     particles_[k] = particles_[size-1];
      // }
    }
  }
  odom_initialized_ = true;
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);
  particles_.resize(N_PARTICLES);
  particles_.clear();
  
  float epsilonX, epsilonY, epsilonTheta;
  for (int i = 0; i < N_PARTICLES; i++){
    epsilonX = rng_.Gaussian(0.0, INIT_VAR_LOC);
    epsilonY = rng_.Gaussian(0.0, INIT_VAR_LOC);
    epsilonTheta = rng_.Gaussian(0.0, INIT_VAR_ANG);
 
    Vector2f newLoc(loc.x() + epsilonX, loc.y() + epsilonY);
    float newAngle = getValidAng(angle + epsilonTheta);
 
    Particle newParticle = {newLoc, newAngle, 1};
    particles_.push_back(newParticle);
  }
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  // loc = Vector2f(0, 0);
  // angle = 0;

  // Calculate means
  // loc = Vector2f(0, 0);
  // float angX = 0;
  // float angY = 0;
  // // float sum = particles_.size();
  // float sum = 0;
  // for(Particle p : particles_) {
  //   loc += p.loc * p.weight;
  //   angY += sin(p.angle) * p.weight;
  //   angX += cos(p.angle) * p.weight;
  //   sum += p.weight;
  // }
  // loc.x() /= sum;
  // loc.y() /= sum;
  // angX /= sum;
  // angY /= sum;
  // if(angX == 0 && angY == 0)
  //   angle = 0;
  // else
  //   angle = atan2(angY, angX);

  if(particles_.size() <= 0) {
    return;
  }
  double w_max = particles_[0].weight;
  loc = particles_[0].loc;
  angle = particles_[0].angle;
  float angY = sin(particles_[0].angle);
  float angX = cos(particles_[0].angle);
  float sum = 1;
  for(Particle p : particles_) {
    if(p.weight > w_max) {
      loc = p.loc;
      angle = p.angle;
      angY = sin(p.angle);
      angX = cos(p.angle);
      sum = 1;
      w_max = p.weight;
    } else if(p.weight == w_max) {
      loc += p.loc;
      angY += sin(p.angle);
      angX += cos(p.angle);
      sum++;
    }
  }
  if(sum > 1) {
    loc.x() /= sum;
    loc.y() /= sum;
    angX /= sum;
    angY /= sum;
    if(angX == 0 && angY == 0)
      angle = 0;
    else
      angle = atan2(angY, angX);
  }

  // if(particles_.size() <= 0) {
  //   return;
  // }
  // double w_max = particles_[0].weight;
  // loc = particles_[0].loc;
  // angle = particles_[0].angle;
  // for(Particle p : particles_) {
  //   if(p.weight > w_max) {
  //     loc = p.loc;
  //     angle = p.angle;
  //     w_max = p.weight;
  //   }
  // }
}


}  // namespace particle_filter
