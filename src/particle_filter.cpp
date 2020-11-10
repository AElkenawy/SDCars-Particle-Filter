  /**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <stdlib.h>
#include <iostream>    
#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;

// Random number generator (gen)
std::default_random_engine gen;


// ############## Particle filter intialization ############## //
void ParticleFilter::init(double x, double y, double theta, double std[]) {
  
  /* Gaussian noise normal distributions added to GPS measurements of
   * x, y, theta as mean. std: corresponding uncertainities */
  normal_distribution<double> dist_x(x, std[0]); 
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  
  // Setting number of particles
  num_particles = 100;
  
  for (int i = 0; i < num_particles; ++i)   {
    // Particle struct instantiation
    Particle prt;
    
    // Intialization particle structure
    prt.id=i;
    // Initializing particles weights to 1
    prt.weight = 1.0;
    // Gaussian noise
    prt.x=dist_x(gen);
    prt.y=dist_y(gen);
    prt.theta=dist_theta(gen);
    
    // Appending particle intial values
    particles.push_back(prt);
  }
  is_initialized = true;
}

// ############## System states prediction (motion model) ############## //
void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {

  /* Gaussian noise normal distributions add to system states of
   * x, y, theta as mean. std_pos: corresponding uncertainities */
  normal_distribution<double> dist_x(0, std_pos[0]); 
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);

  for (int i = 0; i < num_particles; ++i) {
    
    // Storing particles heading state
    double theta = particles[i].theta; 
    
    // Constant velocity (CV) motion model for straight-line driving
    if (fabs(yaw_rate) < 0.00001){
      
      // CV model state transition formulas
      particles[i].x += velocity*cos(theta)*delta_t;
      particles[i].y += velocity*sin(theta)*delta_t;
      particles[i].theta += yaw_rate*delta_t;
    }
    
    // Constant turn rate and velocity (CTRV) motion model
    else {
      
      // CTRV model state transition formulas
      particles[i].x += velocity/yaw_rate*(sin(yaw_rate*delta_t + theta)-sin(theta));
      particles[i].y += velocity/yaw_rate*(-cos(yaw_rate*delta_t + theta)+cos(theta));  
      particles[i].theta += yaw_rate*delta_t;
    }
 
    // Gaussian noise addition
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);   
  }
}

// ############## Observations association to landmarks ##############
void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {

  double min_distance;
  
  // Looping through surrounding landmarks observations
  for (int j = 0; j < observations.size(); ++j){
    
    // Setting minimum distance initially to a large value
    min_distance = std::numeric_limits<double>::max();
    
    // Looping through predicted landmarks observations
    for (int i = 0; i < int(predicted.size()); ++i) {
      // Calculating euclidean distance between observed point predicted point
      double distance = dist(observations[j].x, observations[j].y, predicted[i].x, predicted[i].y);
      // Assigning observation id to the nearest predicted landmark
      if (distance < min_distance){
        min_distance = distance;
        observations[j].id = predicted[i].id;
      }
    }
  }
}    
     
// ############## Particles weights update ############## //
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {

  for (int i = 0; i < num_particles; ++i){
    
    // Setting particles weights to 1
    particles[i].weight=1.0;

    /* 1) Transformation of observations coordinates (from local vehicle/particle 
     * coordinates to global coordinates, matching landmarks coordinates) */

    // Storing particles position states
    double x_part = particles[i].x;
    double y_part = particles[i].y;
    double theta = particles[i].theta;

    // LandmarkObs vector declartion to store global observations
    vector<LandmarkObs> glob_observs;

    // Looping through observations      
    for (int j =0; j < observations.size(); ++j) {
      double x_obs = observations[j].x;
      double y_obs = observations[j].y;
      double glob_x = x_part + (cos(theta) * x_obs) - (sin(theta) * y_obs);
      double glob_y = y_part + (sin(theta) * x_obs) + (cos(theta) * y_obs);

      // Appending global coordinates to glob_observs vector
      glob_observs.push_back(LandmarkObs{j, glob_x, glob_y});
    }

    // LandmarkObs vector declartion to store predicted observations (within sensor range)
    vector<LandmarkObs> predictions;

    // Looping through map landmarks list
    for(int k = 0; k < map_landmarks.landmark_list.size(); k++) {

      // Storing map landmarks coordinates and id
      float lmrk_x = map_landmarks.landmark_list[k].x_f;
      float lmrk_y = map_landmarks.landmark_list[k].y_f;
      int lmrk_id = map_landmarks.landmark_list[k].id_i;

      // Appending landmarks existing within sensor range distance 
      if(dist(lmrk_x, lmrk_y, x_part, y_part) <= sensor_range) {
        predictions.push_back(LandmarkObs{lmrk_id, lmrk_x, lmrk_y});
      }
    }

    /* 2) Observations IDs association to landmarks */

    dataAssociation(predictions, glob_observs);

    /* 3) Particles Weights update (using Multi-variate Gaussian prob. distribution) */
    
    // Landmark measurement uncertainties
    double sig_x = std_landmark[0];
    double sig_y = std_landmark[1];
    
    // Gaussian normalizer
    double gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

    // Looping through landmark predictions and particles global observataions
    for(int m = 0; m < predictions.size(); m++) {
      for(int n = 0; n < glob_observs.size(); n++) {
        
        /* Multi-variate Gaussian formula for particles weights update for a 
         * matched observation-prediction */
        if (glob_observs[n].id ==  predictions[m].id) {
          double x_diff = glob_observs[n].x - predictions[m].x;
          double y_diff = glob_observs[n].y - predictions[m].y;
          particles[i].weight *= gauss_norm*exp(-(pow(x_diff, 2) / (2 * pow(sig_x, 2)) + 
                                                   pow(y_diff, 2) / (2 * pow(sig_y, 2))));
        }  
      }
    }
  }
}

// ############## Particles resampling (with replacement) ##############
void ParticleFilter::resample() {
  
  // Vector declaration for weights storing (prior to resampling)
  vector<double> weights_r;
  
  for (int i = 0; i < num_particles; i++) {
    weights_r.push_back(particles[i].weight);
  }
  
  // Discrete probability distribution (based on particles weights)
  std::discrete_distribution<> weight_dist(weights_r.begin(), weights_r.end());

  vector<Particle> resampled_prt;
  
  // Resampling particles (indices) due to the discrete distribution of weights
  for(int i=0; i<num_particles; i++){
      int index = weight_dist(gen);
      resampled_prt.push_back(particles[index]);
  }
  
  particles = resampled_prt;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}