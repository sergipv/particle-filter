/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  particles.reserve(num_particles);

  default_random_engine gen;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  for (int i=0; i<num_particles; i++) {
    particles.push_back(
      Particle{i, dist_x(gen), dist_y(gen), dist_theta(gen), 1.0});
  }

  is_initialized = true;
}



void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	bool is_moving_straight = fabs(yaw_rate) < 0.001;
	if (is_moving_straight) {
		prediction_straight(delta_t, std_pos, velocity, yaw_rate);
	} else {
		prediction_non_straight(delta_t, std_pos, velocity, yaw_rate);
	}
}


void ParticleFilter::prediction_straight(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  double inc = velocity * delta_t;
  double theta_inc = yaw_rate * delta_t;

  default_random_engine gen;
  normal_distribution<double> dist_x(0.0, std_pos[0]);
  normal_distribution<double> dist_y(0.0, std_pos[1]);
  normal_distribution<double> dist_theta(0.0, std_pos[2]);

  for (Particle &p : particles) {
    p.x += inc * cos(p.theta) + dist_x(gen);
    p.y += inc * sin(p.theta) + dist_y(gen);
    p.theta += theta_inc + dist_theta(gen);
  }
}

void ParticleFilter::prediction_non_straight(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  double inc = velocity / yaw_rate;
  double theta_inc = yaw_rate * delta_t;

  default_random_engine gen;
  normal_distribution<double> dist_x(0.0, std_pos[0]);
  normal_distribution<double> dist_y(0.0, std_pos[1]);
  normal_distribution<double> dist_theta(0.0, std_pos[2]);

  for (Particle &p : particles) {
		p.x += inc * (sin(p.theta + theta_inc) - sin(p.theta)) + dist_x(gen);
		p.y += inc * (cos(p.theta) - cos(p.theta + theta_inc)) + dist_y(gen);
		p.theta += theta_inc + dist_theta(gen);
  }
}

void ParticleFilter::updateObservationId(std::vector<LandmarkObs> landmarks,
    std::vector<LandmarkObs> &observations) {
  for (LandmarkObs &observation : observations) {
    double min_distance = numeric_limits<double>::max();
    for (LandmarkObs landmark : landmarks) {
      double diff_x = landmark.x - observation.x;
      double diff_y = landmark.y - observation.y;
      double dist = diff_x*diff_x + diff_y*diff_y;
      if (dist < min_distance) {
        min_distance = dist;
        observation.id = landmark.id;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
  int i = 0;
  default_random_engine gen;

  for (Particle &p : particles) {
    p.weight = 1.0;
    
    // At this moment we are evaluating a specific particle. To optimize
    // computation, landmarks that are our of range from the vehicle are ignored.
    std::vector<LandmarkObs> landmarks_in_range =
      landmarksInRange(p, map_landmarks, sensor_range);
    // All observations are relative to the vehicle so we need to transform them
    // to world coordinates.
    std::vector<LandmarkObs> observations_world =
      worldCoordinates(p, observations);
    // Once we have world coordinates in both observations and landmarks, we can
    // compute which landmark is closer to each observation, and update the id
    // in the observations.
    
    updateObservationId(landmarks_in_range, observations_world);

    // The particle weights are the probability that the observation
    // correspond to the position of the landmarks with respect to the
    // particles.
    for (LandmarkObs observation : observations_world) {
      LandmarkObs real = landmarkById(landmarks_in_range, observation.id);
      p.weight *= observedProbability(real, observation, std_landmark);
    }
    weights[i] = p.weight;
    i++;
  }
}

std::vector<LandmarkObs> ParticleFilter::landmarksInRange(const Particle &p, const Map &map, double range) {
  std::vector<LandmarkObs> in_range;
  double range_sq = range * range;

  for (Map::single_landmark_s landmark : map.landmark_list) {
    double dx = landmark.x_f - p.x;
    double dy = landmark.y_f - p.y;
    // Using an approximation instead of the real range.
    if (dx*dx+dy*dy < range_sq) {
      in_range.push_back(LandmarkObs{landmark.id_i, landmark.x_f, landmark.y_f});
    }
  }
  return in_range;
}

std::vector<LandmarkObs> ParticleFilter::worldCoordinates(const Particle &p, const std::vector<LandmarkObs> &observations) {
  std::vector<LandmarkObs> observations_world;
  for (LandmarkObs l : observations) {
    double l_x = p.x + l.x * cos(p.theta) - l.y * sin(p.theta);
    double l_y = p.y + l.x * sin(p.theta) + l.y * cos(p.theta);
    observations_world.push_back(LandmarkObs{l.id, l_x, l_y});
  }
  return observations_world;
}

LandmarkObs ParticleFilter::landmarkById(const std::vector<LandmarkObs> &landmarks, int id) {
  for (LandmarkObs l : landmarks) {
    if (l.id == id) {
      return l;
    }
  }
  // Should never happen
  return landmarks[0];
}

double ParticleFilter::observedProbability(LandmarkObs real, LandmarkObs observation,
    double std_landmark[]) {
  double diff_x = observation.x - real.x;
  double diff_y = observation.y - real.y;
  double x = - diff_x * diff_x / (2 * std_landmark[0] * std_landmark[0]);
  double y = - diff_y * diff_y / (2 * std_landmark[1] * std_landmark[1]);

  return (1/(2*M_PI*std_landmark[0]*std_landmark[1]) * exp(x + y));
}

void ParticleFilter::resample() {
  std::vector<Particle> next_particles;
  default_random_engine gen;
  discrete_distribution<int> particle_index(weights.begin(), weights.end());

  for (int i=0; i<num_particles;i++) {
    int idx = particle_index(gen);
    next_particles.push_back(Particle{
      idx, particles[idx].x, particles[idx].y, particles[idx].theta, 1.0});
  }
  particles = next_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle,
    const std::vector<int>& associations,
    const std::vector<double>& sense_x,
    const std::vector<double>& sense_y) {
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
	vector<int> v = best.associations;
	stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseX(Particle best) {
	vector<double> v = best.sense_x;
	stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseY(Particle best) {
	vector<double> v = best.sense_y;
	stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
