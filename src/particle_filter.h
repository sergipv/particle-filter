/*
 * particle_filter.h
 *
 * 2D particle filter class.
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "helper_functions.h"

struct Particle {
	int id;
	double x;
	double y;
	double theta;
	double weight;
	std::vector<int> associations;
	std::vector<double> sense_x;
	std::vector<double> sense_y;
};

class ParticleFilter {
	// Number of particles to draw
	int num_particles; 
	
	// Flag, if filter is initialized
	bool is_initialized;
	
	// Vector of weights of all particles
	std::vector<double> weights;
	
public:
	// Set of current particles
	std::vector<Particle> particles;

	// Constructor
	// @param num_particles Number of particles
	ParticleFilter(int np) : num_particles(np), is_initialized(false), weights(np, 1) {}

	// Destructor
	~ParticleFilter() {}

	/**
	 * init Initializes particle filter by initializing particles to Gaussian
	 *   distribution around first position and all the weights to 1.
	 * @param x Initial x position [m] (simulated estimate from GPS)
	 * @param y Initial y position [m]
	 * @param theta Initial orientation [rad]
	 * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 */
	void init(double x, double y, double theta, double std[]);

	/**
	 * prediction Predicts the state for the next time step
	 *   using the process model.
	 * @param delta_t Time between time step t and t+1 in measurements [s]
	 * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 * @param velocity Velocity of car from t to t+1 [m/s]
	 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
	 */
	void prediction(double delta_t, double std_pos[],
      double velocity, double yaw_rate);

	/**
	 * updateWeights Updates the weights for each particle based on the likelihood of the 
	 *   observed measurements. 
	 * @param sensor_range Range [m] of sensor
	 * @param std_landmark[] Array of dimension 2 [Landmark measurement uncertainty [x [m], y [m]]]
	 * @param observations Vector of landmark observations
	 * @param map Map class containing map landmarks
	 */
	void updateWeights(double sensor_range, double std_landmark[],
      std::vector<LandmarkObs> observations,
			Map map_landmarks);
	
	/**
	 * resample Resamples from the updated set of particles to form
	 *   the new set of particles.
	 */
	void resample();

  /**
	 * initialized Returns whether particle filter is initialized yet or not.
   */
	const bool initialized() const {
		return is_initialized;
	}
  /*
	 * Set a particles list of associations, along with the associations calculated world x,y coordinates
	 * This can be a very useful debugging tool to make sure transformations are correct and assocations correctly connected
	 */
	Particle SetAssociations(Particle &particle,
      const std::vector<int>& associations,
      const std::vector<double>& sense_x,
      const std::vector<double>& sense_y);

	std::string getAssociations(Particle best);
	std::string getSenseX(Particle best);
	std::string getSenseY(Particle best);

private:
  /**
	 * prediction_straight Predicts the state for the next time step
	 *   using the process model considering the particle is moving
   *   straight.
	 * @param delta_t Time between time step t and t+1 in measurements [s]
	 * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 * @param velocity Velocity of car from t to t+1 [m/s]
	 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
	 */
	void prediction_straight(double delta_t, double std_pos[],
      double velocity, double yaw_rate);

  /**
	 * prediction_non_straight Predicts the state for the next time step
	 *   using the process model considering the particle is not moving
   *   straight.
	 * @param delta_t Time between time step t and t+1 in measurements [s]
	 * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 * @param velocity Velocity of car from t to t+1 [m/s]
	 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
	 */
	void prediction_non_straight(double delta_t, double std_pos[],
      double velocity, double yaw_rate);
	
	/**
	 * updateObservationId Finds which observations correspond to
   * which landmarks (likely by using a nearest-neighbors data association).
	 * @param predicted Vector of predicted landmark observations
	 * @param observations Vector of landmark observations
	 */
	void updateObservationId(std::vector<LandmarkObs> landmarks,
      std::vector<LandmarkObs>& observations);

  /**
   *  landmarksInrange Removes the landmarks that are out of range.
   *  @param particle Particle that we are using as reference.
   *  @param landmarks Landmarks we want to filter
   */
  std::vector<LandmarkObs> landmarksInRange(const Particle &p,
     const Map &map, double range);

  /**
   * landmarkById returns the LandmarkObs from landmarks with ID = id.
   * @param landmarks Vector with all landmarks.
   * @param id Reference id we are trying to find.
   */
  LandmarkObs landmarkById(const std::vector<LandmarkObs> &landmarks, int id);

  /**
   * observedProbability computes the probability that we are obseving a
   * specific landmark (in world coordinates).
   * @param real LandmarkObs with the real world coordinates of the landmark.
   * @param observed LandmarkObs with the real world coordinates considering
   * the observation from a specific particle.
   * @param std_landmark Std from the measurement error.
   */
  double observedProbability(LandmarkObs real,
      LandmarkObs observation, double std_landmark[]);

  /**
   * worldCoordinates Transforms the observations from vehicle coordinates to
   * world coordinates.
   * @param p Particle we consider represents the real location of the object.
   * @param observations LandmarkObs from the observations made by the object.
   */
  std::vector<LandmarkObs> worldCoordinates(const Particle &p,
      const std::vector<LandmarkObs> &observations);
};

#endif /* PARTICLE_FILTER_H_ */
