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
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    //cout<< "here" <<endl;

    default_random_engine gen;
    double std_x, std_y, std_theta;

    num_particles = 5;
    std_x = std[0];
    std_y = std[1];
    std_theta = std[2];

    normal_distribution<double> dist_x(x, std_x);
    normal_distribution<double> dist_y(y, std_y);
    normal_distribution<double> dist_theta(theta, std_theta);

    // define size of particles vector
    particles.resize(num_particles);

    for (int i = 0; i < num_particles; i++){

        particles[i].id = i;
        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);
        particles[i].weight = 1.0;
        }

    is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    default_random_engine gen;
    double std_x, std_y, std_theta;

    std_x = std_pos[0];
    std_y = std_pos[1];
    std_theta = std_pos[2];

    normal_distribution<double> dist_x(0.0, std_x);
    normal_distribution<double> dist_y(0.0, std_y);
    normal_distribution<double> dist_theta(0.0, std_theta);

    for (int i = 0; i < num_particles; i++){
        if (fabs(yaw_rate) > 0.001){
            particles[i].x += (velocity / yaw_rate) * ( sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta) );
            particles[i].y += (velocity / yaw_rate) * ( cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t) );
            particles[i].theta += yaw_rate * delta_t;
        }
        else {
            particles[i].x += velocity * cos(particles[i].theta) * delta_t;
            particles[i].y += velocity * sin(particles[i].theta) * delta_t;
        }

        // Add random noise
        particles[i].x += dist_x(gen);
        particles[i].y += dist_y(gen);
        particles[i].theta += dist_theta(gen);

    }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

    // Associate landmark ID to sensor observation using Nearest Neighbour Data Association

    for (int iObs = 0; iObs < observations.size(); iObs++){
        double min_dist = 1000000.0; //large value
        double dist_obs_land;
        //cout << "observation id before = " << observations[iObs].id <<"" << "x = "<<observations[iObs].x << "" << "y = "<< observations[iObs].y << endl;
        for (int jLand = 0; jLand < predicted.size(); jLand++){
            dist_obs_land = dist(observations[iObs].x, observations[iObs].y, predicted[jLand].x, predicted[jLand].y);
            if (dist_obs_land < min_dist){
                min_dist = dist_obs_land;
                observations[iObs].id = predicted[jLand].id;
            }
        }
        //cout << "observation id after" << observations[iObs].id <<endl;

    }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

    double sigma_x2 = std_landmark[0] * std_landmark[0];
    double sigma_y2 = std_landmark[1] * std_landmark[1];
    double gauss_norm =(1/(2 * M_PI * std_landmark[0] * std_landmark[1]));
    double sum_weights = 0.0;

    for (int i = 0; i < num_particles; i++){
        double theta = particles[i].theta;
        double x = particles[i].x;
        double y = particles[i].y;
        vector<LandmarkObs> transformed_particles_obs;
        for(int jObs = 0; jObs < observations.size(); jObs++){
            LandmarkObs transformed_particle_obs;

            // transformed observation
            //transformed_particle_obs.id = observations[jObs].id;
            transformed_particle_obs.id = jObs;
            //cout <<"j = " <<jObs << "id = " << transformed_particle_obs.id << endl;
            transformed_particle_obs.x = observations[jObs].x * cos(theta) - observations[jObs].y * sin(theta) + x;
            transformed_particle_obs.y = observations[jObs].x * sin(theta) + observations[jObs].y * cos(theta) + y;
            transformed_particles_obs.push_back(transformed_particle_obs);
        }

        // Find landmarks within range of sensor
        vector<LandmarkObs> map_landmarks_within_range;

        for (int kLand = 0; kLand < map_landmarks.landmark_list.size(); kLand++){
            LandmarkObs single_landmark;
            double dist_particle_to_landmark;

            single_landmark.id = map_landmarks.landmark_list[kLand].id_i;
            single_landmark.x = map_landmarks.landmark_list[kLand].x_f;
            single_landmark.y = map_landmarks.landmark_list[kLand].y_f;

            dist_particle_to_landmark = dist(x, y, single_landmark.x, single_landmark.y);
            if (dist_particle_to_landmark < sensor_range){
                map_landmarks_within_range.push_back(single_landmark);
                }
        }


        // carry out data association
        dataAssociation(map_landmarks_within_range, transformed_particles_obs);

        //Calculate weight after data association
        double updated_weight = 1.0;
        double exponent;
        double multi_gauss_prob;
        double mu_x, mu_y;
        //cout<<"size = "<< map_landmarks_within_range.size()<<endl;
        for (int iTobs = 0; iTobs < transformed_particles_obs.size(); iTobs++){
            double idTObs = transformed_particles_obs[iTobs].id;
            double xTobs = transformed_particles_obs[iTobs].x;
            double yTobs = transformed_particles_obs[iTobs].y;
            for (int jLand = 0; jLand < map_landmarks_within_range.size(); jLand++){
                if (idTObs == map_landmarks_within_range[jLand].id){
                    mu_x = map_landmarks_within_range[jLand].x;
                    mu_y = map_landmarks_within_range[jLand].y;
                    }
            }
            double diff_x = xTobs - mu_x;
            double diff_y = yTobs - mu_y;
            //cout << "xtobs = "<<xTobs<< "mux = "<<mu_x<<endl;
            exponent = -0.5 *( ((diff_x * diff_x)/sigma_x2) + ((diff_y * diff_y)/sigma_y2) );
            multi_gauss_prob = gauss_norm * exp(exponent);
            //cout << "i = "<<i<<"weight = "<<multi_gauss_prob<<endl;
            updated_weight *= multi_gauss_prob;
        }

        sum_weights += updated_weight;
        //cout << "i = "<<i<<"weight = "<<updated_weight<<endl;
        particles[i].weight = updated_weight;

    } // particles loop

    // Normalize weights
    weights.resize(num_particles);
    for (int i = 0; i < num_particles; i++){
        particles[i].weight = particles[i].weight / sum_weights;
        weights[i] = particles[i].weight; // need in resampling step
        //cout << "i = "<<i<<"weight = "<<particles[i].weight<<endl;
    }

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    default_random_engine gen;
    int particle_num_resample;
    //cout << "weight 1 = "<<particles[0].weight<<"last = "<<particles[num_particles-1].weight<<endl;
    std::discrete_distribution<> particle_dist(weights.begin(), weights.end());
	vector<Particle> new_particles;
    new_particles.resize(num_particles);
    for (int i = 0; i < num_particles; i++){
        particle_num_resample = particle_dist(gen);
        //cout << "particle_num_resample = "<< particle_num_resample << endl;
        new_particles[i] = particles[particle_num_resample];
    }

    particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
