#ifndef _PSO_ALGORITHM_H
#define _PSO_ALGORITHM_H

#include "data.h"
#include "utils.h"
#include "forward_kinematics.h"


// Define a struct to represent a particle
struct Particle_woa {
    std::vector<double> position;
    std::vector<double> r1;
    std::vector<double> r2;
    std::vector<double> r3;
    double fitness;
    double beta;
    double b;

    Particle_woa() {  // constructor
        position.resize(NUM_DIMENSIONS);
        r1.resize(NUM_DIMENSIONS);
        r2.resize(NUM_DIMENSIONS);
        r3.resize(NUM_DIMENSIONS);
        position = rand_range_vector(minPosition, maxPosition);
        fitness = calc_fitness_woa(position);
        beta = 1.5;
        b = 1;
    }

    // Define a function to update the fitness
    void update_fitness() {
        fitness = calc_fitness_woa(position);
    }

    void update_position( double a,  double a2,  Particle_woa& rand_particle_woa,  Particle_woa& best_particle_woa) {
        r1 = rand_range_vector(_vector_0, _vector_1);
        r2 = rand_range_vector(_vector_0, _vector_1);
        r3 = rand_range_vector(_vector_0, _vector_1);
        std::vector<double> A = vector_multiply(a, vector_subtract(vector_multiply(2, r1), _vector_1));
        std::vector<double> C = vector_multiply(2, r2);
        std::vector<double> l = vector_add(vector_multiply(a2 - 1, r3), _vector_1);
        double p = rand_range(0, 1);
        for (int i = 0; i < NUM_DIMENSIONS; i++) {
            if (p < 0.5) {
                if (std::fabs(A[i]) > 1) {
                    position[i] = rand_particle_woa.position[i] - A[i] * (std::fabs(C[i] * rand_particle_woa.position[i] - position[i])) + rand_range(0, 1) * sign_func(rand_range(0, 1) - 0.5) * rand_levy(beta);
                } else {
                    position[i] = best_particle_woa.position[i] - A[i] * (std::fabs(C[i] * best_particle_woa.position[i] - position[i])) + rand_range(0, 1) * sign_func(rand_range(0, 1) - 0.5) * rand_levy(beta);
                }
            }
            else {
                position[i] = std::fabs(best_particle_woa.position[i] - position[i]) * exp(b * l[i]) * cos(2 * M_PI * l[i]) + best_particle_woa.position[i] + rand_range(0, 1) * sign_func(rand_range(0, 1) - 0.5) * rand_levy(beta);
            }
            // clamp the position vector to the range [minPosition, maxPosition]
            // if the position is greater than maxPosition, set it to (position[j] + maxPosition[j]) / 2
            // if the position is less than minPosition, set it to (position[j] + minPosition[j]) / 2
            if (position[i] > maxPosition[i]) {
                position[i] = (position[i] + maxPosition[i]) / 2;
            }
            if (position[i] < minPosition[i]) {
                position[i] = (position[i] + minPosition[i]) / 2;
            }
        }
    }
};


#endif // _PSO_ALGORITHM_H