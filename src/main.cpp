#include "utils.h"
#include "data.h"
#include <thread>
#include <algorithm>
#include <numeric>
#include <limits>  
#include "woa.h"


// #define WMAX 0.9
// #define WMIN 0.1


void evaluate_fitness(Particle_woa& particle_woa);

void update_global_best(std::vector<Particle_woa>& particles, 
                        std::vector<double>& global_best_position, 
                        double& global_best_fitness, 
                        Particle_woa &best_particle_woa);

void update_particle(const double a, const double a2, Particle_woa &particle_woa, 
                        Particle_woa &rand_particle_woa, Particle_woa &best_particle_woa);

void run_woa(std::vector<Particle_woa>& particles, int num_threads);

// void reset_GuideCoe();


int main() 
{
    // set the start time of the program
    auto start = std::chrono::high_resolution_clock::now(); 

    // int num_threads = std::thread::hardware_concurrency();  // Get the number of threads supported by the hardware
    int num_threads = std::thread::hardware_concurrency();

    std::cout << "NUM_CALCULATIONS  " << NUM_CALCULATIONS << std::endl;
    std::cout << "NUM_PARTICLES  " << NUM_PARTICLES << std::endl;
    std::cout << "NUM_DIMENSIONS  " << NUM_DIMENSIONS << std::endl;
    std::cout << "MAX_ITER  " << MAX_ITER << std::endl;
    std::cout << "NUM_THREADS  " << num_threads << std::endl;

    for(int k = 0; k < NUM_CALCULATIONS; ++k)
    {
        // reset_GuideCoe();

        std::cout << "Calculation: " << k << std::endl;  

        // Create the particles
        std::vector<Particle_woa> particles_woa(NUM_PARTICLES);  

        // Run the PSO algorithm 
        run_woa(particles_woa, num_threads);  

        particles_woa.clear();

    }

    auto end = std::chrono::high_resolution_clock::now();  // set the end time of the program

    // print the total time taken by the program
    std::cout << "Total time taken by the program: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;

    return 0;
}



void run_woa(std::vector<Particle_woa>& particles_woa, int num_threads) 
{
    double a;
    double a2;
    int rand_particle_woa;

    // Create a particle to store the best particle
    Particle_woa best_particle_woa;  

    // Calculate the number of particles per thread
    int num_particles_woa_per_thread = NUM_PARTICLES / num_threads;  

    // Create a vector to store the global best position
    std::vector<double> global_best_position(NUM_DIMENSIONS);  

    // initialize the global best fitness to the maximum value of a double
    double global_best_fitness = std::numeric_limits<double>::max();  

    // Create the threads vector
    std::vector<std::thread> threads;   

    for (int i = 0; i < num_threads; ++i) 
    {
        // Create the threads and evaluate the fitness of each particle
        threads.emplace_back([&particles_woa, i, num_particles_woa_per_thread]() {
            for (int j = i * num_particles_woa_per_thread; j < (i + 1) * num_particles_woa_per_thread; ++j) {
                evaluate_fitness(particles_woa[j]);
            }
        });
    }

    // Wait for the threads to finish
    for (auto& thread : threads) 
    {
        thread.join();
    }

    // Update the global best position
    update_global_best(particles_woa, global_best_position, global_best_fitness, best_particle_woa);

    // Run the WOA algorithm
    for (int iter = 0; iter < MAX_ITER; ++iter) 
    {
        // update the a value
        a = 2 * (1 - (double)iter / (double)MAX_ITER);  

        // update the a2 value
		a2 = (-1) + (double)iter * ((-1) / (double)MAX_ITER);  

        // Generate a random number between 0 and NUM_PARTICLES - 1
        rand_particle_woa = rand() % ((NUM_PARTICLES - 1) - 0 + 1) + 0;  

        // clear the threads vector
        threads.clear();  

        for (int i = 0; i < num_threads; ++i) 
        { 
            threads.emplace_back([&particles_woa, &global_best_position, i, num_particles_woa_per_thread, a, a2, &best_particle_woa, rand_particle_woa]() {
                for (int j = i * num_particles_woa_per_thread; j < (i + 1) * num_particles_woa_per_thread; ++j)
                { 
                    auto get_other_particle = [j, rand_particle_woa]() -> int 
                    {
                        // Make a copy of rand_particle_woa
                        int temp_rand_particle_woa = rand_particle_woa; 

                        while (temp_rand_particle_woa == j) 
                        {
                            temp_rand_particle_woa = rand() % ((NUM_PARTICLES - 1) - 0 + 1) + 0;
                        }
                        return temp_rand_particle_woa;
                    };

                    // particles_woa[j].update_position(a, a2, particles_woa[get_other_particle()], best_particle_woa);
                    // particles_woa[j].update_fitness();
                    update_particle(a, a2, particles_woa[j], particles_woa[get_other_particle()], best_particle_woa);
                }
            });
        }

        for (auto& thread : threads) 
        {
            thread.join(); 
        }

        // Update the global best position
        update_global_best(particles_woa, global_best_position, global_best_fitness, best_particle_woa);

        if(iter % 50 == 0)
        {
            std::cout << "a is :" << a << std::endl;
			std::cout << "a2 is :" << a2 << std::endl;
            std::cout << "iteration is :" << iter << std::endl;
			std::cout << "global_best_fitness is :" << global_best_fitness << std::endl;

			for (auto x : global_best_position) 
            {
                std::cout << x << " ";
            }		
            std::cout << std::endl;
		}
			
		if (global_best_fitness < 1)
		{
			std::cout << "Find solution, iteration is: " << iter << std::endl;
            std::cout << "global_best_fitness is: " << global_best_fitness << std::endl;

			for (auto x : global_best_position) 
            {
                std::cout << x << " ";
            }		
            std::cout << std::endl;
			break;
		}
    }

    threads.clear();

    // Print the results
    std::cout << "Global best fitness: " << global_best_fitness << std::endl;
    std::cout << "Global best position: ";
    for (auto x : global_best_position) 
    {
        std::cout << x << " ";
    }
    std::cout << std::endl;

}




// Define a function to evaluate the fitness of a particle
void evaluate_fitness(Particle_woa& particle_woa) 
{
    particle_woa.update_fitness();
}

// Define a function to update the global best position
void update_global_best(std::vector<Particle_woa> &particles_woa, std::vector<double> &global_best_position, double &global_best_fitness, Particle_woa &best_particle_woa) 
{
    for (auto& particle_woa : particles_woa) 
    {
        if (particle_woa.fitness < global_best_fitness) 
        {  
            // minimize the fitness
            global_best_fitness = particle_woa.fitness;
            global_best_position = particle_woa.position;
            best_particle_woa = particle_woa;
        }
    }
}

//  Define a function to update the particles, a, a2, particles_woa[other_particle], best_particle_woa
void update_particle(const double a, const double a2, Particle_woa &particle_woa, Particle_woa &rand_particle_woa, Particle_woa &best_particle_woa) 
{
    particle_woa.update_position(a, a2, rand_particle_woa, best_particle_woa);
    particle_woa.update_fitness();
}

// void reset_GuideCoe()
// {
//     inertGuideCoe = 0.9;  
//     localGuideCoe = 1.47;  
//     globalGuideCoe = 1.47; 
// }