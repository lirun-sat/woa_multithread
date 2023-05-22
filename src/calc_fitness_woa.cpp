#include "forward_kinematics.h"
#include "data.h"
#include "utils.h"


// define the fitness function
double calc_fitness_woa(const std::vector<double>& position) 
{
    // define the parameters
    double* para = new double[N];
	double* eta_end = new double;
	double* xi_end = new double[3];
	double* Pe = new double[3];
	double* eta_b = new double;
	double* xi_b = new double[3];
	double* quaternion_end_desired = new double[4];
	double* eta_end_desired = new double;
	double* xi_end_desired = new double[3];
	double* delta_eta_base = new double;
	double* delta_eta_end = new double;
	double* delta_xi_base = new double[3];
	double* delta_xi_end = new double[3];
	double* delta_Pe_end = new double[3];
    double* p_e_initial = new double[3];
    double* locus = new double;
    double* delta_xi_b_distrb_max = new double;
    double* manipl = new double;
    double* T_min = new double;
	double* collision_times = new double;

	for (int i = 0; i < N; i++)
		para[i] = position[i];

    double K_a = 1 / 0.0002;  
	double K_p = 1 / 0.002;  
    double K_s = 0;  
    double K_b = 1 / 0.0008;
    double K_M = 1;
    double K_t = 1 / 100;  
	double cost_func = 0;

	double delta_xi_end_mod_temp = 0;
	double delta_Pe_end_mod_temp = 0;
	double delta_xi_base_mod_temp = 0;

	forward_kin( para,  eta_end,  xi_end,  Pe,  eta_b,  xi_b,  p_e_initial,  locus,  delta_xi_b_distrb_max,  manipl, T_min, collision_times);

// ********************************************************  straight_line_locus  ************************************************************************************
    double delta_p_e = 0;
    double straight_line_locus = 0;
    for(int i = 0; i < 3; i++)
    {
        delta_p_e = p_e_initial[i] - Pe[i];
        straight_line_locus += delta_p_e * delta_p_e;
    }
    straight_line_locus = sqrt(straight_line_locus);
// **********************************************************************************************************************************************************************

	zyx2quaternion(RPY_END_DESIRED[2], RPY_END_DESIRED[1], RPY_END_DESIRED[0], quaternion_end_desired);

	*eta_end_desired = quaternion_end_desired[0];

	for (int i = 0; i < 3; i++)
	{
		xi_end_desired[i] = quaternion_end_desired[i + 1];
	}

	delta_var(Pe_DESIRED, eta_end_desired, xi_end_desired, Pe, *eta_end, xi_end, *eta_b, xi_b, delta_eta_end, delta_xi_end, delta_Pe_end, delta_eta_base, delta_xi_base);

	for (int i = 0; i < 3; i++)
	{
		delta_xi_end_mod_temp += delta_xi_end[i] * delta_xi_end[i];
		delta_Pe_end_mod_temp += delta_Pe_end[i] * delta_Pe_end[i];
		delta_xi_base_mod_temp += delta_xi_base[i] * delta_xi_base[i];
	}

	delta_xi_end_mod_temp = sqrt(delta_xi_end_mod_temp); 

    delta_Pe_end_mod_temp = sqrt(delta_Pe_end_mod_temp); 

    delta_xi_base_mod_temp = sqrt(delta_xi_base_mod_temp);


	// ****************************************  RPY error of end-effector,  Pe error  ********************************************************************************
    
	// cout << "manipl:" << "  " << (*manipl) << endl;

    cost_func = K_a * delta_xi_end_mod_temp 
                + K_p * delta_Pe_end_mod_temp 
                + K_b * (*delta_xi_b_distrb_max) 
                + K_s * fabs((*locus) - straight_line_locus)  
                + K_M * (1 / (*manipl))
                + K_t * (*T_min)
				+ (*collision_times);


    delete[] para;
    delete eta_end;
    delete[] xi_end ;
    delete[] Pe ;
    delete eta_b ;
	delete[] xi_b ;
	delete[] quaternion_end_desired;
	delete eta_end_desired;
	delete[] xi_end_desired ;
	delete delta_eta_base ;
	delete delta_eta_end ;
	delete[] delta_xi_base ;
	delete[] delta_xi_end ;
	delete[] delta_Pe_end ; 
	delete[] p_e_initial ;
	delete locus ;
	delete delta_xi_b_distrb_max;
	delete manipl;
	delete T_min;
	delete collision_times;


	return cost_func;

}


// int main() {
//     // set the start time of the program
//     auto start = chrono::high_resolution_clock::now();

//     vector<Particle> particles(NUM_PARTICLES);  // Create the particles

//     // Run the PSO algorithm using 2 threads
//     run_pso(particles, 2);

//     // set the end time of the program
//     auto end = chrono::high_resolution_clock::now();

//     // print the total time taken by the program
//     cout << "Total time taken by the program: " << chrono::duration_cast<chrono::milliseconds>(end - start).count() << " ms" << endl;

//     return 0;
// }
