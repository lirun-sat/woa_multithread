#ifndef CALC_UTILS
#define CALC_UTILS


#include<stdio.h>
#include<cmath>
#include<iostream>
#include<ctime>
#include<cstring>
#include<cstdlib>
#include <vector>
#include <random>
#include <chrono>

// using namespace std;

// default_random_engine rng;
// default_random_engine rng(chrono::system_clock::now().time_since_epoch().count());
// default_random_engine rng();

void random_double_generation(double* random, int num, double min, double max);

void MatrixMulti(double* A, double* B, double* AB);

void MatrixMulti_(int row_1, int col_1, int col_2, double* A, double* B, double* AB);

void zyx2quaternion(double psi, double theta, double phi, double* quaternion);

void quaternion2dc(double q_0, double q_1, double q_2, double q_3, double* dc);

void rpy2dc(double roll, double pitch, double yaw, double* direction_cosine);

void dc2euler(double* R, double* euler);

void euler_ZYX2dc(double alpha, double beta, double gamma, double* A);

void quaternion2euler(double w, double x, double y, double z, double* euler);

void MatrixMultiTranspose(double* A, double* B, double* AB);

void MatrixTranspose_(int row, int col, double* M, double* M_T);

void MatrixTranspose(double* M);

void MatrixAdd_(int row, int col, double* A, double* B, double* AB);

void MatrixSub_(int row, int col, double* A, double* B, double* AB);

void ScaleMatrix_(int row, int col, double scale, double* A, double* scale_A);

void SetZeroMatrix_(int row, int col, double* A);

void MatrixExtract_(int m, int n, int i_s, int i_f, int j_s, int j_f, double *A, double *ans);

void MatrixCopy_(int m, int n, double *a, double *u);

void MatrixDiagExpand(double* A, int row, int col, double* A_expand);

void cross(double* V, double* M);

void cxyz(double theta, int x, int y, int z, double* direction_cosines);

void LUP_Descomposition(double* A, double* L, double* U, int* P, int n);

void LUP_Solve(double* L, double* U, int* P, double* b, int n, double* inv_A_each);

int getNext(int i, int m, int n);

int getPre(int i, int m, int n);

void movedata(double *mtx, int i, int m, int n);

void transpose(double *mtx, int m, int n);

void LUP_solve_inverse(double* A, int n, double* inv_A);

double calc_determinantOfMatrix(double* a, int num, int row, int col, int n);

void calc_Euler_dev2angvl(double* Euler_angle, double* Euler_dev, double* angvl);


double rand_range(const double& min_val, const double& max_val);
std::vector<double> rand_range_vector(const std::vector<double>& min_val, const std::vector<double>& max_val);
double rand_normal(const double& mean_val, const double& std_val);
double rand_levy(const double& beta);
double sign_func(const double& x);

std::vector<double> vector_multiply(const double& scalar, const std::vector<double>& vector_1);
std::vector<double> vector_add(const std::vector<double>& vector_1, const std::vector<double>& vector_2);
std::vector<double> vector_subtract(const std::vector<double>& vector_1, const std::vector<double>& vector_2);
std::vector<double> vector_dot_vector(const std::vector<double>& vector1, const std::vector<double>& vector2);
std::vector<double> vector_min(const std::vector<double>& vector1, const std::vector<double>& vector2);
std::vector<double> vector_max(const std::vector<double>& vector1, const std::vector<double>& vector2);
std::vector<double> vector_fabs(const std::vector<double>& vector1);

std::vector<double> vector_pow(const std::vector<double>& vector1, const double& power);

// vector<double> vector_normalize(const vector<double>& vector_1);
// double vector_norm(const vector<double>& vector_1);
// vector<double> vector_rotate(const vector<double>& vector_1, const vector<double>& vector_2, const double& angle);
// vector<double> vector_cross(const vector<double>& vector_1, const vector<double>& vector_2);




#endif 
