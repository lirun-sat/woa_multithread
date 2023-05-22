#include "utils.h"


// // Define a random number generator
// default_random_engine rng(chrono::system_clock::now().time_since_epoch().count());
// rng(chrono::system_clock::now().time_since_epoch().count());

// Define a function to generate a random number within a range
double rand_range(const double& min_val, const double& max_val) {
    std::default_random_engine rng(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> distr(min_val, max_val);
    return distr(rng);
}

// Define a function to generate a vector of random numbers within a range
std::vector<double> rand_range_vector(const std::vector<double>& min_val, const std::vector<double>& max_val) {
    std::vector<double> result;
    for (int i = 0; i < min_val.size(); ++i) {
        result.push_back(rand_range(min_val[i], max_val[i]));
    }
    return result;
}

// Define a function to generate a random number follows a normal distribution
double rand_normal(const double& mean_val, const double& std_val) {
    std::default_random_engine rng(std::chrono::system_clock::now().time_since_epoch().count());
    std::normal_distribution<double> distr(mean_val, std_val);
    return distr(rng);
}

// Define a function to generate a vector of which each element is random number following levy flight distribution
double rand_levy(const double& beta) {
    // double beta = 1.5;
    double alpha_u = pow(tgamma(1 + beta) * sin(M_PI * beta / 2) / (tgamma((1 + beta) / 2) * beta * pow(2, (beta - 1) / 2)), 1 / beta);
    double alpha_v = 1;
    double u = rand_normal(0, alpha_u);
    double v = rand_normal(0, alpha_v);
    double z = u / pow(fabs(v), 1 / beta);
    return z;

} 

// Define a function to return the sign of a number
double sign_func(const double& x) {
    if (x > 0) {
        return 1;
    } else if (x < 0) {
        return -1;
    } else {
        return 0;
    }
}

// define a function to multiply a vector by a scalar
std::vector<double> vector_multiply(const double& scalar, const std::vector<double>& vector_1) {
    std::vector<double> result;
    for (int i = 0; i < vector_1.size(); ++i) {
        result.push_back(scalar * vector_1[i]);
    }
    return result;
}

// define a function to add two vectors
std::vector<double> vector_add(const std::vector<double>& vector1, const std::vector<double>& vector2) {
    std::vector<double> result;
    for (int i = 0; i < vector1.size(); ++i) {
        result.push_back(vector1[i] + vector2[i]);
    }
    return result;
}

// define a function to subtract two vectors
std::vector<double> vector_subtract(const std::vector<double>& vector1, const std::vector<double>& vector2) {
    std::vector<double> result;
    for (int i = 0; i < vector1.size(); ++i) {
        result.push_back(vector1[i] - vector2[i]);
    }
    return result;
}

// define a function to multiply two vectors
std::vector<double> vector_dot_vector(const std::vector<double>& vector1, const std::vector<double>& vector2) {
    std::vector<double> result;
    for (int i = 0; i < vector1.size(); ++i) {
        result.push_back(vector1[i] * vector2[i]);
    }
    return result;
}

// define a function to find the minimum values of two vectors based on each element
std::vector<double> vector_min(const std::vector<double>& vector1, const std::vector<double>& vector2) {
    std::vector<double> result;
    for (int i = 0; i < vector1.size(); ++i) {
        result.push_back(std::min(vector1[i], vector2[i]));
    }
    return result;
}

// define a function to find the maximum values of two vectors based on each element
std::vector<double> vector_max(const std::vector<double>& vector1, const std::vector<double>& vector2) {
    std::vector<double> result;
    for (int i = 0; i < vector1.size(); ++i) {
        result.push_back(std::max(vector1[i], vector2[i]));
    }
    return result;
}

// define a function to return the absulute value of a vector
std::vector<double> vector_fabs(const std::vector<double>& vector1) {
    std::vector<double> result;
    for (int i = 0; i < vector1.size(); ++i) {
        result.push_back(std::fabs(vector1[i]));
    }
    return result;
}

// define a function to return the power of a vector
std::vector<double> vector_pow(const std::vector<double>& vector1, const double& power) {
    std::vector<double> result;
    for (int i = 0; i < vector1.size(); ++i) {
        result.push_back(std::pow(vector1[i], power));
    }
    return result;
}