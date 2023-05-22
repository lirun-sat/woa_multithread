

extern int N;


extern int NUM_CALCULATIONS;
extern int NUM_DIMENSIONS;
extern int NUM_PARTICLES;
extern int MAX_ITER;

extern std::vector<double> maxSpeed;
extern std::vector<double> minSpeed;
extern std::vector<double> maxPosition;
extern std::vector<double> minPosition;
extern std::vector<double> _vector_0;
extern std::vector<double> _vector_1;

// Define the PSO parameters
extern double inertGuideCoe;  // W = 0.7;    // inertia weight
extern double localGuideCoe;  // C1 = 1.2;   // cognitive weight
extern double globalGuideCoe;  // C2 = 1.2;   // social weight



extern double Ez[];
extern double eye[];

extern double joint_angle_min_limit_degree[];
extern double joint_angle_min_limit_rad[];
extern double joint_angle_max_limit_degree[];
extern double joint_angle_max_limit_rad[];

extern double joint_angle_velocity_min_limit;
extern double joint_angle_velocity_max_limit;
extern double joint_angle_acceleration_min_limit;
extern double joint_angle_acceleration_max_limit;

extern double rpy_joints[];
extern double m_b;
extern double b_b[];
extern double m[];
extern double a[];
extern double b[];
extern double I_b_body[];
extern double I_links_body[];
extern double q_INITIAL[];
extern double RPY_END_INITIAL[];
extern double delta_tau;
extern double Pe_DESIRED[];
extern double RPY_END_DESIRED[];
extern double RPY_BASE_INITIAL[];

//******************************************************************************************************************************

extern double base_center2vertex[][3];
extern double base_center2left_solar_vertex[][3];
extern double base_center2right_solar_vertex[][3];

extern double link_1_center2link_1_part_1_vertex[][3];
extern double link_1_center2link_1_part_2_vertex[][3];

extern double link_2_center2link_2_part_1_vertex[][3];
extern double link_2_center2link_2_part_2_vertex[][3];

extern double link_5_center2link_5_part_1_vertex[][3];
extern double link_5_center2link_5_part_2_vertex[][3];
extern double link_5_center2link_5_part_3_vertex[][3];
extern double link_5_center2link_5_part_4_vertex[][3];
extern double link_5_center2link_5_part_5_vertex[][3];

extern double link_6_center2link_6_part_1_vertex[][3];
extern double link_6_center2link_6_part_2_vertex[][3];

extern double link_7_center2link_7_vertex[][3];

extern int nvrtx;




extern double center2vertices[][8][3];

