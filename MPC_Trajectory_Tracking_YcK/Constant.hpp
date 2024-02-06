#include <math.h>


namespace Constants {
    const int NX = 4; // State dimension (x, y, yaw (yaw angle), and velocity (v))
    const int T = 6; // Prediction Horizon
    const double DT = 0.2; // Time step
    const double MAX_STEER = 45.0 / 180 * M_PI;  // Maximum Steering Angle (rad)
    const double MAX_DSTEER = 30.0 / 180 * M_PI; // Maximum Steering Rate (rad/sec)
    const int MAX_ITER = 3; //  Maximum Iterations for MPC Solver
    const double DU_TH = 0.1;   // Change Threshold for Control Inputs for Optimization
    const int N_IND_SEARCH=10;  // Number of Index Search Steps
    const int MAX_TIME=5000; //  Maximum Simulation time in iterations
    const double WB=2.5;  // Wheelbase
    const double MAX_SPEED=55.0/3.6; // Maximum Speed (m/s)
    const double MIN_SPEED=55.0/3.6; // Minimum Speed (m/s)
    const double MAX_ACCEL=1.0; // Maximum Acceleration
    const double LENGTH=4.5;
    const double WIDTH =4.5;
    const double BACKTOWHEEL=1.0;
    const double WHEEL_LEN=0.3;
    const double WHEEL_WIDTH=0.2;
    const double TREAD=0.7;

}
