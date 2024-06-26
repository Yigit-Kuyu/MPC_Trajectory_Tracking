#include<iostream>
#include<iomanip>
#include<limits>
#include<vector>
#include<opencv4/opencv2/opencv.hpp>
#include<opencv4/opencv2/core/core.hpp>
#include<opencv4/opencv2/highgui/highgui.hpp>
#include<sys/time.h>
#include<eigen3/Eigen/Eigen>
#include<cppad/cppad.hpp>
#include<cppad/ipopt/solve.hpp>
#include"Cubic_Spline_Interpolation.hpp"
#include"Vehicle_Model.hpp"
#include"Type_Alias.hpp"
#include"Constant.hpp"



using CppAD::AD;
using namespace vector_types;
using namespace parameters;
using namespace vect_difference;
using Eigen_ref=Eigen::Matrix<float, Constants::NX, Constants::T>;

int x_start = 0;
int y_start = x_start + Constants::T;
int yaw_start = y_start + Constants::T;
int v_start = yaw_start + Constants::T;

int delta_start = v_start + Constants::T;
int a_start = delta_start + Constants::T-1;

cv::Point2i cv_offset(float x, float y, int image_width=2000, int image_height=2000){
  cv::Point2i output;
  output.x = int(x * 20) + 300;
  output.y = image_height - int(y * 20) - image_height/5;
  return output;
};

void update(State& state, float a, float delta){
  if (delta >= Constants::MAX_STEER) delta = Constants::MAX_STEER;
  if (delta <= - Constants::MAX_STEER) delta = - Constants::MAX_STEER;

  state.x = state.x + state.v * std::cos(state.yaw) * Constants::DT;
  state.y = state.y + state.v * std::sin(state.yaw) * Constants::DT;
  state.yaw = state.yaw + state.v / Constants::WB * CppAD::tan(delta) * Constants::DT;
  state.v = state.v + a * Constants::DT;

  if (state.v > Constants::MAX_SPEED) state.v = Constants::MAX_SPEED;
  if (state.v < Constants::MIN_SPEED) state.v = Constants::MIN_SPEED;

};

Vec_f calc_speed_profile(Vec_f rx, Vec_f ry, Vec_f ryaw, float target_speed){
  Vec_f speed_profile(ryaw.size(), target_speed);

  float direction = 1.0;
  for(unsigned int i=0; i < ryaw.size()-1; i++){
    float dx = rx[i+1] - rx[i]; // Calculate the change in x-coordinate between the current and next trajectory interpolated points
    float dy = ry[i+1] - ry[i]; // Calculate the change in y-coordinate between the current and next trajectory interpolated points
    float move_direction = std::atan2(dy, dx); // Calculate the angle in rads (move_direction) between the current and next points. This angle represents the direction of movement between the points.

    if (dx != 0.0 && dy != 0.0){ // If both dx and dy are non-zero, calculate the absolute change in between  move_direction and the yaw (rotation)
      float dif_angle = std::abs(YAW_P2P(move_direction - ryaw[i])); // periodic normalization function,  ensures that the angle is within the range [-pi, pi],
      if (dif_angle >= M_PI/4.0) direction = -1.0; // if the angle difference is greater than or equal to pi/4 (45 degrees), set the direction to -1 (reverse direction); otherwise, set it to 1 (forward).
      else direction = 1.0;

    /*
    Setting the direction to -1 essentially signals that a significant change in direction has been detected,
    and it implies that the vehicle should move in the opposite or reverse direction to better align with the desired trajectory.
    */

    }

    // This negative speed can be interpreted as a deceleration or slowdown to account for the change in direction.
    if (direction != 1.0) speed_profile[i] = -1 * target_speed; // Based on the determined direction, if the direction is not 1, it sets the speed to the negative of the target_speed (indicating reverse movement).

  }
  speed_profile[-1] = 0.0;  // Sets the last element of the speed_profile vector to 0.0.
                           //  It ensures that the vehicle doesn't continue with any residual speed beyond the intended trajectory.
  return speed_profile;
};

int calc_nearest_index(State state, Vec_f cx, Vec_f cy, Vec_f cyaw, int target_indx){
  float mind = std::numeric_limits<float>::max(); // assign very high value for comparison in if condition (d_e<mind)
  float ind = 0;
  for(unsigned int i=target_indx; i<target_indx+Constants::N_IND_SEARCH; i++){
    float idx = cx[i] - state.x;        // cx: interpolated x values, state.x=cx[0]
    float idy = cy[i] - state.y;       //  cy: interpolated y values, state.y=cy[0]
    float d_e = idx*idx + idy*idy;    //  Euclidean distance between the vehicle's current state and the interpolated points

    if (d_e<mind){
      mind = d_e;
      ind = i;
    }
  }

  // float dxl = cx[ind] - state.x;
  // float dyl = cy[ind] - state.y;
  // float angle = YAW_P2P(cyaw[ind] - std::atan2(dyl, dxl));
  // if (angle < 0) mind = mind * -1;

  return ind; // nearest point index from the current state to the interpolated points
};


void calc_ref_trajectory(State state, Vec_f cx, Vec_f cy, Vec_f cyaw, Vec_f
ck, Vec_f sp, float dl, int& target_ind, Eigen_ref& xref){  // cx, cy: interpolated x, y values,
// ck: calculated curvature, sp: speed profile, dl=1, xref=Eigen::Matrix<float, NX, T>: NX=4 (row number), T=6 (column number)
// target_ind=0 (initial value)

  // Eigen_ref=Eigen::Matrix<float, NX, T>
  xref = Eigen_ref::Zero(); // predicted reference trajectory


  int ncourse = cx.size();

  int ind = calc_nearest_index(state, cx, cy, cyaw, target_ind); // nearest point index from the current state to the interpolated points
  if (target_ind >= ind) ind = target_ind;

  xref(0, 0) = cx[ind];
  xref(1, 0) = cy[ind];
  xref(2, 0) = cyaw[ind];
  xref(3, 0) = sp[ind];

  /*
  for (int i = 0; i < xref.rows(); ++i)
  {
    for (int j = 0; j < xref.cols(); ++j) {
        std::cout << xref(i, j) << " ";
    }
    std::cout << std::endl;
  }
  */

  float travel = 0.0;

  // PREDICTED HORIZON
  // T=6, it determines the number of columns in the xref matrix, representing the prediction horizon. This means the MPC is planning the vehicle's trajectory over the next 6 time steps into the future.
  // a prediction horizon is used to plan the vehicle's trajectory over a certain number of time steps into the future.
  for(int i=0; i<Constants::T; i++){ // DT: time sample, dl: res
    travel += std::abs(state.v) * Constants::DT; // state.v: velocity in current state, DT=0.2, traveled distance (travel) is established based on the assumption that
                                     //  the vehicle moves a certain distance per time step (DT) at its current velocity (state.v).

    // dl determines how much physical distance each index corresponds to.
    // If dl = 1, it implies that each index in the trajectory corresponds to a distance of 1 unit in the physical world (e.g., 1 meter).
    int discrete_distance_indx = (int)std::round(travel/dl); // dl=1, represents how many discrete indices forward the vehicle is expected to move.
    // int dind = i;

    //std::cout << "future index: " << discrete_distance_indx << " current index: " << i << std::endl;
    // Updates the predicted reference trajectory based on the expected future position of the vehicle.
    if ((ind+discrete_distance_indx)<ncourse){ // Access future points in the trajectory (cx, cy, cyaw, sp).
      xref(0, i) = cx[ind + discrete_distance_indx];
      xref(1, i) = cy[ind + discrete_distance_indx];
      xref(2, i) = cyaw[ind + discrete_distance_indx];
      xref(3, i) = sp[ind + discrete_distance_indx];
      // dref(0, i) = 0.0;
    }else{
      xref(0, i) = cx[ncourse - 1];
      xref(1, i) = cy[ncourse - 1];
      xref(2, i) = cyaw[ncourse - 1];
      xref(3, i) = sp[ncourse - 1];
      // dref(0, i) = 0.0;
    }
    std::cout << std::endl;
  }

  target_ind = ind; // Update the target_ind variable with the nearest index for the next iteration.
  std::cout << std::endl;
};

void smooth_yaw(Vec_f& cyaw){ // smooth out sudden jumps or discontinuities in the yaw angles of the trajectory
  for(unsigned int i=0; i<cyaw.size()-1; i++){
    float dyaw = cyaw[i+1] - cyaw[i];

    while (dyaw > M_PI/2.0){ // Adjustments for large positive jumps in yaw within a specified range [-pi/2, pi/2]
      cyaw[i+1] -= M_PI*2.0;
      dyaw = cyaw[i+1] - cyaw[i];
    }
    while (dyaw < -M_PI/2.0){ // Adjustments for large negative jumps in yaw within a specified range [-pi/2, pi/2]
      cyaw[i+1] += M_PI*2.0;
      dyaw = cyaw[i+1] - cyaw[i];
    }
  }
};


class FG_EVAL{
public:
  // Eigen::VectorXd coeeffs;
  Eigen_ref traj_ref; // Eigen_ref=Eigen::Matrix<float, NX, T>;

  FG_EVAL(Eigen_ref traj_ref){
    this->traj_ref = traj_ref;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector; // <cppad/cppad.hpp>

  void operator()(ADvector& fg, const ADvector& vars){
    fg[0] = 0;

    for(int i=0; i<Constants::T-1; i++){
      fg[0] +=  0.01 * CppAD::pow(vars[a_start+i], 2);
      fg[0] += 0.01 * CppAD::pow(vars[delta_start+i], 2);
    }

    for(int i=0; i<Constants::T-2; i++){
      fg[0] += 0.01 * CppAD::pow(vars[a_start+i+1] - vars[a_start+i], 2);
      fg[0] += 1 * CppAD::pow(vars[delta_start+i+1] - vars[delta_start+i], 2);
    }

    // fix the initial state as a constraint
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + yaw_start] = vars[yaw_start];
    fg[1 + v_start] = vars[v_start];

    // fg[0] += CppAD::pow(traj_ref(0, 0) - vars[x_start], 2);
    // fg[0] += CppAD::pow(traj_ref(1, 0) - vars[y_start], 2);
    // fg[0] += 0.5 * CppAD::pow(traj_ref(2, 0) - vars[yaw_start], 2);
    // fg[0] += 0.5 * CppAD::pow(traj_ref(3, 0) - vars[v_start], 2);

    // The rest of the constraints
    for (int i = 0; i < Constants::T - 1; i++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + i + 1];
      AD<double> y1 = vars[y_start + i + 1];
      AD<double> yaw1 = vars[yaw_start + i + 1];
      AD<double> v1 = vars[v_start + i + 1];

      // The state at time t.
      AD<double> x0 = vars[x_start + i];
      AD<double> y0 = vars[y_start + i];
      AD<double> yaw0 = vars[yaw_start + i];
      AD<double> v0 = vars[v_start + i];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + i];
      AD<double> a0 = vars[a_start + i];

      // constraint with the dynamic model
      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(yaw0) * Constants::DT);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(yaw0) * Constants::DT);
      fg[2 + yaw_start + i] = yaw1 - (yaw0 + v0 * CppAD::tan(delta0) / Constants::WB * Constants::DT);
      fg[2 + v_start + i] = v1 - (v0 + a0 * Constants::DT);
      // cost with the ref traj
      fg[0] += CppAD::pow(traj_ref(0, i+1) - (x0 + v0 * CppAD::cos(yaw0) * Constants::DT), 2);
      fg[0] += CppAD::pow(traj_ref(1, i+1) - (y0 + v0 * CppAD::sin(yaw0) * Constants::DT), 2);
      fg[0] += 0.5 * CppAD::pow(traj_ref(2, i+1) - (yaw0 + v0 * CppAD::tan(delta0) / Constants::WB * Constants::DT), 2);
      fg[0] += 0.5 * CppAD::pow(traj_ref(3, i+1) - (v0 + a0 * Constants::DT), 2);
    }
  }
};

Vec_f mpc_solve(State x0, Eigen_ref traj_ref){

/*
    x0: The initial state of the vehicle, containing its position (x, y), orientation (yaw), and velocity (v).
    traj_ref: A reference trajectory that the vehicle should follow.
*/


  // The CppAD API allows one to use any SimpleVector class.
  // The preprocessor symbol CPPAD_TESTVECTOR is template vector class which is used for correctness testing.

  typedef CPPAD_TESTVECTOR(double) Dvector; //  CPPAD_TESTVECTOR template class to pass information to CppAD (<cppad/cppad.hpp>)
  double x = x0.x;
  double y = x0.y;
  double yaw = x0.yaw;
  double v = x0.v;

  // Control inputs, delta and acceleration, thats why (T - 1) * 2
  // In each time step, over the prediction horizon (T) has four states (x, y, yaw, v), thats why we multiplies 4
  size_t n_vars = Constants::T * 4 + (Constants::T - 1) * 2; // T=6-->n_vars=34
  size_t n_constraints = Constants::T * 4; // n_constraints=24

  Dvector vars(n_vars); // KALDIM!!!
  for (int i = 0; i < n_vars; i++){
    vars[i] = 0.0;
  }

  vars[x_start] = x;
  vars[y_start] = y;
  vars[yaw_start] = yaw;
  vars[v_start] = v;

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  // NOTE there must be both lower and upper bounds for all vars!!!!!
  for (auto i = 0; i < n_vars; i++) {
    vars_lowerbound[i] = -10000000.0;
    vars_upperbound[i] = 10000000.0;
  }

  for (auto i = delta_start; i < delta_start+Constants::T-1; i++) {
    vars_lowerbound[i] = -Constants::MAX_STEER;
    vars_upperbound[i] = Constants::MAX_STEER;
  }

  for (auto i = a_start; i < a_start+Constants::T-1; i++) {
    vars_lowerbound[i] = -Constants::MAX_ACCEL;
    vars_upperbound[i] = Constants::MAX_ACCEL;
  }

  for (auto i = v_start; i < v_start+Constants::T; i++) {
    vars_lowerbound[i] = Constants::MIN_SPEED;
    vars_upperbound[i] = Constants::MAX_SPEED;
  }

  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (auto i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[yaw_start] = yaw;
  constraints_lowerbound[v_start] = v;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[yaw_start] = yaw;
  constraints_upperbound[v_start] = v;

  FG_EVAL fg_eval(traj_ref);

  // options
  std::string options;
  options += "Integer print_level  0\n";
  // options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Integer max_iter      50\n";
  // options += "Numeric tol          1e-6\n";
  options += "Numeric max_cpu_time          0.05\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_EVAL>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  Vec_f result; // Vec_f=std::vector<float>;
  for (auto i =0 ; i < n_vars; i++) {
    result.push_back((float)solution.x[i]);
    std::cout << i << " solution.x[i]: " << solution.x[i] << std::endl;
  }
  return result;
};

void mpc_simulation(Vec_f cx, Vec_f cy, Vec_f cyaw, Vec_f ck, Vec_f speed_profile, Poi_f goal){
  State state(cx[0], cy[0], cyaw[0], speed_profile[0]); // State struct initializing, cx, cy: interpolated positions

  /*

  This logic is adjusting the initial yaw angle of the state to be within the range of [-pi, +pi].
  If the initial yaw angle is greater than or equal to pi, it subtracts 2pi, until it falls within the range.
  If the initial yaw angle is less than or equal to -pi, it adds 2pi until it falls within the range

  */

  if ((state.yaw - cyaw[0]) >= M_PI) state.yaw -= M_PI * 2.0;
  else if ((state.yaw - cyaw[0]) <= -1.0*M_PI) state.yaw += M_PI * 2.0;

  float goal_dis = 0.5;
  int iter_count = 0;

  int target_ind = 0;
  //calc_nearest_index(state, cx, cy, cyaw, target_ind); // nearest point index from the current state to the interpolated points

  smooth_yaw(cyaw);

  // visualization
  cv::namedWindow("mpc", cv::WINDOW_NORMAL);
  cv::resizeWindow("mpc", 600, 600);
  int count = 0;

  Vec_f x_h;
  Vec_f y_h;

  Eigen_ref xref; // Eigen_ref=Eigen::Matrix<float, NX, T>

  std::cout << "Initialization : "<<std::endl;
  for (int i = 0; i < Constants::NX; ++i) {
    for (int j = 0; j < Constants::T; ++j) {
        std::cout << "xref(" << i << ", " << j << ") = " << xref(i, j) << "\t";
    }
    std::cout << std::endl;
}

  while (Constants::MAX_TIME >= iter_count){
  //while (390 >= iter_count){
    // Below function updates the columns of 'xref' with the future states, considering the discrete index shift 'discrete_distance_indx'
    calc_ref_trajectory(state, cx, cy, cyaw, ck, speed_profile, 1.0, target_ind, xref); // responsible for predicting the reference trajectory for the vehicle based on its current state and the available trajectory information.

    Vec_f output = mpc_solve(state, xref);

    // State dimension (x, y, yaw (yaw angle), and velocity (v)) = 4
    // Prediction Horizon = 6
    // xref --> 4 satır, 6 sütun
    std::cout << "After MPC solve(Reference trajectory updated): "<<std::endl;
    for (int i = 0; i < xref.rows(); ++i) {
    for (int j = 0; j < xref.cols(); ++j) {
        std::cout << "xref(" << i << ", " << j << ") = " << xref(i, j) << "\t";
    }
    std::cout << std::endl;
}


    update(state, output[a_start], output[delta_start]);


    std::cout << "After update : "<<std::endl;
    for (int i = 0; i < Constants::NX; ++i) {
    for (int j = 0; j < Constants::T; ++j) {
        std::cout << "xref(" << i << ", " << j << ") = " << xref(i, j) << "\t";
    }
    std::cout << std::endl;
    }


    float steer = output[delta_start];
    std::cout << "delta index: " <<delta_start << '\n';
    std::cout << "steer: " <<steer << '\n';

    float acceleration = output[a_start];
    std::cout << "a index: " <<a_start << '\n';
    std::cout << "acceleration: " <<acceleration  << '\n';



    float dx = state.x - goal[0];
    float dy = state.y - goal[1];
    if (std::sqrt(dx*dx + dy*dy) <= goal_dis) {
      std::cout<<("Goal")<<std::endl;
      break;
    }

    x_h.push_back(state.x);
    y_h.push_back(state.y);

    // visualization
    cv::Mat bg(2000, 3000, CV_8UC3, cv::Scalar(255, 255, 255));
    for(unsigned int i=1; i<cx.size(); i++){
      cv::line(
        bg,
        cv_offset(cx[i-1], cy[i-1], bg.cols, bg.rows),
        cv_offset(cx[i], cy[i], bg.cols, bg.rows),
        cv::Scalar(0, 0, 0),
        10);
    }

    // for(unsigned int j=0; j< T; j++){
    // 	cv::circle(
    // 		bg,
    // 		cv_offset(output[x_start+j], output[y_start+j], bg.cols, bg.rows),
    // 		10, cv::Scalar(0, 0, 255), -1);
    // }

    for(unsigned int k=0; k<x_h.size(); k++){
      cv::circle(
        bg,
        cv_offset(x_h[k], y_h[k], bg.cols, bg.rows),
        8, cv::Scalar(255, 0, 0), -1);
    }


    cv::line(
      bg,
      cv_offset(state.x, state.y, bg.cols, bg.rows),
      cv_offset(state.x + std::cos(state.yaw)*Constants::WB*2, state.y + std::sin(state.yaw)*Constants::WB*2, bg.cols, bg.rows),
      cv::Scalar(255,0,255),
      15);

    cv::line(
      bg,
      cv_offset(state.x + std::cos(state.yaw)*0.5,
                state.y + std::sin(state.yaw)*0.5,
                bg.cols, bg.rows),
      cv_offset(state.x - std::cos(state.yaw)*0.5,
                state.y - std::sin(state.yaw)*0.5,
                bg.cols, bg.rows),
      cv::Scalar(255,0,127),
      30);

    cv::line(
      bg,
      cv_offset(state.x + std::cos(state.yaw)*Constants::WB*2 + std::cos(state.yaw+steer)*0.5,
                state.y + std::sin(state.yaw)*Constants::WB*2 + std::sin(state.yaw+steer)*0.5,
                bg.cols, bg.rows),
      cv_offset(state.x + std::cos(state.yaw)*Constants::WB*2 - std::cos(state.yaw+steer)*0.5,
                state.y + std::sin(state.yaw)*Constants::WB*2 - std::sin(state.yaw+steer)*0.5,
                bg.cols, bg.rows),
      cv::Scalar(255,0,127),
      30);

    for (int i = 0; i < Constants::NX; ++i) {
    for (int j = 0; j < Constants::T; ++j) {
        std::cout << "xref(" << i << ", " << j << ") = " << xref(i, j) << "\t";
    }
    std::cout << std::endl;
    }

    for(unsigned int k=0; k<xref.cols(); k++){
      cv::drawMarker(
        bg,
        cv_offset(xref(0, k), xref(1, k), bg.cols, bg.rows),
        cv::Scalar(0, 255, 255),
        cv::MARKER_CROSS,
        20, 3);
    }  // xref(0, k) -->  x value in k. time in prediction horizon, xref(1, k) --> y value in k. time in prediction horizon

    // save image in build/bin/pngs
    // struct timeval tp;
    // gettimeofday(&tp, NULL);
    // long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
    // std::string int_count = std::to_string(ms);
    // cv::imwrite("./pngs/"+int_count+".png", bg);

    cv::imshow("mpc", bg);
    cv::waitKey(5);
    iter_count++;
  }
};

int main(){

  Vec_f wx{0.0, 60.0, 125.0,  50.0,   75.0,  35.0,  -10.0}; //  vector wx that represents the x-coordinates of waypoints on a trajectory.
  Vec_f wy({0.0,  4.0,  -4.0,  4.0,  -4.0,   4.0,  0.0});
  //Vec_f wy{0.0,  0.0,  50.0,  65.0,   30.0,  50.0,  -20.0}; //  vector wy that represents the y-coordinates of waypoints on a trajectory.

  /*
  // range-based for-loop (1D array)
  for (auto& e: wy)
        std::cout << e << std::endl;
  */

  /*
  csp_obj.sx: Spline class--> Vec_f x=s,   Vec_f y=wx
  csp_obj.sy: Spline class  --> Vec_f x=s  Vec_f y=wy
  csp_obj.s:  Vec_f  vector:  //  Calculate cumulative arc lengths

  Spline:

  Vec_f x;  // Calculate cumulative arc lengths
  Vec_f y;  // Given vector x or y
  int nx;  // length of vector x
  Vec_f h;  // vector difference between n and n-1
  Vec_f a;  // cubic spline function coef
  Vec_f b;  // cubic spline function coef
  Vec_f c;  // cubic spline function coef
  Vec_f d;  // cubic spline function coef
  */

  Spline2D csp_obj(wx, wy); // cubic_spline.h

  /*
  int i{0};
  for (auto& e: csp_obj.s) {
        std::cout << "Cumulative Arc Length from point "<< "0" << " to point " << (i+1) << ": " << e << std::endl; // cumulative arc lengths represent the total distance traveled along a trajectory from the starting point to each specific point.
        ++i;
        }
  */


  // Vec_f=std::vector<float>
  Vec_f r_x{}; // interpolated x-coordinate
  Vec_f r_y{}; //  interpolated y-coordinate
  Vec_f ryaw{}; // yaw (heading angle)
  Vec_f rcurvature{}; // curvature
  Vec_f rs{}; // given point
  //for(float i=0; i<csp_obj.s.back(); i+=1.0){
  for(std::size_t i{0}; i<csp_obj.s.back(); ++i) { // Iterates through the calculated arc lengths (csp_obj.s) of the entire trajectory using index i, csp_obj.s.back()=389.44
    std::array<float, 2> point_ = csp_obj.calc_position(static_cast<float>(i)); // evaluate the interpolated value at a specific position t along s (Vec_f s)
    r_x.push_back(point_[0]); // interpolated position x
    r_y.push_back(point_[1]); // interpolated position y
    float CalculatedYaw=csp_obj.calc_yaw(static_cast<float>(i)); // CalculatedYaw is an angle, and it is typically expressed in radians.
    ryaw.push_back(CalculatedYaw); // yaw (heading angle)
    // Calculates the curvature of a trajectory at a specific position i along the path. Curvature is a measure of how much a curve deviates from being a straight line.
    float CalculatedCurvature=csp_obj.calc_curvature(static_cast<float>(i)); //
    rcurvature.push_back(CalculatedCurvature);
    rs.push_back(static_cast<float>(i));
  }

  //std::cout<< "r_x back last element in vector: " << r_x.back() << "r_x front first element in vector: " << r_x.front() << std::endl;

  float target_speed = 10.0 / 3.6;
  Vec_f speed_profile = calc_speed_profile(r_x, r_y, ryaw, target_speed);

  mpc_simulation(r_x, r_y, ryaw, rcurvature, speed_profile, {{wx.back(), wy.back()}});
}
