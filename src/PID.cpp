#include "PID.hpp"
#include <cmath>
#include <limits>
using namespace std;

/*
 * PID class.
 */

PID::PID() { Reset(); }

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte) {
  if (steps_ == 1) {
    p_error = cte;
  }

  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  squared_error_sum_ += pow(cte, 2);
  error_ = squared_error_sum_ / steps_++;
}

double PID::TotalError() { return -Kp * p_error - Kd * d_error - Ki * i_error; }

void PID::Reset() {
  steps_ = 1;
  squared_error_sum_ = 0.0;
  error_ = std::numeric_limits<double>::max();
  p_error = d_error = i_error = 0.0;
}
