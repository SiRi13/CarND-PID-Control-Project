#include "PID.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

using namespace std;

/*
 * PID class.
 */

PID::PID() {
  steps_ = 1;
  squared_error_sum_ = 0.0;
  best_error_ = std::numeric_limits<double>::max();
  p_error = d_error = i_error = 0.0;
  p_ = {0.0, 0.0, 0.0};
  dp_ = {0.05, 0.05, 0};
  tolerance_ = 1.0e-2;
  improved_ = false;
  index_ = 0;
}

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

  squared_error_sum_ += std::pow(cte, 2);
  ++steps_;
}

double PID::TotalError() { return -Kp * p_error - Kd * d_error - Ki * i_error; }

double PID::Error() { return squared_error_sum_ / steps_; }

void PID::Twiddle() {
  std::cout << "Twiddle" << std::endl;
  double err = Error();
  if (err < best_error_) {
    std::cout << "improved! " << err << " < " << best_error_ << std::endl;
    best_error_ = err;
    dp_[index_] *= 1.1;
    NextIndex();
    p_[index_] += dp_[index_];
  } else {
    if (!improved_) {
      std::cout << "not improved!" << std::endl;
      // reset and twiddle in opposite direction
      p_[index_] -= 2 * dp_[index_];
      improved_ = true;
    } else {
      std::cout << "not improved again!" << std::endl;
      improved_ = false;
      // reset previous twiddle
      p_[index_] += dp_[index_];
      dp_[index_] *= 0.9;
      NextIndex();
      p_[index_] += dp_[index_];
    }
  }

  this->Kp = p_[0];
  this->Kd = p_[1];
  // this->Ki = p_[2];

  steps_ = 1;
  squared_error_sum_ = 0;
}

void PID::Reset() {
  ResetErrors();
  improved_ = false;
  steps_ = 1;
  index_ = 0;
}

void PID::ResetErrors() {
  p_error = i_error = d_error = 0.0;
  squared_error_sum_ = 0.0;
}

void PID::NextIndex() {
  index_ = ++index_ % 2;
  improved_ = false;
}
