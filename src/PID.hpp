#ifndef PID_H
#define PID_H

#include <vector>

class PID {
 public:
  /*
   * Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /*
   * Coefficients
   */
  double Kp;
  double Ki;
  double Kd;

  /*
   * Tuning
   */
  unsigned steps_;
  unsigned index_;
  double squared_error_sum_;
  double best_error_;
  double tolerance_;

  std::vector<double> dp_;
  std::vector<double> p_;

  bool improved_;

  /*
   * Constructor
   */
  PID();

  /*
   * Destructor.
   */
  virtual ~PID();

  /*
   * Initialize PID.
   */
  void Init(double Kp, double Ki, double Kd);

  /*
   * Update the PID error variables given cross track error.
   */
  void UpdateError(double cte);

  /*
   * Calculate the total PID error.
   */
  double TotalError();

  void Twiddle();

  void Reset();

  void ResetErrors();

  void NextIndex();

  double Error();
};

#endif /* PID_H */
