#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp, double Ki, double Kd, unsigned int calc_tot_err_after, unsigned int tune_coeffs_each,
            double twiddle_dKp_initial, double twiddle_dKi_initial, double twiddle_dKd_initial,
            double twiddle_tolerance);
  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

 private:
  /*
  * Helper variables
  */
  unsigned int _calc_tot_err_after;
  unsigned int _tune_coeffs_each;
  unsigned long long _step_cnt;

  /*
  * Helper variables for TWIDDLE algorithm
  */
  double _dKp;
  double _dKi;
  double _dKd;
  double _tol;
  double _best_err;
  double _total_err;
  int _coeff_ind;
  int _attempts_per_ind;

  /*
  * Tune parameters in accordance with the "TWIDDLE" algorithm for parameter tuning.
  */
  void TuneCoeffsUsingTwiddleAlg();

  double &IndToCoeff(int ind);
  double &IndToDeltaCoeff(int ind);
};

#endif  // PID_H
