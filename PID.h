#ifndef PID_H
#define PID_H

class PID {
 public:
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
  void Init(double Kp_, double Ki_, double Kd_);

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
  
  /**
   * Calculate the Ki,Kp and Kd parameter.
   * @output best error.
   */
  void Twiddle(double tol=0.2);

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  
  double prev_cte;
  double step_cnt;
  double total_err;
  double best_err;
  /**
  
  
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
};

#endif  // PID_H