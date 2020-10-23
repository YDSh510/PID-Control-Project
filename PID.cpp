#include "PID.h"
#include <limits>
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
  p_error = 0;
  i_error = 0;
  d_error = 0;
  
  prev_cte = 0.0;
  step_cnt = 0.0;
  total_err = 0.0;
  best_err = std::numeric_limits<double>::max();

}

void PID::UpdateError(double cte) {

        p_error = cte;
  
        d_error = cte - prev_cte;
        prev_cte = cte;
  
        i_error += cte;
  
  ++step_cnt;
  
  if (step_cnt >= 30) {
    total_err += cte * cte;
  }
  
  //Twiddle();

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  
  double corr = p_error * Kp + i_error * Ki + d_error * Kd;
  
            if (corr > 1) {
            corr = 1;
          }
          else if (corr < -1) {
            corr = -1;
          }
  
  return corr;  // TODO: Add your total error calc here!
}


void PID::Twiddle(double tol){
    double p[3] = {Kp, Ki, Kd };
    double dp[3] = {1,1,1};
  std::cout << "hi1" << std::endl;

    //double best_err = total_err;
    double sum = dp[0] + dp[1] + dp[2];
    while (sum > tol){
      printf("hi2");
        for(int i=0;i<3;i++){
          printf("hi3");
            p[i] += dp[i];
            //double err = TotalError();
          
            if (total_err < best_err)
            {
                best_err = total_err;
                dp[i] *= 1.1; 
            }
            else
            {
                p[i] -= 2 * dp[i];
                //err = TotalError();

                if (total_err < best_err)
                {
                    best_err = total_err;
                    dp[i] *= 1.1;
                    //print("hi",dp[i])
                }
                else
                {
                    p[i] += dp[i];
                    dp[i] *= 0.9;
                  }
            }
          
      }
      printf("hiafter3");
    }
  printf("hiafter2");
}
