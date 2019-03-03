#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
//using std::cout;
//using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  //Lets create the RSME vector
  VectorXd RSME = VectorXd(4);
  RSME << 0,0,0,0;
  
  //Now the ground truth and the estimation vector should both have the same number of elements in them
  //for the computation to proceed if that is not the case then really what are you even doing????
  //Like seriously bruh???
  //PLEASE DONT SUE ME FOR THAT BAD JOKE, I apologize
  if ((estimations.size()!=ground_truth.size())||(estimations.size()==0))
  {
    std::cout << "Estimation and ground truth vector sizes DON'T match or estimation vector size is ZERO" << std::endl;
    return RSME; //terminates the function
  }
  
  //Now to implement the RSME 
  for (unsigned int i=0; i < estimations.size(); ++i) 
  {
    VectorXd residual = estimations[i] - ground_truth[i];
  	residual = residual.array()*residual.array();
  	RSME += residual;
  }

  // calculate the mean
  RSME = RSME/estimations.size();

  // calculate the squared root
  RSME = RSME.array().sqrt();

  // return the result
  return RSME;
}

//I just realized that I have written RSME instead of RMSE

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  
  //I picked this code directly from the lesson itself
  
  MatrixXd Hj(3,4);
  // recover state parameters
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  double c1 = px*px+py*py;
  double c2 = sqrt(c1);
  double c3 = (c1*c2);

  // check division by zero
  //const double c1 = std::max(eps, px*px + py*py)
  // I am not doing this as to keep consistency with the lesson
  if (fabs(c1) < 0.0001) {
    std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
    return Hj;
  }
   // compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}
