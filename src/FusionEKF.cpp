#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  Hj_ << 1, 1, 0, 0,
  		 1, 1, 0, 0,
  		 1, 1, 1, 1;    //OCD kicking in XD XD XD 
  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  // Let us first initialize the state vector
	ekf_.P_ = MatrixXd(4,4);
	ekf_.P_  <<1, 0, 0, 0,
			   0, 1, 0, 0,
			   0, 0, 1000, 0,
			   0, 0, 0, 1000;
//taking high values for position in x and y so that we can work towards the actual values though any values can be chosen
//provided it is sufficiently large
  	H_laser_ << 1, 0, 0, 0,
  				0, 1, 0, 0;
  
    ekf_.F_ = MatrixXd(4,4);
    ekf_.F_ <<	1, 0, 1, 0,
    			0, 1, 0, 1,
    			0, 0, 1, 0,
    			0, 0, 0, 1;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      cout << "Using the measurements from the RADAR" << endl;
      //Reading the data from the measurement pack
      double rho = measurement_pack.raw_measurements_[0];
      double phi = measurement_pack.raw_measurements_[1];
      double rho_dot = measurement_pack.raw_measurements_[2];
      
      //Converting the coordinates from polar to cartesian
      double x = rho*cos(phi);
      double y = rho*sin(phi);
      //Also to find the initial velocities in x and y directions
      double vx = rho_dot*cos(phi);
      double vy = rho_dot*sin(phi);
      //Now lets initailize the initial measurement matrix for measurements from the RADAR
      ekf_.x_<<x,y,vx,vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      cout << "Using the measurements from the LIDAR" << endl;
      
      //Lidar measurements are already in cartesian coordinates so no conversion is required
      double a=measurement_pack.raw_measurements_[0];
      double b=measurement_pack.raw_measurements_[1];
    
      //Lets initialze the initial measurement matrix for measurements from the LIDAR 
      ekf_.x_ << a,b,0,0;
	  //velocity quantities are zero since LIDAR cannot measure velocity directly 
    }

    // done initializing, no need to predict or update
    previous_timestamp_=measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }
  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
    //creating the timestamp

  
    double dt = (measurement_pack.timestamp_-previous_timestamp_)/1000000.0;
    previous_timestamp_=measurement_pack.timestamp_;

    //Lets define the state transition matrix F now,

    ekf_.F_(0,2)=dt;
  	ekf_.F_(1,3)=dt;
  	//this gives us the required matrix F<< 1, 0, dt, 0,
  	//										0, 1, 0, dt,
  	//										0, 0, 1, 0,
  	//										0, 0, 0, 0
  
    // Now to define the noise covariance matrix Q (its really tedious defining this matrix, its just such a long process)
    // *insert nervous laugh emoji here (three of them)*
    // These noises are produced for a number of reaso such as movement of the vehicle, dust, weather conditions etc.
    // So to in order to deal with all these factors noise factors are intorduced
    
    //These were provided before hand 
    const double noise_ax=9.0;
    const double noise_ay=9.0;
    
    const double dt_2=dt*dt;
    const double dt_3=dt*dt*dt;
    const double dt_4=dt*dt*dt*dt;
    
    //Lets now define the matrix
    
    ekf_.Q_ = MatrixXd(4,4);
    ekf_.Q_ << 	dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
    			0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
    			dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
    			0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
    
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */
	//Now for the easy part, you see this is how codes should be nice and small, not requiring to write long matrices
    //I see you there noise transition matrix
    //Jokes aside now we have two types of inputs as we know, one from the RADAR and LIDAR
    //Now mearsurement update can be done directly for the LIDAR using simple linera equations
    //But is significantly different with RADAR because of how RADAR actually fuctions
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    //Update for RADAR

  } else {
    // TODO: Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
	//Update for LIDAR data
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
