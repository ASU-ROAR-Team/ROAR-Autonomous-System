#include "ROAR_UKF.h"
#include "WGS84toCartesian.hpp"
#include <iostream>
#include <limits>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std;
ROVER::ROVER()
{
    /***
    ROVER Default Constructor
    ***/
    // Default Constructor with default parameters
    Kinematic_model_parameters = Eigen::MatrixXd::Zero(2, 2);
    rover_speeds = Eigen::VectorXd::Zero(2);
    
    Kinematic_model_parameters << 0.06225,  0.06225,
                                  -0.1507, 0.1507;

}
void ROVER::calculate_wheel_change(Eigen::VectorXd w, double dt)
{
    /****
      Process wheel speeds to calculate rover linear and angular velocity, position and orientation
        Inputs:
        x: current sigma point of state estimate x = [q0 q1 q2 q3 omega_x omega_y omega_z x y].T
        w: Wheel speeds (w1, w2).T
        dt: delta time

        Outputs:
        velocity: Linear velocity of rover
        omega: Angular velocity of rover
    ***/
    rover_speeds = Kinematic_model_parameters * w;
}
ROVER::~ROVER()
{
    // Destructor
}
/*** ------ Sigma points --------- ***/
MerwedSigmaPoints::MerwedSigmaPoints()
{
    /***
    Merwe Sigma Point Default Constructor
    ***/    
    // Default Constructor with default parameters
    this->n = n;
    this->num_sigma_points = int(2*n + 1);

    // Default sigma points params
    this->alpha = 3.0;
    this->beta = 2.0;
    this->kappa = 0.1;

    // Compute the weight matrices
    this->Wm = compute_Wm();
    this->Wc = compute_Wc();
}
MerwedSigmaPoints::MerwedSigmaPoints(int n, double alpha, double beta, double kappa)
{
    /***
    Merwe Sigma Point Generalized Constructor

    Inputs:
    n: state dimension of state estimate
    alpha: alpha param for Merwe Sigma point
    beta: beta param of Merwe Sigma Point
    kappa: kappa param of Merwe Sigma Point
    ***/
    // Num sigma points
    this->n = n;
    this->num_sigma_points = int(2 * n + 1);

    // Default sigma points params
    this->alpha = alpha;
    this->beta = beta;
    this->kappa = kappa;

    // Compute the weight matrices
    this->Wm = compute_Wm();
    this->Wc = compute_Wc();
}
MerwedSigmaPoints::~MerwedSigmaPoints()
{
    // Destructor
}
/*** Weight computation ***/
Eigen::VectorXd MerwedSigmaPoints::compute_Wm()
{
    /***
    Calculates The Weighted Mean for Merwe Sigma Points

    Outputs:
    Wm: (2n+1)x(n) Weight Mean
    ***/

    // Compute lambda
    double lambda_ = alpha * alpha * (n + kappa) - n;

    // Initialize Wm weight array 
    // BLA::Matrix<2*n + 1> Wm;
    Wm = Eigen::VectorXd(num_sigma_points);

    // Compute initial weight
    Wm(0) = lambda_ / (n + lambda_);

    // Compute the rest of the weight
    for (int i = 1; i < 2 * n + 1; i++)
    {
        Wm(i) = 1.0 / (2 * (n + lambda_));
    }

    return Wm;
}
Eigen::VectorXd MerwedSigmaPoints::compute_Wc()
{
    /***
    Calculates The Weighted Covariance for Merwe Sigma Points

    Outputs:
    Wc: (2n+1)x(n) Weight Covariance
    ***/

    // Compute lambda
    double lambda_ = alpha * alpha * (n + kappa) - n;

    // Initialize Wm weight array 
    // BLA::Matrix<2*n + 1> Wc;
    Wc = Eigen::VectorXd(num_sigma_points);

    // Compute initial weight
    Wc(0) = (lambda_ / (n + lambda_)) + 1 - alpha * alpha + beta;

    // Compute the rest of the weight
    for (int i = 1; i < 2 * n + 1; i++)
    {
        Wc(i) = 1.0 / (2 * (n + lambda_));
    }

    return Wc;

}
/*** Sigma point calculation ***/
Eigen::MatrixXd MerwedSigmaPoints::calculate_sigma_points(Eigen::VectorXd mean, Eigen::MatrixXd cov)
{
    /***
    Calculates Merwe Sigma Points
    Inputs:
    mean "X": nx1 matrix of state mean
    cov "P": nxn covariance matrix

    Outputs:
    sigma_points: (2n+1) Merwe Sigma Points
    ***/
    // Init sigma point array
    Eigen::MatrixXd sigma_points = Eigen::MatrixXd::Zero(n, num_sigma_points); // Sigma points Matrix is not transposed

    // Square root of (n + lambda) * cov
    double lambda_ = alpha * alpha * (n + kappa) - n;
    Eigen::MatrixXd n_plus_lambda_times_cov = (n + lambda_) * cov;

    Eigen::LLT<Eigen::MatrixXd> lltOfA(n_plus_lambda_times_cov);    // compute the Cholesky decomposition of A
    Eigen::MatrixXd U = lltOfA.matrixU();                           // retrieve factor U  in the decomposition (upper)

    // Calculate sigma points
    sigma_points.col(0) = mean; // First column corresponds to the mean
    for (int i = 0; i < n; i++)
    {
        sigma_points.col(i + 1) = mean + U.col(i); // Positive sigma points
        sigma_points.col(i + n + 1) = mean - U.col(i); // Negative sigma points
    }
    return sigma_points;
}
UKF::UKF(MerwedSigmaPoints merwed_sigma_points)
{
    /***
    Unscented Kalman Filter Constructor with Merwe Sigma Points

    This is a default constructor for the Unscented Kalman Filter for Orientation Estimation
    using Quaternions. The constructor takes in Merwe sigma points and assigns it to the UKF.

    The state space is:

    x = [q0 q1 q2 q3 omega_x, omega_y, omega_z x y].T

    where (q0,q1,q2,q3) are the quaternion elements of the following quaternion
    q0 + q1*i + q2*j + q3*k. (omega_x, omega_y, omega_z) is the angular velocity. (x,y) 
    is the position

    The z measurement state space is:

    z = [z_gyro, z_acc, z_mag, z_gps].T

    where z_gyro is the measurement from the gyro, z_acc is the measurement from the accelerometer
    z_mag is the measurement from the magnetometer. Note that these measurements are in the
    body frame. z_gps is the measurement from the GPS (latitude, longitude).
    ***/
    // Initialize x state vector
    x_dim = 9;
    x_hat = Eigen::VectorXd::Zero(x_dim);
    x_hat << 1, 0, 0, 0, 0, 0, 0, 0, 0;   // Initial Quaterion: 1+0i+0j+0k and initial ang vel: [0 0 0].T

    // Initialize z state vector
    z_dim = 14; //////////////////////////////////Hassan 11 -> 14
    z = Eigen::VectorXd::Zero(z_dim); // Initial measurement in frame {B}, [z_gyro, z_acc, z_mag].T

    // Intialize Posteriori Estimate Covariance Matrix
    //P = Eigen::MatrixXd::Zero(x_dim, x_dim);
    P = Eigen::MatrixXd::Identity(x_dim, x_dim)*0.5;
    S = Eigen::MatrixXd::Zero(z_dim, z_dim);

    // Initialize Prior Estimates
    x_prior = x_hat;
    P_prior = P;

    // Initial Posterior Estimates
    x_post = x_hat;
    P_post = P;

    // Compute mean and covariance using unscented transform
    z_prior = z;
    S_prior = S;

    // Assign the sigma points into UKF class
    sigma_points = merwed_sigma_points;

    X_sigma = Eigen::MatrixXd::Zero(x_dim, sigma_points.num_sigma_points); // Predicted sigma points
    Z_sigma = Eigen::MatrixXd::Zero(z_dim, sigma_points.num_sigma_points); // Measurement sigma points

    // Initialize noise matrices
    Q = Eigen::MatrixXd::Identity(x_dim, x_dim) * 1e-10;    // Process Noise Matrix //research

    R = Eigen::MatrixXd::Identity(z_dim, z_dim) * 0.7;      // Measurement Noise Matrix //add noise covariance for each sensor from datasheet

    // Intialize inertial frame quantities
    g0 << 0, 0, 1;                          // Gravitational Acceleration Vector
    m0 << B_INTENSITY * cos(INCLINATION),   // Magnetic Field Intensity Vector
        0.0,
        B_INTENSITY* sin(INCLINATION);

    yaw = 0.0;

}
/*** Destructor ***/
UKF::~UKF()
{
    // Destructor
}
// --- Unscented Transform ---
std::tuple<Eigen::VectorXd, Eigen::MatrixXd> UKF::unscented_transform(Eigen::MatrixXd sigmas,
    Eigen::MatrixXd Wm,
    Eigen::MatrixXd Wc,
    Eigen::MatrixXd noise_cov)
{
    /***
    Computes the unscented transform from the sigma points, weighted means and weighted noise covariance.

    Inputs:
    sigmas: Sigma points of UKF
    Wm: Weighted Mean
    Wc: Weighted Covariance
    noise_cov: Noise covariance matrix

    Outputs:
    mu: Mean from unscented transform computation
    P_cov: Covariance from unscented transform computation
    ***/
    // Compute new mean
    Eigen::VectorXd mu = sigmas * Wm;   // Vectorization of sum(wm_i* sigma_i)

    // Compute new covariance matrix
    int kmax = sigmas.cols(); // Number of columns instead of rows
    int n = sigmas.rows();    // Number of rows
    Eigen::MatrixXd P_cov = Eigen::MatrixXd::Zero(n, n);

    for (int k = 0; k < kmax; k++)
    {
        Eigen::VectorXd y = sigmas.col(k) - mu;
        P_cov += Wc(k) * y * y.transpose();
    }

    // Add noise
    P_cov += noise_cov;

    return std::make_tuple(mu, P_cov);
}
void UKF::predict_states(Eigen::VectorXd w, double dt) // what is dt
{
    /***
    Predict with wheel odometry process model
    u_t: Measured wheels velocity as input
    ***/
    // Compute the sigma points for given mean and posteriori covariance
    Eigen::MatrixXd sigmas = sigma_points.calculate_sigma_points(x_hat, P);

    // Pass sigmas into f(x) for wheel odometry
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        // Predict with wheel odometry process model for sigma points
        X_sigma.col(i) = process_model(sigmas.col(i), w, dt);
    }

    // Compute unscented mean and covariance
    std::tie(x_hat, P) = unscented_transform(X_sigma,
        sigma_points.Wm,
        sigma_points.Wc,
        Q);

    // Save prior
    x_prior = x_hat.replicate(1, 1);
    P_prior = P.replicate(1, 1);

}
// --- Process & Measurement Model with Wheel Odometry ---
// Process Model
Eigen::VectorXd UKF::process_model(Eigen::VectorXd x, Eigen::VectorXd w, double dt)
{
    /***
    Nonlinear process model for Wheel Odometry

    Inputs:
    x: current sigma point of state estimate x = [q0 q1 q2 q3 omega_x omega_y omega_z x y].T
    u_t: Current input to nonlinear process model
    dt: delta time

    ***/

    // Process wheel speeds using Kinematic Model
    ROVER rover;
    rover.calculate_wheel_change(w, dt);

    // considern changing this quaternion into UnitQuaternion
    Quaternion attitude(x(0),
        x(1),
        x(2),
        x(3));

    // Estimated attitude update with incremental rotation update
    // EQN 3.26 & EQN 3.17 (Exponential with skew matrix and delta_t)
    //consider adding noise to the angular velocity and orientation
    UnitQuaternion uq_omega = UnitQuaternion::omega(x(4) * dt,
        x(5) * dt,
        x(6) * dt);
    attitude = attitude * uq_omega;

    Eigen::VectorXd x_pred_sigma(9);

    // Quaternions
    x_pred_sigma(0) = attitude.s;
    x_pred_sigma(1) = attitude.v_1;
    x_pred_sigma(2) = attitude.v_2;
    x_pred_sigma(3) = attitude.v_3;

    // Angular velocity
    x_pred_sigma(4) = x(4);
    x_pred_sigma(5) = x(5);
    x_pred_sigma(6) = x(6);

    //position
    //float yaw = atan2(2 * (x(0) * x(3) + x(1) * x(2)), (1 - 2 * (x(2) * x(2) + x(3) * x(3))));
    double roll, pitch, yaw;
    tf2::Quaternion quat (x(0), x(1), x(2), x(3));
    tf2::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);

    // Update position based on linear and angular velocities
    // cout << "yaw: " << yaw << endl;

    // Update x and y positions
    double linear_velocity = round(rover.rover_speeds(0)*100)/100;
    double angular_velocity = round(rover.rover_speeds(1)*100)/100;

    // Calculate change in x and y positions
    double dx = linear_velocity * cos(yaw) * dt;
    double dy = linear_velocity * sin(yaw) * dt;

    // Update x and y positions
    x_pred_sigma(7) = x(7) + dx;
    x_pred_sigma(8) = x(8) + dy;
    return x_pred_sigma;
}
void UKF::predict_measurement(double dt, Eigen::VectorXd w, double lon0, double lat0, double X0, double Y0, double Z0)
{
    /***
    Update step of UKF with Quaternion + Angular Velocity model i.e state space is:

    x = [q0 q1 q2 q3 omega_x omega_y omega_z].T
    z = [z_gyro z_acc z_mag].T

    Inputs:
    z_measurement: Sensor measurements from gyroscope, accelerometer and magnetometer
    ***/
   
    // Pass the transformed sigmas into measurement function
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        // Update sigmas with measurement model
        Z_sigma.col(i) = UKF::measurment_model(X_sigma.col(i) , w, lon0, lat0, X0, Y0, Z0, dt);
    }

    std::tie(z_prior, S) = unscented_transform(Z_sigma,
        sigma_points.Wm,
        sigma_points.Wc,
        R);
}
// Measurement Model
Eigen::VectorXd UKF::measurment_model(Eigen::VectorXd x, Eigen::VectorXd w, double lon0, double lat0, double X0, double Y0, double Z0, double dt)
{
    /***
    Nonlinear measurement model for Orientation estimation with Quaternions

    Inputs:
    x: current sigma point of state estimate x = [q0 q1 q2 q3 omega_x omega_y omega_z].T

    Outputs:
    z_pred_sigma: sigma point after being propagated through nonlinear measurement model
    ***/
    // --- Measurement model ---
    // Extract quaternion from current state estimates 
    UnitQuaternion attitude(x(0),
        x(1),
        x(2),
        x(3));

    // Inverse: {B} to {0}
    UnitQuaternion invq = attitude.inverse();

    // Accelerometer
    Eigen::VectorXd acc_pred = invq.vector_rotation_by_quaternion(g0);

    // Magnetomer
    Eigen::VectorXd mag_pred = invq.vector_rotation_by_quaternion(m0);

    // Gyroscope
    Eigen::VectorXd gyro_pred(3);
    gyro_pred << x(4), x(5), x(6);

    // float yaw = atan2(2 * (x(0) * x(3) + x(1) * x(2)), 1 - 2 * (x(2) * x(2) + x(3) * x(3)));
    // yaw += round(rover.rover_speeds(1) * dt*100)/100; // Update orientation


    ROVER rover;
    rover.calculate_wheel_change(w, dt);
    
    // Update position based on linear and angular velocities
    yaw += rover.rover_speeds(1) * dt; // Update orientation

    // Update x and y positions
    double linear_velocity = rover.rover_speeds(0);
    double angular_velocity = rover.rover_speeds(1);

    // Calculate change in x and y positions
    double dx = linear_velocity * cos(yaw) * dt;
    double dy = linear_velocity * sin(yaw) * dt;
    double lat = lat0 + (180 / PI) * (dy / 6378137);
    double lon = lon0 + (180 / PI) * (dx / 6378137) / cos(lat0);

    // Calculate change in x and y positions for LL
    double dxLandmark = X0 - x(7);
    double dyLandmark = Y0 - x(8);
    double X = X0 + dxLandmark;
    double Y = Y0 + dyLandmark;
    double Z = Z0;

    // Z prediction
    Eigen::VectorXd z_pred_sigma(14);
    z_pred_sigma << gyro_pred, acc_pred, mag_pred, lat, lon, X, Y, Z;

    return z_pred_sigma;

}
void UKF::update(Eigen::MatrixXd z_measurement)
{
	/***
    	Update step of UKF with Quaternion + Angular Velocity model i.e state space is:
        
        	x = [q0 q1 q2 q3 omega_x omega_y omega_z].T
            	z = [z_gyro z_acc z_mag].T
                
                	Inputs:
                    	z_measurement: Sensor measurements from gyroscope, accelerometer and magnetometer
                        	***/

	// Compute cross covariance
	Eigen::MatrixXd T = Eigen::MatrixXd::Zero(x_dim, z_dim);
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
		T = T + sigma_points.Wc(i) * (X_sigma.col(i) - x_hat) * (Z_sigma.col(i) - z_prior).transpose();
	}

	// Compute Kalman gain
	Eigen::MatrixXd K = T * S.inverse();

	// Update state estimate
	x_hat = x_hat + K * (z_measurement - z_prior);

	// Update covariance
	P = P - K * S * K.transpose();

	// Save posterior
	x_post = x_hat.replicate(1, 1);
	P_post = P.replicate(1, 1); 

}
void UKF::encoder_callback(Eigen::VectorXd w, double dt, double yaw)
{
    /***
    Predict with wheel odometry process model
    u_t: Measured wheels velocity as input
    ***/
    // Compute the sigma points for given mean and posteriori covariance
    Eigen::MatrixXd sigmas = sigma_points.calculate_sigma_points(x_post, P_post);

    // Pass sigmas into f(x) for wheel odometry
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        X_sigma.col(i)(0) = sigmas.col(i)(0);
        X_sigma.col(i)(1) = sigmas.col(i)(1);
        X_sigma.col(i)(2) = sigmas.col(i)(2);
        X_sigma.col(i)(3) = sigmas.col(i)(3);
        X_sigma.col(i)(4) = sigmas.col(i)(4);
        X_sigma.col(i)(5) = sigmas.col(i)(5);
        X_sigma.col(i)(6) = sigmas.col(i)(6);

        // Process wheel speeds using Kinematic Model
        ROVER rover;    
        rover.calculate_wheel_change(w, dt);

        //position
        // Update x and y positions
        double linear_velocity = rover.rover_speeds(0);

        // Calculate change in x and y positions
        double dx = linear_velocity * cos(yaw) * dt;
        double dy = linear_velocity * sin(yaw) * dt;

        // Update x and y positions
        X_sigma.col(i)(7) = sigmas.col(i)(7) + dx;
        X_sigma.col(i)(8) = sigmas.col(i)(8) + dy;
    }

    // Compute unscented mean and covariance
    std::tie(x_hat, P) = unscented_transform(X_sigma,
        sigma_points.Wm,
        sigma_points.Wc,
        Q);

    // // Save prior
    x_post.tail(2) = x_hat.tail(2);
    P_post.col(7) = P.col(7);
    P_post.col(8) = P.col(8);
    P_post.row(7) = P.row(7);
    P_post.row(8) = P.row(8);
}
void UKF::imu_callback(Eigen::VectorXd w, Eigen::VectorXd z_measurement, double dt)
{
        /***
    Predict with wheel odometry process model
    u_t: Measured wheels velocity as input
    ***/
    // Compute the sigma points for given mean and posteriori covariance
    Eigen::MatrixXd sigmas = sigma_points.calculate_sigma_points(x_post, P_post);

        // Pass sigmas into f(x) for wheel odometry
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        X_sigma.col(i)(4) = sigmas.col(i)(4);
        X_sigma.col(i)(5) = sigmas.col(i)(5);
        X_sigma.col(i)(6) = sigmas.col(i)(6);

        // considern changing this quaternion into UnitQuaternion
        UnitQuaternion attitude(sigmas.col(i)(0),
        sigmas.col(i)(1),
        sigmas.col(i)(2),
        sigmas.col(i)(3));

        // Estimated attitude update with incremental rotation update
        // EQN 3.26 & EQN 3.17 (Exponential with skew matrix and delta_t)
        //consider adding noise to the angular velocity and orientation
        UnitQuaternion uq_omega = UnitQuaternion::omega(sigmas.col(i)(4) * dt,
            sigmas.col(i)(5) * dt,
            sigmas.col(i)(6) * dt);  

        attitude = attitude * uq_omega;

        X_sigma.col(i)(0) = attitude.s;
        X_sigma.col(i)(1) = attitude.v_1;
        X_sigma.col(i)(2) = attitude.v_2;
        X_sigma.col(i)(3) = attitude.v_3;


        //position
        double roll, pitch, yaw;
        tf2::Quaternion quat (sigmas.col(i)(0),
        sigmas.col(i)(1),
        sigmas.col(i)(2),
        sigmas.col(i)(3));

        tf2::Matrix3x3 m(quat);
        m.getRPY(roll, pitch, yaw);

        ROVER rover;    
        rover.calculate_wheel_change(w, dt);
        //position
        // Update x and y positions
        double linear_velocity = rover.rover_speeds(0);

        // Calculate change in x and y positions
        double dx = linear_velocity * cos(yaw) * dt;
        double dy = linear_velocity * sin(yaw) * dt;

        // Update x and y positions
        X_sigma.col(i)(7) = sigmas.col(i)(7) + dx;
        X_sigma.col(i)(8) = sigmas.col(i)(8) + dy;
    }

    // Compute unscented mean and covariance
    std::tie(x_hat, P) = unscented_transform(X_sigma,
        sigma_points.Wm,
        sigma_points.Wc,
        Q);

    // Save posterior
    UnitQuaternion uq(x_hat(0), x_hat(1), x_hat(2), x_hat(3));
    uq.normalize();
    x_prior.head(4) = uq.to_quaternion_vector();
    x_prior(4) = z_measurement(0);
    x_prior(5) = z_measurement(1);
    x_prior(6) = z_measurement(2);
    P_prior.topLeftCorner(7,7) = P.topLeftCorner(7,7);

    // Pass the transformed sigmas into measurement function
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
            /***
        Nonlinear measurement model for Orientation estimation with Quaternions

        Inputs:
        x: current sigma point of state estimate x = [q0 q1 q2 q3 omega_x omega_y omega_z].T

        Outputs:
        z_pred_sigma: sigma point after being propagated through nonlinear measurement model
        ***/
        // --- Measurement model ---
        // Extract quaternion from current state estimates 
        UnitQuaternion attitude(X_sigma.col(i)(0),
            X_sigma.col(i)(1),
            X_sigma.col(i)(2),
            X_sigma.col(i)(3));

        // Inverse: {B} to {0}
        UnitQuaternion invq = attitude.inverse();

        // Accelerometer
        Eigen::VectorXd acc_pred = invq.vector_rotation_by_quaternion(g0);

        // Magnetomer
        Eigen::VectorXd mag_pred = invq.vector_rotation_by_quaternion(m0);

        // Gyroscope
        Eigen::VectorXd gyro_pred(3);
        gyro_pred << X_sigma.col(i)(4), X_sigma.col(i)(5), X_sigma.col(i)(6);

        Z_sigma.col(i) << gyro_pred, acc_pred, mag_pred, Z_sigma.col(i)(9), Z_sigma.col(i)(10), Z_sigma.col(i)(11), Z_sigma.col(i)(12), Z_sigma.col(i)(13); // test behaviour for last two values
    }

    std::tie(z, S) = unscented_transform(Z_sigma,
        sigma_points.Wm,
        sigma_points.Wc,
        R);

    z_prior.head(9) = z.head(9);
    S_prior.topLeftCorner(9,9) = S.topLeftCorner(9,9);

    	/***
    	Update step of UKF with Quaternion + Angular Velocity model i.e state space is:
        
        	x = [q0 q1 q2 q3 omega_x omega_y omega_z].T
            	z = [z_gyro z_acc z_mag].T
                
                	Inputs:
                    	z_measurement: Sensor measurements from gyroscope, accelerometer and magnetometer
                        	***/

	    // Compute cross covariance
	    Eigen::MatrixXd T = Eigen::MatrixXd::Zero(x_dim, z_dim);
        for (int i = 0; i < sigma_points.num_sigma_points; i++)
        {
	    	T = T + sigma_points.Wc(i) * (X_sigma.col(i) - x_prior) * (Z_sigma.col(i) - z_prior).transpose();
	    }

	    // Compute Kalman gain
	    Eigen::MatrixXd K = T * S.inverse();

	    // Update state estimate
            x_hat = x_hat + K * (z_measurement - z_prior); // x_hat is defined in constructor for as a temp vector (overwriting x_post)

            // Update covariance
            P = P_prior - K * S_prior * K.transpose();

            // Save posterior
            x_post.head(4) = x_hat.head(4);
            x_post(4) = z_measurement(0);
            x_post(5) = z_measurement(1);
            x_post(6) = z_measurement(2);
            x_post(7) = x_hat(7);
            x_post(8) = x_hat(8);
            P_post= P;
}

void UKF::gps_callback( Eigen::VectorXd z_measurement, double lon0, double lat0)
{
    /***
    Predict with wheel odometry process model
    u_t: Measured wheels velocity as input
    ***/
    // Compute the sigma points for given mean and posteriori covariance
    Eigen::MatrixXd sigmas = sigma_points.calculate_sigma_points(x_post, P_post);

    //cout << "P_Post: \n"<< P_post << endl;

    std::array<double, 2> WGS84Reference{lon0, lat0};
    std::array<double, 2> result{wgs84::toCartesian(WGS84Reference, {z_measurement[10], z_measurement[9]})};
    std::cout << "lon0: " << fixed << setprecision(numeric_limits<double>::max_digits10) << lon0 << " | lat0: " << lat0 << endl;
    std::cout << "lon: " << z_measurement[10] << " | lat: " << z_measurement[9] << endl;
    std::cout << "x: " << result[0] << " | y: " << result[1] << endl;
    z_measurement[11] = result[0];
    z_measurement[12] = result[1];

    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        std::array<double, 2> result2{wgs84::fromCartesian(WGS84Reference, {sigmas.col(i)(7), sigmas.col(i)(8)})};
        //double lat = lat0 + (180 / PI) * (sigmas.col(i)(7) / 6378137);
        //double lon = lon0 + (180 / PI) * (sigmas.col(i)(8) / 6378137) / cos(lat0 * PI /180.0);
        double lon = result2[0];
        double lat = result2[1];
        //std::cout << "i: " << i << " | x: " << sigmas.col(i)(7) << " | y: " << sigmas.col(i)(8) << " | lat: " << lat << " | lon: " << lon << endl;

        Z_sigma.col(i) << Z_sigma.col(i)(0), Z_sigma.col(i)(1), Z_sigma.col(i)(2), Z_sigma.col(i)(3), Z_sigma.col(i)(4), Z_sigma.col(i)(5), Z_sigma.col(i)(6),
                        Z_sigma.col(i)(7), Z_sigma.col(i)(8), lat, lon, sigmas.col(i)(7), sigmas.col(i)(8), Z_sigma.col(i)(13);

        X_sigma.col(i) << X_sigma.col(i)(0),X_sigma.col(i)(1),X_sigma.col(i)(2),X_sigma.col(i)(3), 
                        X_sigma.col(i)(4), X_sigma.col(i)(5), X_sigma.col(i)(6),
                        sigmas.col(i)(7), sigmas.col(i)(8);

    }
    
    /*
    std::tie(z, S) = unscented_transform(Z_sigma,
        sigma_points.Wm,
        sigma_points.Wc,
        R);
    */
    
    //cout << "Z is as following:\n" << z << endl;
    
    std::tie(x_hat, P) = unscented_transform(X_sigma,
        sigma_points.Wm,
        sigma_points.Wc,
        Q);
    
    /*
    z_prior(9) = z(9);
    z_prior(10) = z(10);
    z_prior(11) = z(11);
    z_prior(12) = z(12);

    S_prior.col(9)  = S.col(9);
    S_prior.col(10) = S.col(10);
    S_prior.col(11) = S.col(11);
    S_prior.col(12) = S.col(12);
    S_prior.row(9)  = S.row(9);
    S_prior.row(10) = S.row(10);
    S_prior.row(11) = S.row(11);
    S_prior.row(12) = S.row(12);

    x_prior(7) = x_hat(7);
    x_prior(8) = x_hat(8);

    P_prior.col(7) = P.col(7);
    P_prior.col(8) = P.col(8);
    P_prior.row(7) = P.row(7);
    P_prior.row(8) = P.row(8);
    */


    	/***
    	Update step of UKF with Quaternion + Angular Velocity model i.e state space is:
        
        	x = [q0 q1 q2 q3 omega_x omega_y omega_z x y].T
            	z = [z_gyro z_acc z_mag z_gps].T
                
                	Inputs:
                    	z_measurement: Sensor measurements from gyroscope, accelerometer, magnetometer and GPS
                        	***/

    // Compute cross covariance
//Eigen::MatrixXd T = Eigen::MatrixXd::Zero(x_dim, z_dim);
    //Eigen::MatrixXd Tc(2, 3);
    //Tc.setZero();
//for (int i = 0; i < sigma_points.num_sigma_points; i++)
//{
//    T = T + sigma_points.Wc(i) * (X_sigma.col(i) - x_prior) * (Z_sigma.col(i) - z_prior).transpose();

        //cout << fixed << setprecision(numeric_limits<double>::max_digits10) << "X diff: \n" << X_sigma.col(i) - x_prior << endl;
        //cout << fixed << setprecision(numeric_limits<double>::max_digits10) << "Z diff: \n" << Z_sigma.col(i) - z_prior << endl;
        //Eigen::VectorXd dx = X_sigma.col(i).tail(2) - x_prior.tail(2);
        //Eigen::Vector3d dz = Z_sigma.col(i).tail(3) - z_prior.tail(3);
        /*
        cout << "created dx and dz:" << endl<< dx << endl << dz << endl;
        cout << "multi:" << dx * dz.transpose() << endl;
        */
        //Tc = Tc + sigma_points.Wc(i) * (dx * dz.transpose());
//}

    /*
    Eigen::Matrix3d Sc = Eigen::Matrix3d::Zero();
    Sc.col(0) = S.col(10).tail(3);
    Sc.col(1) = S.col(11).tail(3);
    Sc.col(2) = S.col(12).tail(3);
    */

    // Compute Kalman gain
//Eigen::MatrixXd K = T * S.inverse();
    //cout << "K is calculated: \n" << K << endl;
    //Eigen::MatrixXd Kc = Tc * Sc.inverse();
    //cout << "K is calculated: \n" << Kc << endl;
    //cout << z_measurement.tail(3) - z_prior.tail(3);
    //cout << "Added: \n" << Kc * (z_measurement.tail(3) - z_prior.tail(3)) << endl;
    

    
    //Eigen::MatrixXd myK = Eigen::MatrixXd::Zero(9, 14);  // start with all zeros
    // Apply strong gain (e.g., 0.9) to z[12] and z[13], affecting state rows 5 and 6 (for example)
    //myK(7, 11) = 0.9;  // state x[5] is affected by z[12]
    //myK(8, 12) = 0.9;  // state x[6] is affected by z[13]
    
    // Update state estimate
/*Prev code was here*/
    //x_hat = x_hat + K * (z_measurement - z_prior); // x_hat is defined in constructor for as a temp vector (overwriting x_post)
    //cout << "The gain:\n" << K * (z_measurement - z_prior) << endl;

    /*
    // Step 1: extract residual
    Eigen::Vector2d z_diff = z_measurement.segment<2>(11) - z_prior.segment<2>(11);
    // Step 2: get only position-related Kalman gain
    Eigen::MatrixXd K_pos = K.block(0, 12, 9, 2);  // 9x2
    // Step 3: state update
    x_hat = x_hat + K_pos * z_diff;
    */


    // Update covariance
//P = P_prior - K * S_prior * K.transpose(); ////////////////////////////
    /*
    Eigen::MatrixXd S_pos = S.block(12, 12, 2, 2);
    Eigen::MatrixXd Tc_pos = T.block(0, 12, 9, 2);
    Eigen::MatrixXd k_pos = Tc_pos * S_pos.inverse();
    P = P - k_pos * S_pos * K_pos.transpose();
    */

    //P.col(7) = P_prior.col(7).tail(2) - K * Sc * K.transpose();
    //P.col(8) = P_prior.col(8).tail(2) - K * Sc * K.transpose();

    // Save posterior
    
    //x_post.tail(2) = x_hat.tail(2);
    
    P_post.col(7) = P.col(7);
    P_post.col(8) = P.col(8);
    P_post.row(7) = P.row(7);
    P_post.row(8) = P.row(8);
    

    //std::cout << "The result: \n"<< x_hat.tail(2) << endl;


    Eigen::Vector2d z_gps(result[0], result[1]);
    Eigen::VectorXd x_mean = Eigen::VectorXd::Zero(x_dim);
    for (int i = 0; i < 2 * x_dim + 1; i++)
        x_mean += sigma_points.Wm(i) * X_sigma.col(i);

    Eigen::MatrixXd Z_sigma2(2, 2 * x_dim + 1);
    for (int i = 0; i < 2 * x_dim + 1; i++) {
        // Extract x, y from state sigma point
        double x = X_sigma(7, i);
        double y = X_sigma(8, i);
        Z_sigma2.col(i) << x, y;
    }
    
    // Predicted measurement mean
    Eigen::Vector2d z_mean = Eigen::Vector2d::Zero();
    for (int i = 0; i < 2 * x_dim + 1; i++)
        z_mean += sigma_points.Wm(i) * Z_sigma2.col(i);

    // Measurement covariance
    Eigen::Matrix2d S2 = Eigen::Matrix2d::Zero();
    for (int i = 0; i < 2 * x_dim + 1; i++) {
        Eigen::Vector2d dz = Z_sigma2.col(i) - z_mean;
        S2 += sigma_points.Wc(i) * dz * dz.transpose();
    }
    // Add GPS noise
    Eigen::Matrix2d R_gps;
    R_gps << 1, 0,
        0, 1;
    S2 += R_gps; // 2x2, diagonal with variances in meters^2

    Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(x_dim, 2);
    for (int i = 0; i < 2 * x_dim + 1; i++) {
        Eigen::VectorXd dx = X_sigma.col(i) - x_mean;
        Eigen::Vector2d dz = Z_sigma2.col(i) - z_mean;
        Tc += sigma_points.Wc(i) * dx * dz.transpose();
    }

    Eigen::MatrixXd K2 = Tc * S2.inverse();  // K: (n_x x 2)

    x_hat = x_mean + K2 * (z_gps - z_mean);
    P = P_prior - K2 * S2 * K2.transpose();

    std::cout << "The result2: \n"<< x_hat.tail(2) << endl;

    x_prior = x_post;
    P_prior = P_post;

    x_post.tail(2) = x_hat.tail(2);
    /*
    P_post.col(7) = P.col(7);
    P_post.col(8) = P.col(8);
    P_post.row(7) = P.row(7);
    P_post.row(8) = P.row(8);
    */


}


void UKF::bno_callback(double roll, double pitch ,double yaw)
{
    Quaternion q(roll, pitch, yaw);
    x_post(0) = q.s;
    x_post(1) = q.v_1;
    x_post(2) = q.v_2;
    x_post(3) = q.v_3;
}

void UKF::LL_Callback( Eigen::VectorXd z_measurement){
    
    //inputs: X and Y absolute positions of the rover 

    Eigen::MatrixXd sigmas = sigma_points.calculate_sigma_points(x_post, P_post);

    double X0 = z_measurement[11]; 
    double Y0 = z_measurement[12];
    // double Z0 = z_measurement[13];

    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        double dx = X0 - x_post(7); 
        double dy = Y0 - x_post(8);
        double X = dx + sigmas.col(i)(7); //from state space model
        double Y = dy + sigmas.col(i)(8); //from state space model
        
        X_sigma.col(i)(7) = X;
        X_sigma.col(i)(8) = Y;

        Z_sigma.col(i) << Z_sigma.col(i)(0), Z_sigma.col(i)(1), Z_sigma.col(i)(2), Z_sigma.col(i)(3), Z_sigma.col(i)(4), Z_sigma.col(i)(5), Z_sigma.col(i)(6),
                        Z_sigma.col(i)(7), Z_sigma.col(i)(8), Z_sigma.col(i)(9), Z_sigma.col(i)(10),
                        X_sigma.col(i)(7),
                        X_sigma.col(i)(8),
                        Z_sigma.col(i)(13);
    }
    
    std::tie(z, S) = unscented_transform(Z_sigma, sigma_points.Wm, sigma_points.Wc, R);
    
    // Save posterior
     z_prior.tail(3) = z.tail(3);
     
     S_prior.col(11)  = S.col(11);
     S_prior.col(12) = S.col(12);
     S_prior.col(13) = S.col(13);

     S_prior.row(11)  = S.row(11);
     S_prior.row(12) = S.row(12);
     S_prior.row(13) = S.row(13);

    Eigen::MatrixXd T = Eigen::MatrixXd::Zero(x_dim, z_dim);
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
	   	T = T + sigma_points.Wc(i) * (X_sigma.col(i) - x_prior) * (Z_sigma.col(i) - z_prior).transpose();
	}
    
    // Compute Kalman gain
    Eigen::MatrixXd K = T * S.inverse();

    // Update state estimate
    x_hat = x_hat + K * (z_measurement - z_prior); // x_hat is defined in constructor for as a temp vector (overwriting x_post)

    // Update covariance
    P = P_prior - K * S_prior * K.transpose();

    // Save posterior
    x_post.tail(2) = x_hat.tail(2);
    P_post.col(7) = P.col(7);
    P_post.col(8) = P.col(8);
    P_post.row(7) = P.row(7);
    P_post.row(8) = P.row(8);
}