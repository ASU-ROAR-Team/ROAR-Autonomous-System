#include "ROAR_UKF.h"
#include "WGS84toCartesian.hpp"
#include <iostream>
#include <limits>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <XmlRpcValue.h>

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
    XmlRpc::XmlRpcValue initPose;
    x_hat << initPose["orientation"]["w"], initPose["orientation"]["x"], initPose["orientation"]["y"], initPose["orientation"]["z"], 0, 0, 0, initPose["position"]["x"], initPose["position"]["y"];   // Initial Quaterion: 1+0i+0j+0k and initial ang vel: [0 0 0].T

    // Initialize z state vector
    z_dim = 14; 
    z = Eigen::VectorXd::Zero(z_dim); // Initial measurement in frame {B}, [z_gyro, z_acc, z_mag].T

    // Intialize Posteriori Estimate Covariance Matrix
    //P = Eigen::MatrixXd::Zero(x_dim, x_dim);
    P = Eigen::MatrixXd::Identity(x_dim, x_dim)*0.5;  // The P should be set indvidually for each diagonal element, not like this
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
    Q = Eigen::MatrixXd::Identity(x_dim, x_dim) * 1e-2;    // Process Noise Matrix //research

    R = Eigen::MatrixXd::Identity(z_dim, z_dim) * 0.7;      // Measurement Noise Matrix //add noise covariance for each sensor from datasheet

    // Intialize inertial frame quantities
    g0 << 0, 0, 1;                          // Gravitational Acceleration Vector
    m0 << B_INTENSITY * cos(INCLINATION),   // Magnetic Field Intensity Vector
        0.0,
        B_INTENSITY* sin(INCLINATION);

    yaw = 0.0;
    pitch = 0.0;

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

void UKF::encoder_callback(Eigen::VectorXd w, double dt, double yaw, double pitch)
{
    ROS_DEBUG("[*] UKF -> Encoder Callback called");
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
        double dx = - linear_velocity * sin(yaw) * cos(pitch) * dt; //the bno reads 0 from the -y so I added a -ve and switched the sin and cos
        double dy = - linear_velocity * cos(yaw) * cos(pitch) * dt;

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

    ROS_DEBUG("[*] UKF -> Encoder Callback finished");
}

void UKF::imu_callback(Eigen::VectorXd w, Eigen::VectorXd z_measurement, double dt, double roll, double yaw, double pitch)
{
    ROS_DEBUG("[*] UKF -> IMU Callback called");
    ROS_DEBUG("[+] UKF -> IMU Callback: Position Started");
        /***
    Predict with wheel odometry process model
    u_t: Measured wheels velocity as input
    ***/
    // Compute the sigma points for given mean and posteriori covariance
    if (x_post(0) == 1)
    {
        x_post << 1, 0, 0, 0, 0, 0, 0, 0, 0;
    }
    if (P_post(0,0) == 0.5)
    {
        P_post = Eigen::MatrixXd::Identity(x_dim, x_dim)*0.5;
    }
    Eigen::MatrixXd sigmas = sigma_points.calculate_sigma_points(x_post, P_post);

    ROVER rover;
    rover.calculate_wheel_change(w, dt);
    //position
    // Update x and y positions
    double linear_velocity = rover.rover_speeds(0);

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
        attitude.normalize();

        X_sigma.col(i)(0) = attitude.s;
        X_sigma.col(i)(1) = attitude.v_1;
        X_sigma.col(i)(2) = attitude.v_2;
        X_sigma.col(i)(3) = attitude.v_3;

        //position
        // Calculate change in x and y positions
        double dx = - linear_velocity * sin(yaw) * cos(pitch) * dt;
        double dy = - linear_velocity * cos(yaw) * cos(pitch) * dt;
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
    x_prior(4) = x_hat(4);
    x_prior(5) = x_hat(5);
    x_prior(6) = x_hat(6);
    x_prior(7) = x_hat(7);
    x_prior(8) = x_hat(8);
    P_prior.topLeftCorner(9,9) = P.topLeftCorner(9,9);

    // Pass the transformed sigmas into measurement function
    Z_sigma.setZero();
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        UnitQuaternion att_pred(X_sigma(0,i), X_sigma(1,i), X_sigma(2,i), X_sigma(3,i));
        UnitQuaternion invq_pred = att_pred.inverse();

        Eigen::VectorXd gyro_pred(3);
        gyro_pred << X_sigma.col(i)(4), X_sigma.col(i)(5), X_sigma.col(i)(6);
        Eigen::VectorXd acc_pred = invq_pred.vector_rotation_by_quaternion(g0);
        Eigen::VectorXd mag_pred = invq_pred.vector_rotation_by_quaternion(m0);
        Z_sigma.col(i) << gyro_pred, acc_pred, mag_pred, 
                         Z_sigma(9,i), Z_sigma(10,i), Z_sigma(11,i), Z_sigma(12,i), Z_sigma(13,i);
    }

    std::tie(z, S) = unscented_transform(Z_sigma,
                                         sigma_points.Wm,
                                         sigma_points.Wc,
                                         R);

    z_prior = z;
    S_prior.topLeftCorner(9,9) = S.topLeftCorner(9,9);

    /***
    Update step of UKF with Quaternion + Angular Velocity model i.e state space is:
    x = [q0 q1 q2 q3 omega_x omega_y omega_z].T
    z = [z_gyro z_acc z_mag].T
    Inputs:
    z_measurement: Sensor measurements from gyroscope, accelerometer and magnetometer
    ***/

    Eigen::MatrixXd T = Eigen::MatrixXd::Zero(x_dim, z_dim);
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        T += sigma_points.Wc(i) * 
             (X_sigma.col(i) - x_prior) *
             (Z_sigma.col(i) - z_prior).transpose();
    }

    Eigen::MatrixXd K = T * S.inverse();

    Eigen::VectorXd innovation = z_measurement.head(z_dim) - z_prior.head(z_dim);
    Eigen::VectorXd state_update = K * innovation;
    x_hat += state_update;

    P = P_prior - K * S_prior * K.transpose();

    ROS_DEBUG("[+] UKF -> IMU Callback: Position Finished");
    ROS_WARN(x_post.allFinite() ? "[+] UKF -> IMU Callback: State is finite" : "[!] UKF -> IMU Callback: State is not finite");
    ROS_WARN(P_post.allFinite() ? "[+] UKF -> IMU Callback: P is finite" : "[!] UKF -> IMU Callback: P is not finite");

    ROS_DEBUG("[*] UKF -> IMU Callback: Orientation Started");
    
    Quaternion q(roll, pitch, yaw);
    x_post(0) = q.s;
    x_post(1) = q.v_1;
    x_post(2) = q.v_2;
    x_post(3) = q.v_3;
    
    x_prior = x_post;
    P_prior = P_post;

    ROS_DEBUG("[+] UKF -> IMU Callback: Orientation Finished");
    
    ROS_DEBUG("[*] UKF -> IMU Callback finished");

    std::cout << "IMU x_post: " << x_post.transpose() << std::endl;
    
    ROS_WARN(x_post.allFinite() ? "[+] UKF -> IMU Callback: State is finite" : "[!] UKF -> IMU Callback: State is not finite");
    ROS_WARN(P_post.allFinite() ? "[+] UKF -> IMU Callback: P is finite" : "[!] UKF -> IMU Callback: P is not finite");

    

}

void UKF::gps_callback( Eigen::VectorXd z_measurement, double lon0, double lat0, double Rgps)
{
    ROS_DEBUG("[*] UKF -> GPS Callback called");
    /***
    Predict with wheel odometry process model
    u_t: Measured wheels velocity as input
    ***/
    // Compute the sigma points for given mean and posteriori covariance
    Eigen::MatrixXd sigmas = sigma_points.calculate_sigma_points(x_post, P_post);
    
    //Save post
    x_prior = x_post;
    P_prior = P_post;

    std::array<double, 2> WGS84Reference{lon0, lat0};
    std::array<double, 2> result{wgs84::toCartesian(WGS84Reference, {z_measurement[10], z_measurement[9]})};
    result[1] = result[1] *2/3; //the 2/3 is for mapping the readings with gazebo world
    z_measurement[11] = result[0];
    z_measurement[12] = result[1];

    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        std::array<double, 2> result2{wgs84::fromCartesian(WGS84Reference, {sigmas.col(i)(7), sigmas.col(i)(8)})};
        double lon = result2[0];
        double lat = result2[1];

        Z_sigma.col(i) << Z_sigma.col(i)(0), Z_sigma.col(i)(1), Z_sigma.col(i)(2), Z_sigma.col(i)(3), Z_sigma.col(i)(4), Z_sigma.col(i)(5), Z_sigma.col(i)(6),
                        Z_sigma.col(i)(7), Z_sigma.col(i)(8), lat, lon, sigmas.col(i)(7), sigmas.col(i)(8), Z_sigma.col(i)(13);

        X_sigma.col(i) << X_sigma.col(i)(0),X_sigma.col(i)(1),X_sigma.col(i)(2),X_sigma.col(i)(3), 
                        X_sigma.col(i)(4), X_sigma.col(i)(5), X_sigma.col(i)(6),
                        sigmas.col(i)(7), sigmas.col(i)(8);

    }
    
    std::tie(x_hat, P) = unscented_transform(X_sigma,
        sigma_points.Wm,
        sigma_points.Wc,
        Q);

    // GPS measurement
    Eigen::Vector2d z_gps(result[0], result[1]);

    // Predicted state mean
    Eigen::VectorXd x_mean = Eigen::VectorXd::Zero(x_dim);
    for (int i = 0; i < 2 * x_dim + 1; ++i)
        x_mean += sigma_points.Wm(i) * X_sigma.col(i);

    // Project sigma points into GPS measurement space (only x, y)
    Eigen::MatrixXd Z_sigma = Eigen::MatrixXd(2, 2 * x_dim + 1);
    for (int i = 0; i < 2 * x_dim + 1; ++i) {
        Z_sigma(0, i) = X_sigma(7, i);  // x
        Z_sigma(1, i) = X_sigma(8, i);  // y
    }

    // Predicted GPS measurement mean
    Eigen::Vector2d z_mean = Eigen::Vector2d::Zero();
    for (int i = 0; i < 2 * x_dim + 1; ++i)
        z_mean += sigma_points.Wm(i) * Z_sigma.col(i);

    // Measurement covariance S
    Eigen::Matrix2d S = Eigen::Matrix2d::Zero();
    for (int i = 0; i < 2 * x_dim + 1; ++i) {
        Eigen::Vector2d dz = Z_sigma.col(i) - z_mean;
        S += sigma_points.Wc(i) * dz * dz.transpose();
    }

    // Add GPS noise
    Eigen::Matrix2d R_gps;
    R_gps << Rgps, 0.0,
            0.0, Rgps;
    S += R_gps;

    // Cross-covariance between state and measurement
    Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(x_dim, 2);
    for (int i = 0; i < 2 * x_dim + 1; ++i) {
        Eigen::VectorXd dx = X_sigma.col(i) - x_mean;
        Eigen::Vector2d dz = Z_sigma.col(i) - z_mean;
        Tc += sigma_points.Wc(i) * dx * dz.transpose();
    }

    // Kalman gain
    Eigen::MatrixXd K = Tc * S.inverse();

    // Update state estimate
    x_hat = x_mean + K * (z_gps - z_mean);

    // Update covariance
    P = P_prior - K * S * K.transpose();

    // Partial update (only x, y)
    x_post.tail(2) = x_hat.tail(2);

    // Update P_post only for x and y rows/cols (indices 7 and 8)
    P_post.block(7, 0, 2, x_dim) = P.block(7, 0, 2, x_dim);
    P_post.block(0, 7, x_dim, 2) = P.block(0, 7, x_dim, 2);

    ROS_DEBUG("[*] UKF -> GPS Callback Finished");

}

void UKF::bno_callback(double roll, double pitch ,double yaw)
{
    Quaternion q(roll, pitch, yaw);
    x_post(0) = q.s;
    x_post(1) = q.v_1;
    x_post(2) = q.v_2;
    x_post(3) = q.v_3;
}

void UKF::planBCallback(Eigen::VectorXd planBstate, double lat0, double lon0, double ZEDX, double ZEDY, int Plan_B_number, const std::array<double, 15>& valid_array){
    ROS_WARN("[*] UKF -> PLAN B CALLBACK STARTED");

    // x = [q0 q1 q2 q3 omega_x, omega_y, omega_z x y].T
    // Extract the state from planBstate

    

    if(Plan_B_number == 1){
        ROS_WARN("[*] UKF -> PLAN B CALLBACK: PLAN B (1) [ZED]");
        x_post(7) = ZEDX;
        x_post(8) = ZEDY;
    } else if (Plan_B_number == 2){
        ROS_WARN("[*] UKF -> PLAN B CALLBACK: PLAN B (2) [Array]");
        // Transforming to moving rover frame

        x_post(7) = valid_array[0];
        x_post(8) = valid_array[1];
    } else if (Plan_B_number == 3){
        ROS_WARN("[*] UKF -> PLAN B CALLBACK: PLAN B (3) [GPS]");
        std::array<double, 2> result{wgs84::toCartesian({lon0, lat0}, {planBstate(8), planBstate(7)})};
        //result[1] = result[1] *2/3; //the 2/3 is for mapping the readings with gazebo world
        x_post(7) = result[0];
        x_post(8) = result[1];
        
    } else {
        ROS_ERROR("[*] UKF -> PLAN B CALLBACK: UNKNOWN PLAN B NUMBER");
    }

    x_post(0) = planBstate(0);
    x_post(1) = planBstate(1);
    x_post(2) = planBstate(2);
    x_post(3) = planBstate(3);
    x_post(4) = planBstate(4);
    x_post(5) = planBstate(5);
    x_post(6) = planBstate(6);

    P_post = Eigen::MatrixXd::Identity(x_dim, x_dim) * 0.5; // Reset covariance to identity scaled by 0.5
    
    ROS_WARN("[*] UKF -> PLAN B CALLBACK ENDDED");
}

void UKF::LL_Callback( Eigen::VectorXd z_measurement, double currentX, double currentY, double RLL){

    ROS_DEBUG("[*] UKF -> LL Callback called");
    
    //inputs: X and Y absolute positions of the rover

    Eigen::VectorXd rotatedX = Eigen::VectorXd::Zero(9);
    rotatedX << x_post(0), x_post(1), x_post(2), x_post(3), x_post(4), x_post(5), x_post(6), currentX, currentY;

    Eigen::MatrixXd sigmas = sigma_points.calculate_sigma_points(rotatedX, P_post);

    double X0 = z_measurement[11]; 
    double Y0 = z_measurement[12];

    std::cout << "The Current X: " << currentX << " | The Current Y: " << currentY << std::endl;
    std::cout << "The measured X: " << X0 << " | The measured Y: " << Y0 << std::endl;
    // double Z0 = z_measurement[13];

    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        double dx = X0 - currentX; 
        double dy = Y0 - currentY;
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

    std::tie(x_hat, P) = unscented_transform(X_sigma, sigma_points.Wm, sigma_points.Wc, Q);
    
    std::tie(z, S) = unscented_transform(Z_sigma, sigma_points.Wm, sigma_points.Wc, R);

    std::array<double, 2> result{z_measurement[11], z_measurement[12]};

    // --- [1] Set prior from last post ---
    x_prior = x_post;
    P_prior = P_post;

    // --- [2] Predict state mean from sigma points ---
    Eigen::VectorXd x_mean = Eigen::VectorXd::Zero(x_dim);
    for (int i = 0; i < 2 * x_dim + 1; ++i)
        x_mean += sigma_points.Wm(i) * X_sigma.col(i);

    // --- [3] Project sigma points (x, y) ---
    Eigen::MatrixXd Z_sigma = Eigen::MatrixXd(2, 2 * x_dim + 1);
    for (int i = 0; i < 2 * x_dim + 1; ++i) {
        Z_sigma(0, i) = X_sigma(7, i); 
        Z_sigma(1, i) = X_sigma(8, i); 
    }

    // --- [4] Predicted measurement mean ---
    Eigen::Vector2d z_mean = Eigen::Vector2d::Zero();
    for (int i = 0; i < 2 * x_dim + 1; ++i)
        z_mean += sigma_points.Wm(i) * Z_sigma.col(i);

    // --- [5] Measurement covariance S ---
    Eigen::Matrix2d S = Eigen::Matrix2d::Zero();
    for (int i = 0; i < 2 * x_dim + 1; ++i) {
        Eigen::Vector2d dz = Z_sigma.col(i) - z_mean;
        S += sigma_points.Wc(i) * dz * dz.transpose();
    }

    // --- [6] Add Lat/Lon sensor noise ---
    Eigen::Matrix2d R_LL;
    R_LL << RLL, 0.0,
            0.0, RLL;  // Adjust to your actual sensor noise in meters²
    S += R_LL;

    // --- [7] Cross-covariance between state and measurement ---
    Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(x_dim, 2);
    for (int i = 0; i < 2 * x_dim + 1; ++i) {
        Eigen::VectorXd dx = X_sigma.col(i) - x_mean;
        Eigen::Vector2d dz = Z_sigma.col(i) - z_mean;
        Tc += sigma_points.Wc(i) * dx * dz.transpose();
    }

    // --- [8] Kalman gain ---
    Eigen::MatrixXd K = Tc * S.inverse();

    std::cout << "||S||: " << S.norm() 
          << "  ||Tc||: " << Tc.norm() 
          << "  ||K||: " << K.norm() << std::endl;

    // --- [9] Measurement (Lat/Lon projection) ---
    Eigen::Vector2d z_LL(result[0], result[1]);

    // --- [10] Update state and covariance ---
    Eigen::VectorXd x_new = Eigen::VectorXd::Zero(x_dim);
    x_new.head(7) = x_mean.head(7);  // Keep quaternion and angular velocity
    x_new(7) = currentX;  // Update only x, y
    x_new(8) = currentY;  // Update only x, y
    z_mean(0) = currentX;
    z_mean(1) = currentY;
    x_hat = x_mean + K * (- z_LL + z_mean);
    std::cout << "x_mean: " << x_mean.transpose() << std::endl;
    std::cout << "x_new: " << x_new.transpose() << std::endl;
    std::cout << "x_hat: " << x_hat.transpose() << std::endl;
    std::cout << "z_LL - z_mean: " << (z_LL - z_mean).transpose() << std::endl;
    
    P = P_prior - K * S * K.transpose();

    // --- [11] Log result (optional debug) ---
    //std::cout << "The result2: \n" << x_hat.tail(2) << std::endl;

    // --- [12] Update post state ---
    std::cout << "Final X: " << x_hat(7) << " | Final Y: " << x_hat(8) << std::endl;
    //x_post(7) = x_hat(7);  // Update only x, y
    //x_post(8) = x_hat(8);  // Update only x, y

    // --- [13] Update post covariance (only x, y rows/cols) ---
    P_post.block(7, 0, 2, x_dim) = P.block(7, 0, 2, x_dim);
    P_post.block(0, 7, x_dim, 2) = P.block(0, 7, x_dim, 2);

    ROS_DEBUG("[*] UKF -> LL Callback finished");

}

void UKF::zed_Callback(Eigen::VectorXd z_measurement, double R_ZED, double currentX, double currentY){
    //zed_callback is used to update the position of the rover using ZED camera measurements
    ROS_DEBUG("[*] UKF -> ZED Callback called");

    Eigen::VectorXd rotatedX = Eigen::VectorXd::Zero(9);
    rotatedX << x_post(0), x_post(1), x_post(2), x_post(3), x_post(4), x_post(5), x_post(6), currentX, currentY;

    Eigen::MatrixXd sigmas = sigma_points.calculate_sigma_points(rotatedX, P_post);

    double X0 = z_measurement[11]; 
    double Y0 = z_measurement[12];
    double Z0 = z_measurement[13];

    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        double dx = X0 - currentX; 
        double dy = Y0 - currentY;
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

    std::tie(x_hat, P) = unscented_transform(X_sigma, sigma_points.Wm, sigma_points.Wc, Q);
    
    std::tie(z, S) = unscented_transform(Z_sigma, sigma_points.Wm, sigma_points.Wc, R);

    std::array<double, 2> result{z_measurement[11], z_measurement[12]};

    // --- [1] Set prior from last post ---
    x_prior = x_post;
    P_prior = P_post;

    // --- [2] Predict state mean from sigma points ---
    Eigen::VectorXd x_mean = Eigen::VectorXd::Zero(x_dim);
    for (int i = 0; i < 2 * x_dim + 1; ++i)
        x_mean += sigma_points.Wm(i) * X_sigma.col(i);

    // --- [3] Project sigma points (x, y) ---
    Eigen::MatrixXd Z_sigma = Eigen::MatrixXd(2, 2 * x_dim + 1);
    for (int i = 0; i < 2 * x_dim + 1; ++i) {
        Z_sigma(0, i) = X_sigma(7, i); 
        Z_sigma(1, i) = X_sigma(8, i); 
    }

    // --- [4] Predicted measurement mean ---
    Eigen::Vector2d z_mean = Eigen::Vector2d::Zero();
    for (int i = 0; i < 2 * x_dim + 1; ++i)
        z_mean += sigma_points.Wm(i) * Z_sigma.col(i);

    // --- [5] Measurement covariance S ---
    Eigen::Matrix2d S = Eigen::Matrix2d::Zero();
    for (int i = 0; i < 2 * x_dim + 1; ++i) {
        Eigen::Vector2d dz = Z_sigma.col(i) - z_mean;
        S += sigma_points.Wc(i) * dz * dz.transpose();
    }

    // --- [6] Add Lat/Lon sensor noise ---
    Eigen::Matrix2d RZED;
    RZED << R_ZED, 0.0,
            0.0, R_ZED;  // Adjust to your actual sensor noise in meters²
    S += RZED;

    // --- [7] Cross-covariance between state and measurement ---
    Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(x_dim, 2);
    for (int i = 0; i < 2 * x_dim + 1; ++i) {
        Eigen::VectorXd dx = X_sigma.col(i) - x_mean;
        Eigen::Vector2d dz = Z_sigma.col(i) - z_mean;
        Tc += sigma_points.Wc(i) * dx * dz.transpose();
    }

    // --- [8] Kalman gain ---
    Eigen::MatrixXd K = Tc * S.inverse();

    // --- [9] Measurement (x, y) ---
    Eigen::Vector2d z_LL(result[0], result[1]);

    // --- [10] Update state and covariance ---
    Eigen::VectorXd x_new = Eigen::VectorXd::Zero(x_dim);
    x_new.head(7) = x_mean.head(7);  // Keep quaternion and angular velocity
    x_new(7) = currentX;  // Update only x, y
    x_new(8) = currentY;  // Update only x, y
    z_mean(0) = currentX;
    z_mean(1) = currentY;
    x_hat = x_mean + K * (- z_LL + z_mean);
    std::cout << "x_mean: " << x_mean.transpose() << std::endl;
    std::cout << "x_new: " << x_new.transpose() << std::endl;
    std::cout << "x_hat: " << x_hat.transpose() << std::endl;
    std::cout << "z_LL - z_mean: " << (z_LL - z_mean).transpose() << std::endl;
    
    P = P_prior - K * S * K.transpose();

    // --- [11] Log result (optional debug) ---
    //std::cout << "The result2: \n" << x_hat.tail(2) << std::endl;

    // --- [12] Update post state ---
    std::cout << "Final X: " << x_hat(7) << " | Final Y: " << x_hat(8) << std::endl;
    //x_post(7) = x_hat(7);  // Update only x, y
    //x_post(8) = x_hat(8);  // Update only x, y

    // --- [13] Update post covariance (only x, y rows/cols) ---
    P_post.block(7, 0, 2, x_dim) = P.block(7, 0, 2, x_dim);
    P_post.block(0, 7, x_dim, 2) = P.block(0, 7, x_dim, 2);

    ROS_DEBUG("[*] UKF -> ZED Callback finished");
}
