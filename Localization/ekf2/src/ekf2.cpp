#include <iostream>
#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;
using namespace std;

//------------------------------------------------------------------------------
// Extended Kalman Filter (EKF) Class
//------------------------------------------------------------------------------
class EKF {
public:
    // Constructor: initializes the state and covariance.
    EKF();

    // Prediction step:
    // dt: time step
    // u: control vector (u[0] = angular velocity (omega), u[1] = linear acceleration (a))
    void predict(double dt, const Vector2d &u);

    // Update step for a GPS measurement (which measures x and y).
    // z: measurement vector [x, y]
    // R: measurement noise covariance matrix for GPS
    void updateGPS(const Vector2d &z, const Matrix2d &R);

    // Update step for an IMU orientation measurement.
    // z: measured orientation (theta)
    // R_scalar: measurement noise variance (as a scalar) for the orientation measurement
    void updateIMU(const double z, const double R_scalar);

    // Public state vector and covariance matrix.
    // x = [x, y, theta, v]^T
    Vector4d x;
    Matrix4d P;

private:
    // Helper function to compute the Jacobian (F) of the process model.
    Matrix4d computeJacobianF(double dt, const Vector2d &u);
};

//------------------------------------------------------------------------------
// EKF Constructor Implementation
//------------------------------------------------------------------------------
EKF::EKF() {
    // Initialize state (for example, starting at the origin with no movement)
    x << 0, 0, 0, 0;
    
    // Initialize the covariance matrix.
    // Here we assume a small initial uncertainty; you may need to tune this.
    P = Matrix4d::Identity() * 0.1;
}

//------------------------------------------------------------------------------
// Prediction Step: Propagate the state and covariance.
//------------------------------------------------------------------------------
void EKF::predict(double dt, const Vector2d &u) {
    // Extract the current state components.
    double theta = x(2);
    double v = x(3);
    double omega = u(0);  // angular velocity
    double a = u(1);      // linear acceleration

    // Apply the nonlinear process model:
    // x_new = x + v * cos(theta) * dt
    // y_new = y + v * sin(theta) * dt
    // theta_new = theta + omega * dt
    // v_new = v + a * dt
    Vector4d x_pred;
    x_pred(0) = x(0) + v * cos(theta) * dt;
    x_pred(1) = x(1) + v * sin(theta) * dt;
    x_pred(2) = x(2) + omega * dt;
    x_pred(3) = x(3) + a * dt;

    // Update the state estimate.
    x = x_pred;

    // Compute the Jacobian F = df/dx of the process model.
    Matrix4d F = computeJacobianF(dt, u);

    // Define the process noise covariance Q.
    // These values should be tuned based on your system's uncertainty.
    Matrix4d Q = Matrix4d::Zero();
    Q(0,0) = 0.01;
    Q(1,1) = 0.01;
    Q(2,2) = 0.001;
    Q(3,3) = 0.1;

    // Update the covariance using the EKF prediction equation.
    // P_k|k-1 = F * P_k-1|k-1 * F^T + Q
    P = F * P * F.transpose() + Q;
}

//------------------------------------------------------------------------------
// Compute the Jacobian of the Process Model (F)
//------------------------------------------------------------------------------
Matrix4d EKF::computeJacobianF(double dt, const Vector2d &u) {
    double theta = x(2);
    double v = x(3);

    // Start with an identity matrix.
    Matrix4d F = Matrix4d::Identity();

    // The partial derivative of x with respect to theta:
    // d/dtheta (v*cos(theta)*dt) = -v*sin(theta)*dt
    F(0,2) = -v * sin(theta) * dt;
    // The partial derivative of x with respect to v:
    // d/dv (v*cos(theta)*dt) = cos(theta)*dt
    F(0,3) = cos(theta) * dt;

    // Similarly, for y:
    // d/dtheta (v*sin(theta)*dt) = v*cos(theta)*dt
    F(1,2) = v * cos(theta) * dt;
    // d/dv (v*sin(theta)*dt) = sin(theta)*dt
    F(1,3) = sin(theta) * dt;

    // The derivatives for theta and v with respect to themselves are 1 (already set in Identity).
    return F;
}

//------------------------------------------------------------------------------
// Update Step for a GPS Measurement
//------------------------------------------------------------------------------
void EKF::updateGPS(const Vector2d &z, const Matrix2d &R) {
    // The observation model for GPS is:
    // h(x) = [x, y]^T.
    Vector2d z_pred;
    z_pred(0) = x(0);
    z_pred(1) = x(1);

    // Compute the innovation (residual): the difference between actual and predicted measurement.
    Vector2d y = z - z_pred;

    // The measurement Jacobian H is the derivative of h(x) with respect to x.
    // For h(x) = [x, y]^T, the Jacobian is:
    // H = [1 0 0 0; 0 1 0 0]
    Matrix<double, 2, 4> H;
    H.setZero();
    H(0,0) = 1.0;
    H(1,1) = 1.0;

    // Compute the innovation covariance: S = H * P * H^T + R
    Matrix2d S = H * P * H.transpose() + R;

    // Compute the Kalman Gain: K = P * H^T * S^-1
    Matrix<double, 4, 2> K = P * H.transpose() * S.inverse();

    // Update the state with the innovation scaled by the Kalman Gain.
    x = x + K * y;

    // Update the covariance matrix.
    Matrix4d I = Matrix4d::Identity();
    P = (I - K * H) * P;
}

//------------------------------------------------------------------------------
// Update Step for an IMU Orientation Measurement
//------------------------------------------------------------------------------
void EKF::updateIMU(const double z, const double R_scalar) {
    // The observation model for the IMU (orientation) is:
    // h(x) = theta, which is the third element of the state vector.
    double z_pred = x(2);
    double y = z - z_pred;  // Innovation (residual)

    // The measurement Jacobian for this measurement is:
    // H = [0 0 1 0]
    Matrix<double, 1, 4> H;
    H << 0, 0, 1, 0;

    // Compute the innovation covariance (scalar in this case):
    double S = (H * P * H.transpose())(0, 0) + R_scalar;

    // Compute the Kalman Gain: K = P * H^T / S
    Vector4d K = P * H.transpose() / S;

    // Update the state.
    x = x + K * y;

    // Update the covariance matrix.
    Matrix4d I = Matrix4d::Identity();
    P = (I - K * H) * P;
}

const int L = 1; //length between 2 wheels "constant from rover's geometry"
float thetaRover = 0; //state no.3
float Vix = 0; //Velocity in X direction state no.4
float Viy = 0; //Velocity in Y direction state no.5
Vector4d x(49.8, 8.9, thetaRover, 0);
Vector2d P1(0, 0); //inital reading (must be taken from the IMU at start)

Vector4d x_pred;
Matrix4d P = Matrix4d::Identity() * 0.1;

EKF ekf;

//The following 3 functions are the nodes callbacks "IMU, GPS and Encoders"

void imuCallback(const sensor_msgs::Imu::ConstPtr& imuMsg) {
    Eigen::Quaterniond q(imuMsg->orientation.w, imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z); //getting the quaternions values
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);  // Yaw-Pitch-Roll //turn the quaternions to euler angles
    
    /*
    ROS_INFO("Linear Acceleration: x=%.3f, y=%.3f, z=%.3f", linearAccX, linearAccy, linearAccz);
    ROS_INFO("Yaw: %.3f, Pitch: %.3f, Roll: %.3f", euler[0], euler[1], euler[2]); //interested in yaw "Rotation around Z"
    */

    ekf.updateIMU(euler[0], 0.001);
  }
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gpsMsg) {
    ROS_INFO("GPS Data: Latitude: %.3f, Longitude: %.3f, Altitude: %.3f meters", gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);
    //you can get the new Positions Latitude and Longitude but convert them to X and Y
    
    //update step
    Eigen::Vector2d gpsReading(gpsMsg->latitude,gpsMsg->longitude);
    Matrix2d R_gps = Matrix2d::Identity() * 0.025;
    ekf.updateGPS(gpsReading, R_gps);

      }


int main(int argc, char **argv) {
    
    ros::init(argc, argv, "ekf");
    
    ros::NodeHandle nhIMU;
    ros::NodeHandle nhGPS;
    ros::NodeHandle nhENCODER;
  
    ros::Subscriber imuSub = nhIMU.subscribe("imu", 1000, imuCallback);
    ros::Subscriber gpsSub = nhGPS.subscribe("gps", 1000, gpsCallback);
    //ros::Subscriber encoderSub = nhENCODER.subscribe("joint_states", 1000, encoderCallback);
    
    ros::Rate rate(20);
    while (ros::ok()){
     // Define a control vector u: [omega, a]
    // For this example, let omega = 0.1 rad/s and acceleration = 0.2 m/s^2.
      Vector2d u;
      u << x(2), 0;
      ekf.predict(0.05,u);

      std::cout << "***************\n" << x << "\n";
      ros::spinOnce();
      rate.sleep();
    }
    
    return 0;
}