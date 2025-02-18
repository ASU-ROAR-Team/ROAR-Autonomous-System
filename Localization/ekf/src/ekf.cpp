#include <iostream>
#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <cmath>

const int L = 1; //length between 2 wheels "constant from rover's geometry"
float thetaRover = 0; //state no.3
float Vix = 0; //Velocity in X direction state no.4
float Viy = 0; //Velocity in Y direction state no.5
Eigen::Vector3d x(49.8, 8.9, thetaRover);
Eigen::Vector2d P1(0, 0); //inital reading (must be taken from the IMU at start)
Eigen::Vector3d x_pred;
Eigen::Matrix3d P = Eigen::Matrix3d::Identity() * 0.1;

//Those two functions are used for predicting new position and orientation
float angularVelocity(float Vr, float Vl, float L){
    //w: angular velocity | Vr: right wheels velocity | Vl: left wheels velocity | L: Length between wheels
    float w = (Vr-Vl)/L;
    return w;
}
void newPosition(float L, float Vr, float Vl, float theta, float dt, float res[]){
  //L: Length between wheels | Vr: right wheels velocity | Vl: left wheels velocity | theta: Rover's orientation | dt: time step | res: result array
    float dx; //distance moved in X direction in this time step
    float dy; //distance moved in Y direction in this time step
    float thetaNew; //New orientation
    float Vavg = (Vr+Vl)/2;
    if (Vr == Vl){
      //no w "angular velocity" so we will resolute the average velocity in X&Y directions
      dx = Vavg * cos(theta) * dt;
      dy = Vavg * sin(theta) * dt;
      thetaNew = theta; //no change in orientation since no w "angular velocity"
    } else {
      float R = (L/2) * ((Vr+Vl)/(Vr-Vl)); //radius of the curvature that the rover turn upon
      thetaNew = theta + dt * angularVelocity(Vr, Vl, L);
      dx = R * (sin(thetaNew)-sin(theta));
      dy = R * (cos(thetaNew)-cos(theta));
    }
    thetaRover = thetaNew; //update the rover's orientation
    x(2) = thetaRover;
    res[0] = dx; //return the change in X position
    res[1] = dy; //return the change in Y position
    res[2] = thetaNew; //return the new orientation
}

//This function is used to calculate the initial velocity for the next time instant in the following manner
// Vfk = Vik + acck*dt
// Vik+1 = Vfk
float initialVelocity(float acc, float dt, float &Vi){
    Vi = Vi + acc*dt;
    return Vi;
}

//The following 3 functions are the nodes callbacks "IMU, GPS and Encoders"

void imuCallback(const sensor_msgs::Imu::ConstPtr& imuMsg) {
    //getting the linear acceleration in each direction
    float linearAccX = imuMsg->linear_acceleration.x;
    float linearAccy = imuMsg->linear_acceleration.y;
    //float linearAccz = imuMsg->linear_acceleration.z;
    const float dt = 0.05; // dt = 1/freq = 1/20
    Eigen::Quaterniond q(imuMsg->orientation.w, imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z); //getting the quaternions values
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);  // Yaw-Pitch-Roll //turn the quaternions to euler angles
    
    /*
    ROS_INFO("Linear Acceleration: x=%.3f, y=%.3f, z=%.3f", linearAccX, linearAccy, linearAccz);
    ROS_INFO("Yaw: %.3f, Pitch: %.3f, Roll: %.3f", euler[0], euler[1], euler[2]); //interested in yaw "Rotation around Z"
    */

    //calculating the distance travelled in each direction X&Y 
    // distance = Vi * t + 0.5 * a * t*t
    float deltaDistanceX = initialVelocity(linearAccX,dt,Vix)*dt + 0.5*linearAccX*dt*dt;
    float deltaDistanceY = initialVelocity(linearAccy,dt,Viy)*dt + 0.5*linearAccy*dt*dt;
    //ended here
    //you have the change in Positions X and Y

    ROS_INFO("IMU Readings: dX=%.3f | dY=%.3f", deltaDistanceX, deltaDistanceY);

    //update step
    /*
    x(0) = x(0) + deltaDistanceX;
    x(1) = x(1) + deltaDistanceY;
    */
   double z_pred = x_pred(2);
   double y = euler[0] - z_pred;  // Innovation (residual)

   // The measurement Jacobian for this measurement is:
   // H = [0 0 1]
   
   Eigen::Matrix<double, 1, 3> H;
   H << 0, 0, 1;

   // Compute the innovation covariance (scalar in this case):
   double S = (H * P * H.transpose())(0, 0);

   // Compute the Kalman Gain: K = P * H^T / S
   Eigen::Vector3d K = P * H.transpose() / S;

   // Update the state.
   x = x + K * y;

   // Update the covariance matrix.
   Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
   P = (I - K * H) * P;

  }
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gpsMsg) {
    ROS_INFO("GPS Data: Latitude: %.3f, Longitude: %.3f, Altitude: %.3f meters", gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);
    //you can get the new Positions Latitude and Longitude but convert them to X and Y

    //update step
    Eigen::Vector2d gpsReading(gpsMsg->latitude,gpsMsg->longitude);
    Eigen::Vector2d z_pred;
    z_pred(0) = x_pred(0);
    z_pred(1) = x_pred(1);
  
    // Compute the innovation (residual): the difference between actual and predicted measurement.
    Eigen::Vector2d y = gpsReading - z_pred;
  
      // The measurement Jacobian H is the derivative of h(x) with respect to x.
      // For h(x) = [x, y]^T, the Jacobian is:
      // H = [1 0 0; 0 1 0]
    Eigen::Matrix<double, 2, 3> H;
    H.setZero();
    H(0,0) = 1.0;
    H(1,1) = 1.0;
  
      // Compute the innovation covariance: S = H * P * H^T + R
    Eigen::Matrix2d S = H * P * H.transpose();
  
      // Compute the Kalman Gain: K = P * H^T * S^-1
    Eigen::Matrix<double, 3, 2> K = P * H.transpose() * S.inverse();
    x = x + K * y;
  
      // Update the covariance matrix.
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    P = (I - K * H) * P;

      }
void encoderCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    
    
    /*
    ROS_INFO("%s position %.3f", msg->name[0].c_str(), msg->position[0]); //RM
    ROS_INFO("%s position %.3f", msg->name[1].c_str(), msg->position[1]); //RF
    ROS_INFO("%s position %.3f", msg->name[2].c_str(), msg->position[2]); //RR
    ROS_INFO("%s position %.3f", msg->name[3].c_str(), msg->position[3]); //LM
    ROS_INFO("%s position %.3f", msg->name[4].c_str(), msg->position[4]); //LF
    ROS_INFO("%s position %.3f", msg->name[5].c_str(), msg->position[5]); //LR
    */
    //in real life I'll fuse each side encoders into 2 readings, one for each side
    //but since in simulation each side velocity is the same I'll use any encoder 
    //from each side and that will do the same work
    //the resulted position will be given to the vector P2 << right_side_position, left_side_position;
    Eigen::Vector2d P2(msg->position[0],msg->position[3]);//position right //position left
    Eigen::Vector2d deltaPosition = P2-P1;
    //the following is a correction step since the encoders' readings are from [-pi,pi]
    //so when moving from just before pi to just after -pi, this cyclic action makes an error in calculations
    //that must be corrected. Which is solved using the line below
    deltaPosition << atan2(sin(deltaPosition(0)),cos(deltaPosition(0))), atan2(sin(deltaPosition(1)),cos(deltaPosition(1)));
    P1 = P2; //update the state for the next callback

    Eigen::Vector2d deltaDistance = deltaPosition * 0.1; //where 0.1m is the wheel radius
    Eigen::Vector2d deltaVelocity = deltaDistance / (0.1);
    float res[3]; //dx dy thetaNew
    newPosition(L, deltaVelocity(0), deltaVelocity(1), thetaRover, 0.1, res);
    //Ended here
    //You can get the New Theta and the change in Positions X and Y for "res" array
    
    ROS_INFO("Encoders Readings: dX=%.3f | dY=%.3f | w=%.3f", res[0], res[1], res[2]);

    //update step
    Eigen::Vector3d encoderReading(res[0],res[1],res[2]);
    Eigen::Vector3d z_pred;
    z_pred(0) = x_pred(0);
    z_pred(1) = x_pred(1);
    z_pred(2) = x_pred(2);

    // Compute the innovation (residual): the difference between actual and predicted measurement.
    Eigen::Vector3d y = encoderReading - z_pred;

    // The measurement Jacobian H is the derivative of h(x) with respect to x.
    // For h(x) = [x, y]^T, the Jacobian is:
    // H = [1 0 0; 0 1 0]
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity();

    // Compute the innovation covariance: S = H * P * H^T + R
    Eigen::Matrix3d S = H * P * H.transpose() + Eigen::Matrix3d::Identity()*0.1;

    // Compute the Kalman Gain: K = P * H^T * S^-1
    Eigen::Matrix3d K = P * H.transpose() * S.inverse();
    x = x + K * y;

    // Update the covariance matrix.
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    P = (I - K * H) * P;
}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "ekf");
    
    ros::NodeHandle nhIMU;
    ros::NodeHandle nhGPS;
    ros::NodeHandle nhENCODER;
  
    ros::Subscriber imuSub = nhIMU.subscribe("imu", 1000, imuCallback);
    ros::Subscriber gpsSub = nhGPS.subscribe("gps", 1000, gpsCallback);
    ros::Subscriber encoderSub = nhENCODER.subscribe("joint_states", 1000, encoderCallback);

    //here subscribe to the controller and make the prediction step
    //and at the end of each callback make the update step
    //and of course the EKF class will be above all
    
    ros::Rate rate(20);
    while (ros::ok()){
      float Vr = 1.0;
      float Vl = 1.0;
      float theta = thetaRover;
      float dt = 0.05;
      float dx; //distance moved in X direction in this time step
      float dy; //distance moved in Y direction in this time step
      float thetaNew; //New orientation
      float Vavg = (Vr+Vl)/2;
      if (Vr == Vl){
        //no w "angular velocity" so we will resolute the average velocity in X&Y directions
        dx = Vavg * cos(theta) * dt;
        dy = Vavg * sin(theta) * dt;
        thetaNew = theta; //no change in orientation since no w "angular velocity"
      } else {
        float R = (L/2) * ((Vr+Vl)/(Vr-Vl)); //radius of the curvature that the rover turn upon
        thetaNew = theta + dt * ((Vr-Vl)/L);
        dx = R * (sin(thetaNew)-sin(theta));
        dy = R * (cos(thetaNew)-cos(theta));
      }
      x_pred(0) = x(0) + dx;
      x_pred(1) = x(1) + dy;
      x_pred(2) = thetaNew;
      Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
      P = F * P * F.transpose();
      std::cout << "***************\n" << x << "\n";
      ros::spinOnce();
      rate.sleep();
    }
    
    return 0;
}