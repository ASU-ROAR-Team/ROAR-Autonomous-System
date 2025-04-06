#include "ros/ros.h"
#include "ROAR-Msgs-main/landmark"
#include "ROAR-Msgs-main/LandmarksArray"
#include "nav_msgs/Odometry"

class ROV{

    public:
    float x, y, z;
    float landmarks[15][3];

};

ROV rov;

void landmarkCallback(const roar_msgs::landmark_poses::ConstPtr& landmark_poses) {

    //Calculate the rover position
    //Rover's position: P
	//Landmark TRUE position: rov.landmarks
	//Landmark RELATIVE position: landmark_poses

    //we should put the landmark id instead of the 0
    float P[3] = [rov.landmarks[0] - landmark_poses.x, rov.landmarks[0] - landmark_poses.y, rov.landmarks[0] - landmark_poses.z];

  }

void roverCallback(const nav_msgs::Odometry::ConstPtr& Odometry) {

    //update rovers pose
    rov.x = Odometry->pose.pose.position.x;
    rov.y = Odometry->pose.pose.position.y;
    rov.z = Odometry->pose.pose.position.z;

  }

void trueLandmarkCallback(const roar_msgs::LandmarksArray::constPtr& LandmarksArray){

    //Set the landmarks
    rov.landmarks = LandmarksArray;

}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "ekf");
    
    ros::NodeHandle landmarkPoses;
    ros::NodeHandle landmarkTruePoses;
    ros::NodeHandle roverPose;
  
    ros::Subscriber landmarkSub = landmarkPoses.subscribe("landmarkPoses", 1000, landmarkCallback);
    ros::Subscriber trueLandmarkSub = landmarkTruePoses.subscribe("landmarkPoses", 1000, trueLandmarkCallback);
    ros::Subscriber roverSub = roverPose.subscribe("Landmarks", 1000, roverCallback);
    
    ros::Rate rate(33);
    while (ros::ok()){
      ros::spinOnce();
      rate.sleep();
    }
    
    return 0;
}