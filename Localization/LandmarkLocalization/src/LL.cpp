#include "ros/ros.h"
#include "roar_msgs/Landmark.h"
#include "roar_msgs/LandmarkArray.h"
#include "nav_msgs/Odometry.h"
#include <vector>

class ROV{

  public:
    float x, y, z;
    std::vector<roar_msgs::Landmark> ROVlandmarks;

};

ROV rov;

void landmarkCallback(const roar_msgs::Landmark::ConstPtr& landmark_poses) {

    //Calculate the rover position
    //Rover's position: P
	  //Landmark TRUE position: rov.landmarks
	  //Landmark RELATIVE position: landmark_poses

    //we should put the landmark id instead of the 0
    float P[3] = {
      rov.ROVlandmarks[landmark_poses->id].pose.pose.position.x - landmark_poses->pose.pose.position.x,
      rov.ROVlandmarks[landmark_poses->id].pose.pose.position.y - landmark_poses->pose.pose.position.y,
      rov.ROVlandmarks[landmark_poses->id].pose.pose.position.z - landmark_poses->pose.pose.position.z
    };
    ROS_INFO("The Rovers True Position: x: %f | y: %f | z: %f", P[0] ,P[1] ,P[2]);

  }

void roverCallback(const nav_msgs::Odometry::ConstPtr& Odometry) {

    //update rovers pose
    rov.x = Odometry->pose.pose.position.x;
    rov.y = Odometry->pose.pose.position.y;
    rov.z = Odometry->pose.pose.position.z;
    ROS_INFO("[+] Recieved the rover position!");

  }

void trueLandmarkCallback(const roar_msgs::LandmarkArray::ConstPtr& msg){

  //Set the landmarks
  rov.ROVlandmarks = msg->landmarks;
  ROS_INFO("\n[+] Recieved the Landmarks!\n");
  std::cout << msg->landmarks[0].id << std::endl;

}


int main(int argc, char **argv) {
    
    ros::init(argc, argv, "LandmarkLocalization");
    
    ros::NodeHandle landmarkPoses; //node to update the pose
    ros::NodeHandle landmarkTruePoses; //node to set the true poses
    ros::NodeHandle roverPose; //node to get rover pose "The estimated"
  
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