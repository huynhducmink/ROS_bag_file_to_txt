#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#include <string>
#include <iostream>
#include <fstream>

std::ofstream gazebo_file, vio_file;

void gazebo_callback(const geometry_msgs::PoseStampedConstPtr& msg){
    gazebo_file << ros::Time::now()<< " " << msg->pose.position.x << " " << msg->pose.position.y << " " << msg->pose.position.z
     << " " << msg->pose.orientation.x << " " << msg->pose.orientation.y << " " << msg->pose.orientation.z << " " << msg->pose.orientation.w << std::endl;
}

void vio_callback(const geometry_msgs::PoseStampedConstPtr& msg){
    vio_file << ros::Time::now()<< " " << msg->pose.position.x << " " << msg->pose.position.y << " " << msg->pose.position.z
     << " " << msg->pose.orientation.x << " " << msg->pose.orientation.y << " " << msg->pose.orientation.z << " " << msg->pose.orientation.w << std::endl;
}

int main(int argc, char ** argv){
    gazebo_file.open("./stamped_groundtruth.txt");
    gazebo_file << "# timestamp tx ty tz qx qy qz qw\n";
    vio_file.open("./stamped_traj_estimate.txt");
    vio_file << "# timestamp tx ty tz qx qy qz qw\n";

    ros::init(argc, argv, "eval_vio");
    ros::NodeHandle nh;
    ros::Subscriber sub_gazebo = nh.subscribe("/gazebo_groundtruth_posestamped", 10, gazebo_callback);
    ros::Subscriber sub_vio = nh.subscribe("/vio_odo_posestamped", 10, vio_callback);

    ros::spin();
    return 0;
}