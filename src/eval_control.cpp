#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"

#include <string>
#include <iostream>
#include <fstream>

std::ofstream gazebo_file, vio_file, preset_traj_file;
bool take_one = true;

void gazebo_callback(const geometry_msgs::PoseStampedConstPtr& msg){
    gazebo_file << ros::Time::now()<< " " << msg->pose.position.x << " " << msg->pose.position.y << " " << msg->pose.position.z
     << " " << msg->pose.orientation.x << " " << msg->pose.orientation.y << " " << msg->pose.orientation.z << " " << msg->pose.orientation.w << std::endl;
}

void vio_callback(const geometry_msgs::PoseStampedConstPtr& msg){
    vio_file << ros::Time::now()<< " " << msg->pose.position.x << " " << msg->pose.position.y << " " << msg->pose.position.z
     << " " << msg->pose.orientation.x << " " << msg->pose.orientation.y << " " << msg->pose.orientation.z << " " << msg->pose.orientation.w << std::endl;
}
void preset_traj_callback(const visualization_msgs::MarkerArray msg){
    std::cout << "receive" <<std::endl;
    for (int i = 0; i < sizeof(msg.markers); i++){
        std::cout << "--------------------------------" << std::endl;
        std::cout << sizeof(msg.markers) << std::endl;
        std::cout << "--------------------------------" << std::endl;
        for (int j = 0; j < sizeof(msg.markers[i].points); j++){
            std::cout << sizeof(msg.markers[i].points) << std::endl;
            geometry_msgs::Point point_msg = msg.markers[i].points[j];
            preset_traj_file << ros::Time::now() << " " << point_msg.x << " " << point_msg.y << " " << point_msg.z << " " << 0 <<  " " << 0 <<  " " << 0 <<  " " << 1 << std::endl; 
        }
    }
}


int main(int argc, char ** argv){
    gazebo_file.open("./stamped_groundtruth.txt");
    gazebo_file << "# timestamp tx ty tz qx qy qz qw\n";
    vio_file.open("./stamped_traj_estimate.txt");
    vio_file << "# timestamp tx ty tz qx qy qz qw\n";
    preset_traj_file.open("./preset_traj.txt");
    preset_traj_file << "# timestamp tx ty tz qx qy qz qw\n";

    ros::init(argc, argv, "eval_vio");
    ros::NodeHandle nh;
    ros::Subscriber sub_gazebo = nh.subscribe("/gazebo_groundtruth_posestamped", 10, gazebo_callback);
    ros::Subscriber sub_vio = nh.subscribe("/controller_setpoint", 10, vio_callback);
    ros::Subscriber sub_preset_traj = nh.subscribe("/global_trajectory", 10, preset_traj_callback);

    ros::spin();
    return 0;
}