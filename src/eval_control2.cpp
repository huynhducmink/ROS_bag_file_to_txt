#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <string>
#include <iostream>
#include <fstream>

#include <boost/foreach.hpp>


int main(int argc, char ** argv){
    std::string bag_file_name;
    std::cout << "This parse the bag file content to .txt file" << std::endl;
    std::cout << "Support the following topic" << std::endl
        << "/controller_setpoint for optimized setpoints from ewok (output from offboard controller)" << std::endl
        << "/gazebo_groundtruth_posestamped for real pose of drone (output from Gazebo API (read_topic package))" << std::endl
        << "/vio_odo_posestamped for pose estimation output (from read_topic package)" << std::endl
        << "/global_trajectory (preset trajectory (from ewok))" << std::endl;
    std::cout << "Input bag file name (include .bag extension)" << std::endl;
    std::getline(std::cin, bag_file_name);

    std::ofstream optimized_traj_file, real_traj_file, estimate_traj_file, preset_traj_file;
    optimized_traj_file.open("./optimized_traj.txt");
    optimized_traj_file << "# timestamp tx ty tz qx qy qz qw\n";
    real_traj_file.open("./real_traj.txt");
    real_traj_file << "# timestamp tx ty tz qx qy qz qw\n";
    estimate_traj_file.open("./estimate_traj.txt");
    estimate_traj_file << "# timestamp tx ty tz qx qy qz qw\n";
    preset_traj_file.open("./preset_traj.txt");
    preset_traj_file << "# timestamp tx ty tz qx qy qz qw\n";

    ros::init(argc, argv, "eval_control2");
    ros::NodeHandle nh;

    rosbag::Bag bag;
    bag.open(bag_file_name);

    std::vector<std::string> topics;
    topics.push_back(std::string("/controller_setpoint"));
    topics.push_back(std::string("/gazebo_groundtruth_posestamped"));
    topics.push_back(std::string("/vio_odo_posestamped"));
    topics.push_back(std::string("/global_trajectory"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    BOOST_FOREACH (rosbag::MessageInstance const m, view)
    {

        // /controller_setpoint
        geometry_msgs::PoseStamped::ConstPtr p_optimized_traj_msg = m.instantiate<geometry_msgs::PoseStamped>();
        if(p_optimized_traj_msg!=nullptr){
            optimized_traj_file << ros::Time::now() << " " 
            << p_optimized_traj_msg->pose.position.x << " " << p_optimized_traj_msg->pose.position.y << " " << p_optimized_traj_msg->pose.position.z << " " 
            << p_optimized_traj_msg->pose.orientation.x << " " << p_optimized_traj_msg->pose.orientation.y << " " << p_optimized_traj_msg->pose.orientation.z << " " << p_optimized_traj_msg->pose.orientation.w << std::endl;
        }

        // /gazebo_groundtruth_posestamped topic
        geometry_msgs::PoseStamped::ConstPtr p_real_traj_msg = m.instantiate<geometry_msgs::PoseStamped>();
        if(p_real_traj_msg!=nullptr){
            real_traj_file << ros::Time::now() << " " 
            << p_real_traj_msg->pose.position.x << " " << p_real_traj_msg->pose.position.y << " " << p_real_traj_msg->pose.position.z << " " 
            << p_real_traj_msg->pose.orientation.x << " " << p_real_traj_msg->pose.orientation.y << " " << p_real_traj_msg->pose.orientation.z << " " << p_real_traj_msg->pose.orientation.w << std::endl;
        }

        // /vio_odo_posestamped topic
        geometry_msgs::PoseStamped::ConstPtr p_estimate_traj_msg = m.instantiate<geometry_msgs::PoseStamped>();
        if(p_estimate_traj_msg!=nullptr){
            estimate_traj_file << ros::Time::now() << " " 
            << p_estimate_traj_msg->pose.position.x << " " << p_estimate_traj_msg->pose.position.y << " " << p_estimate_traj_msg->pose.position.z << " " 
            << p_estimate_traj_msg->pose.orientation.x << " " << p_estimate_traj_msg->pose.orientation.y << " " << p_estimate_traj_msg->pose.orientation.z << " " << p_estimate_traj_msg->pose.orientation.w << std::endl;
        }

        // /global_trajectory topic
        visualization_msgs::MarkerArray::ConstPtr p_preset_traj_msg = m.instantiate<visualization_msgs::MarkerArray>();
        if (p_preset_traj_msg!=nullptr) {
            for (auto i = p_preset_traj_msg->markers.begin(); i < p_preset_traj_msg->markers.end(); i++){
                for (auto j = (*i).points.begin(); j < (*i).points.end(); j++){
                    preset_traj_file << ros::Time::now() << " " << (*j).x << " " << (*j).y << " " << (*j).z << " " << "0" <<  " " << "0" <<  " " << "0" <<  " " << "0" << std::endl; 
                }
            }
        }
    }
    std::cout << "Finish parsing data" << std::endl;
    return 0;
}