#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <racecar_tools/path_generator.h>
#include <ros/console.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "generate_path_node");
    ros::NodeHandle nh;    
    std::vector<std::string> topics;

    if (nh.getParam("topics", topics)) {
        ROS_INFO("Got topics");
        ros::Rate loop_rate(10);
        std::vector<PathGenerator*> paths;
        for (int i = 0; i < topics.size(); i++) {
            paths.push_back(new PathGenerator(topics[i]));
        }
        ROS_INFO("Number of paths: %lu", paths.size());


        while(ros::ok()) {
            for (int i = 0; i < paths.size(); i++) {
                paths[i]->publish();
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
}

    
