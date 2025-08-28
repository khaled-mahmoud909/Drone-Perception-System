#include <ros/ros.h>
#include <thread> // For std::thread
#include <nav_msgs/Odometry.h>
#include <map_manager/dynamicMap.h>
#include <global_planner/dep.h>
#include <nav_msgs/Path.h>

std::shared_ptr<mapManager::dynamicMap> map_ptr;
std::shared_ptr<globalPlanner::DEP> planner_ptr;
ros::Publisher path_pub;
bool odom_received = false;

void odomCheckCB(const nav_msgs::Odometry::ConstPtr& msg){
    odom_received = true;
}

// The main planning loop
void planning_loop(){
    ros::Rate rate(1.0); // Plan at 1 Hz, can be adjusted

    while(ros::ok() && !odom_received) {
        ROS_INFO("Waiting for odometry before starting planner...");
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Odometry received. Starting DEP planning loop.");

    while(ros::ok()){
        // Set the current map for the planner
        planner_ptr->setMap(map_ptr);

        ROS_INFO("[Planner]: Starting a new planning cycle.");
        ros::Time startTime = ros::Time::now();

        bool plan_success = planner_ptr->makePlan();

        ros::Time endTime = ros::Time::now();
        ROS_INFO("[Planner]: Planning cycle finished in %.4f seconds.", (endTime - startTime).toSec());

        if(plan_success){
            nav_msgs::Path best_path = planner_ptr->getBestPath();
            best_path.header.stamp = ros::Time::now();
            best_path.header.frame_id = "map";
            path_pub.publish(best_path);
            ROS_INFO("[Planner]: Found and published a new path with %ld poses.", best_path.poses.size());
        } else {
            ROS_WARN("[Planner]: Failed to find a valid exploration path in this cycle.");
        }

        rate.sleep(); // Wait for the next planning cycle
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "exploration_planner_node");
    ros::NodeHandle nh;

    // Initialize the map manager
    map_ptr.reset(new mapManager::dynamicMap());
    map_ptr->initMap(nh);

    // Initialize the Dynamic Exploration Planner (DEP)
    planner_ptr.reset(new globalPlanner::DEP(nh));

    // Publisher for the final path
    path_pub = nh.advertise<nav_msgs::Path>("/planned_path", 10);

    // Subscriber to check for odometry
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCheckCB);

    // Start the planning loop in a separate thread
    std::thread planning_thread(planning_loop);

    ros::spin(); // Keep the main thread alive for callbacks

    planning_thread.join(); // Cleanly exit thread

    return 0;
}
