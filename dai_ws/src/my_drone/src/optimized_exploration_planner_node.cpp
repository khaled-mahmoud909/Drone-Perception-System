#include <ros/ros.h>
#include <thread>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include <map_manager/dynamicMap.h>
#include <global_planner/dep.h>
// Use the Polynomial Trajectory planner instead of B-Spline
#include <trajectory_planner/polyTrajOccMap.h> 

class OptimizedExplorationPlannerNode {
public:
    OptimizedExplorationPlannerNode(ros::NodeHandle& nh) : nh_(nh) {
        // Initialize components
        map_ptr_ = std::make_shared<mapManager::dynamicMap>(nh);
        global_planner_ptr_ = std::make_shared<globalPlanner::DEP>(nh);
        // *** CHANGE: Use polyTrajOccMap instead of bsplineTraj ***
        trajectory_planner_ptr_ = std::make_shared<trajPlanner::polyTrajOccMap>(nh);

        // Set dependencies
        global_planner_ptr_->setMap(map_ptr_);
        trajectory_planner_ptr_->setMap(map_ptr_); // This planner also needs the map
        
        // Publishers
        final_trajectory_pub_ = nh_.advertise<nav_msgs::Path>("/final_trajectory", 10);
        global_path_pub_ = nh_.advertise<nav_msgs::Path>("/global_path", 10);
        
        // Subscriber
        odom_sub_ = nh.subscribe("/odom", 1, &OptimizedExplorationPlannerNode::odomCallback, this);
        
        ROS_INFO("Exploration Planner Node initialized. Waiting for dependencies...");
    }

    void run() {
        planning_thread_ = std::thread(&OptimizedExplorationPlannerNode::planningLoop, this);
        ros::spin();
        planning_thread_.join();
    }

private:
    ros::NodeHandle nh_;
    std::shared_ptr<mapManager::dynamicMap> map_ptr_;
    std::shared_ptr<globalPlanner::DEP> global_planner_ptr_;
    // *** CHANGE: Use polyTrajOccMap instead of bsplineTraj ***
    std::shared_ptr<trajPlanner::polyTrajOccMap> trajectory_planner_ptr_;

    ros::Publisher final_trajectory_pub_;
    ros::Publisher global_path_pub_;
    ros::Subscriber odom_sub_;
    std::thread planning_thread_;
    bool odom_received_ = false;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!odom_received_) {
            ROS_INFO("Odometry received. Planning can now start.");
        }
        odom_received_ = true;
    }

    void planningLoop() {
        ros::Rate planning_rate(0.5);

        while (ros::ok() && (!odom_received_ || !(map_ptr_ && map_ptr_->getRes() > 0.0))) {
            ROS_INFO_THROTTLE(5, "Waiting for odometry and map initialization...");
            ros::Duration(1.0).sleep();
        }
        ROS_INFO("Dependencies met. Starting autonomous exploration planning.");

        while (ros::ok()) {
            ROS_INFO_STREAM("----------------- New Planning Cycle -----------------");

            // 1. GLOBAL PLANNING
            if (!global_planner_ptr_->makePlan()) {
                ROS_WARN("[Planner]: Global planner could not find a frontier path.");
                planning_rate.sleep();
                continue;
            }
            nav_msgs::Path rough_path = global_planner_ptr_->getBestPath();
            if (rough_path.poses.size() < 2) {
                ROS_INFO("[Planner]: Exploration complete or no valid path found.");
                planning_rate.sleep();
                continue;
            }
            rough_path.header.stamp = ros::Time::now();
            rough_path.header.frame_id = "map";
            global_path_pub_.publish(rough_path);
            ROS_INFO("[Planner]: Found global path with %ld poses.", rough_path.poses.size());

            // 2. TRAJECTORY GENERATION (Minimum Snap Polynomial)
            ROS_INFO("[Planner]: Generating a smooth polynomial trajectory...");
            trajectory_planner_ptr_->updatePath(rough_path); // Update the path
            
            // Generate the trajectory. The 'true' argument enables corridor constraints for safety.
            nav_msgs::Path final_trajectory;
            if (trajectory_planner_ptr_->makePlan(final_trajectory, true)) {
                 final_trajectory_pub_.publish(final_trajectory);
                 ROS_INFO("[Planner]: >>> SUCCESS: Generated and published a new trajectory! <<<");
            } else {
                 ROS_WARN("[Planner]: >>> FAILED: Polynomial trajectory generation failed. <<<");
            }
            
            planning_rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "optimized_exploration_planner_node");
    ros::NodeHandle nh;
    
    OptimizedExplorationPlannerNode planner_node(nh);
    planner_node.run();
    
    return 0;
}
