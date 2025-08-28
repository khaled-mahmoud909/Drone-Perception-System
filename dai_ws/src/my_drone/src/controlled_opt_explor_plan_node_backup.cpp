#include <ros/ros.h>
#include <thread>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include <map_manager/dynamicMap.h>
#include <global_planner/dep.h>
#include <trajectory_planner/polyTrajOccMap.h> 
#include <tracking_controller/Target.h>

class ControlledOptExplorPlanNode {
public:
    ControlledOptExplorPlanNode(ros::NodeHandle& nh) : nh_(nh) {
        // Initialize components
        map_ptr_ = std::make_shared<mapManager::dynamicMap>(nh);
        global_planner_ptr_ = std::make_shared<globalPlanner::DEP>(nh);
        // *** CHANGE: Use polyTrajOccMap instead of bsplineTraj ***
        trajectory_planner_ptr_ = std::make_shared<trajPlanner::polyTrajOccMap>(nh);

        // Set dependencies
        global_planner_ptr_->setMap(map_ptr_);
        trajectory_planner_ptr_->setMap(map_ptr_); // This planner also needs the map
        
        // Publishers
	target_pub_ = nh_.advertise<tracking_controller::Target>("/autonomous_flight/target_state", 10);
        final_trajectory_pub_ = nh_.advertise<nav_msgs::Path>("/final_trajectory", 10);
        global_path_pub_ = nh_.advertise<nav_msgs::Path>("/global_path", 10);
        
        // Subscriber
        odom_sub_ = nh.subscribe("/odom", 1, &ControlledOptExplorPlanNode::odomCallback, this);
        
        ROS_INFO("Exploration Planner Node initialized. Waiting for dependencies...");
    }

    void run() {
        planning_thread_ = std::thread(&ControlledOptExplorPlanNode::planningLoop, this);
        ros::spin();
        planning_thread_.join();
    }

private:
    ros::NodeHandle nh_;
    std::shared_ptr<mapManager::dynamicMap> map_ptr_;
    std::shared_ptr<globalPlanner::DEP> global_planner_ptr_;
    std::shared_ptr<trajPlanner::polyTrajOccMap> trajectory_planner_ptr_;

    ros::Publisher target_pub_;
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
		 executeTrajectory();

            } else {
                 ROS_WARN("[Planner]: >>> FAILED: Polynomial trajectory generation failed. <<<");
            }
            
            planning_rate.sleep();
        }
    }

    void executeTrajectory() {
        double duration = trajectory_planner_ptr_->getDuration();
        if (duration < 0.1) return; // Ignore very short trajectories
        
        ROS_INFO("[Executor]: Executing new trajectory (Duration: %.2fs).", duration);
        ros::Rate execution_rate(50.0); // 50Hz control rate
        ros::Time start_time = ros::Time::now();
        double t = 0.0;

        while (t <= duration && ros::ok()) {
            t = (ros::Time::now() - start_time).toSec();
            
            // Get the desired state from the polynomial trajectory
            Eigen::Vector3d pos = trajectory_planner_ptr_->getPos(t);
            Eigen::Vector3d vel = trajectory_planner_ptr_->getVel(t);
            Eigen::Vector3d acc = trajectory_planner_ptr_->getAcc(t);
            double yaw = atan2(vel.y(), vel.x());

            // Create the Target message for the controller
            tracking_controller::Target target_msg;
            target_msg.header.stamp = ros::Time::now();
            target_msg.header.frame_id = "map";
            
            target_msg.position.x = pos.x();
            target_msg.position.y = pos.y();
            target_msg.position.z = pos.z();

            target_msg.velocity.x = vel.x();
            target_msg.velocity.y = vel.y();
            target_msg.velocity.z = vel.z();
            
            target_msg.acceleration.x = acc.x();
            target_msg.acceleration.y = acc.y();
            target_msg.acceleration.z = acc.z();

            target_msg.yaw = yaw;

            target_pub_.publish(target_msg);

            execution_rate.sleep();
        }
        ROS_INFO("[Executor]: Trajectory finished.");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "controlled_opt_explor_plan_node");
    ros::NodeHandle nh;
    
    ControlledOptExplorPlanNode planner_node(nh);
    planner_node.run();
    
    return 0;
}
