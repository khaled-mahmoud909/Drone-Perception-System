#include <ros/ros.h>
// #include <thread>      // *** REMOVED ***
// #include <atomic>      // *** REMOVED ***
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Trigger.h>

#include <map_manager/dynamicMap.h>
#include <global_planner/dep.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <tracking_controller/Target.h>

class ControlledOptExplorPlanNode {
public:
    ControlledOptExplorPlanNode(ros::NodeHandle& nh) : nh_(nh), exploration_active_(false), odom_received_(false) {
        // Initialize components
        map_ptr_ = std::make_shared<mapManager::dynamicMap>(nh);
        global_planner_ptr_ = std::make_shared<globalPlanner::DEP>(nh);
        trajectory_planner_ptr_ = std::make_shared<trajPlanner::polyTrajOccMap>(nh);

        // Set dependencies
        global_planner_ptr_->setMap(map_ptr_);
        trajectory_planner_ptr_->setMap(map_ptr_);

        // Publishers
        target_pub_ = nh_.advertise<tracking_controller::Target>("/autonomous_flight/target_state", 10);
        final_trajectory_pub_ = nh_.advertise<nav_msgs::Path>("/final_trajectory", 10);
        global_path_pub_ = nh_.advertise<nav_msgs::Path>("/global_path", 10);

        // Subscriber
        odom_sub_ = nh.subscribe("/odom", 1, &ControlledOptExplorPlanNode::odomCallback, this);
        
        // Service server
        start_exploration_srv_ = nh_.advertiseService("/my_drone/start_exploration_command", &ControlledOptExplorPlanNode::startExplorationCallback, this);

        ROS_INFO("Exploration Planner Node initialized.");
    }

    // *** MODIFIED: The main run loop, now single-threaded ***
    void run() {
        ros::Rate loop_rate(10.0); // Rate to process callbacks during waits

        // 1. Wait for odometry and map to be ready
        ROS_INFO("Waiting for odometry and map initialization...");
        while (ros::ok() && (!odom_received_ || !(map_ptr_ && map_ptr_->getRes() > 0.0))) {
            ros::spinOnce(); // Process incoming messages (like odom)
            loop_rate.sleep();
        }

        // 2. Wait for the start exploration command
        ROS_INFO("Planner is ready. Waiting for the '/my_drone/start_exploration_command' service call...");
        while(ros::ok() && !exploration_active_) {
            ros::spinOnce(); // Process incoming messages (like the service call)
            loop_rate.sleep();
        }
        
        ROS_INFO("Start signal received! Starting autonomous exploration planning.");
        ros::Rate planning_rate(0.5);

        // 3. Main exploration planning loop
        while (ros::ok()) {
            ROS_INFO_STREAM("----------------- New Planning Cycle -----------------");

            if (!global_planner_ptr_->makePlan()) {
                ROS_WARN("[Planner]: Global planner could not find a frontier path.");
                planning_rate.sleep();
                ros::spinOnce();
                continue;
            }
            nav_msgs::Path rough_path = global_planner_ptr_->getBestPath();
            if (rough_path.poses.size() < 2) {
                ROS_INFO("[Planner]: Exploration complete or no valid path found.");
                break; // Exit the loop
            }
            rough_path.header.stamp = ros::Time::now();
            rough_path.header.frame_id = "map";
            global_path_pub_.publish(rough_path);
            ROS_INFO("[Planner]: Found global path with %ld poses.", rough_path.poses.size());

            ROS_INFO("[Planner]: Generating a smooth polynomial trajectory...");
            trajectory_planner_ptr_->updatePath(rough_path);

            nav_msgs::Path final_trajectory;
            if (trajectory_planner_ptr_->makePlan(final_trajectory, true)) {
                 final_trajectory_pub_.publish(final_trajectory);
                 ROS_INFO("[Planner]: >>> SUCCESS: Generated and published a new trajectory! <<<");
                 executeTrajectory();
            } else {
                 ROS_WARN("[Planner]: >>> FAILED: Polynomial trajectory generation failed. <<<");
            }
            
            ros::spinOnce(); // Process any other callbacks
            planning_rate.sleep();
        }

        ROS_INFO("Exploration loop has finished.");
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
    ros::ServiceServer start_exploration_srv_;

    bool odom_received_;
    bool exploration_active_; // No longer needs to be atomic

    bool startExplorationCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
        if (exploration_active_) {
            res.success = false;
            res.message = "Exploration is already active.";
        } else {
            exploration_active_ = true; // Just set the flag
            res.success = true;
            res.message = "Exploration initiated.";
            ROS_INFO(">>> Start Exploration Command Received via service call. <<<");
        }
        return true;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!odom_received_) {
            ROS_INFO("Odometry received.");
        }
        odom_received_ = true;
    }

    void executeTrajectory() {
        double duration = trajectory_planner_ptr_->getDuration();
        if (duration < 0.1) return;

        ROS_INFO("[Executor]: Executing new trajectory (Duration: %.2fs).", duration);
        ros::Rate execution_rate(50.0);
        ros::Time start_time = ros::Time::now();
        double t = 0.0;

        while (t <= duration && ros::ok()) {
            t = (ros::Time::now() - start_time).toSec();
            
            Eigen::Vector3d pos = trajectory_planner_ptr_->getPos(t);
            Eigen::Vector3d vel = trajectory_planner_ptr_->getVel(t);
            Eigen::Vector3d acc = trajectory_planner_ptr_->getAcc(t);
            double yaw = atan2(vel.y(), vel.x());

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
            
            ros::spinOnce(); // Allow callbacks to be processed during execution
            execution_rate.sleep();
        }
        ROS_INFO("[Executor]: Trajectory finished.");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "controlled_opt_explor_plan_node");
    ros::NodeHandle nh;

    ControlledOptExplorPlanNode planner_node(nh);
    planner_node.run(); // This now blocks and contains the main loop

    return 0;
}
