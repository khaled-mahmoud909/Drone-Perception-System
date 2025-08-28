#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

class GridmapGenerator {
public:
    GridmapGenerator(ros::NodeHandle& nh) {
        // Get parameters from the parameter server
        // These should match the parameters in your map_manager config
        std::vector<double> map_size;
        nh.param<double>("map_resolution", resolution_, 0.1);
        nh.param<std::vector<double>>("map_size", map_size, {20.0, 20.0, 4.0});
        nh.param<std::string>("input_topic", input_topic_, "/dynamic_map/inflated_voxel_map");
        nh.param<std::string>("output_topic", output_topic_, "/generated_2d_map");

        // Set up the OccupancyGrid message structure. This is done once.
        grid_map_.header.frame_id = "map";
        grid_map_.info.resolution = resolution_;
        
        // Calculate grid dimensions
        grid_map_.info.width = static_cast<unsigned int>(map_size[0] / resolution_);
        grid_map_.info.height = static_cast<unsigned int>(map_size[1] / resolution_);

        // Set the origin of the map (bottom-left corner)
        grid_map_.info.origin.position.x = -map_size[0] / 2.0;
        grid_map_.info.origin.position.y = -map_size[1] / 2.0;
        grid_map_.info.origin.position.z = 0.0;
        grid_map_.info.origin.orientation.w = 1.0;

        // Initialize the map data with "unknown"
        grid_map_.data.assign(grid_map_.info.width * grid_map_.info.height, -1); // -1 for unknown

        ROS_INFO("Gridmap Generator initialized.");
        ROS_INFO("Map dimensions: %u x %u, Resolution: %.2f", grid_map_.info.width, grid_map_.info.height, resolution_);

        // Set up subscriber and publisher
        voxel_sub_ = nh.subscribe(input_topic_, 1, &GridmapGenerator::voxelCallback, this);
        map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>(output_topic_, 1, true); // Latched publisher
    }

private:
    void voxelCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        // Reset the map data to unknown before processing the new cloud
        // Note: For a persistent map, you would only clear free space, not reset everything.
        // But for this use case, we assume the input cloud is the complete current obstacle set.
        grid_map_.data.assign(grid_map_.info.width * grid_map_.info.height, -1);

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*cloud_msg, pcl_cloud);

        for (const auto& point : pcl_cloud.points) {
            // Convert world coordinates to map coordinates
            int map_x = static_cast<int>((point.x - grid_map_.info.origin.position.x) / resolution_);
            int map_y = static_cast<int>((point.y - grid_map_.info.origin.position.y) / resolution_);

            // Check if the point is within the map bounds
            if (map_x >= 0 && map_x < grid_map_.info.width && map_y >= 0 && map_y < grid_map_.info.height) {
                // Convert 2D map coordinates to 1D array index
                int index = map_y * grid_map_.info.width + map_x;
                grid_map_.data[index] = 100; // 100 for occupied
            }
        }

        // We don't have sensor data to clear free space, so we leave non-obstacle areas as "unknown"
        // A more advanced version could use the /dynamic_map/explored_voxel_map to mark free space (0)

        grid_map_.header.stamp = ros::Time::now();
        map_pub_.publish(grid_map_);
    }

    ros::Subscriber voxel_sub_;
    ros::Publisher map_pub_;
    nav_msgs::OccupancyGrid grid_map_;
    double resolution_;
    std::string input_topic_;
    std::string output_topic_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gridmap_generator_node");
    ros::NodeHandle nh("~"); // Use private node handle to get parameters

    GridmapGenerator generator(nh);

    ros::spin();
    return 0;
}
