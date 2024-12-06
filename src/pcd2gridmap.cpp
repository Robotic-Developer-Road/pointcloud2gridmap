/*
Pcd2GridMap (Author: sujit-168)
Convert a .pcd file to a gridmap and publish it as a nav_msgs::OccupancyGrid message.
The set of points in the specified z-coordinate range (min_z~max_z)
into a 2D map with the specified resolution.
*/

#include "rclcpp/rclcpp.hpp"
#include "pcd2grid/pcd2gridmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>         //条件滤波器头文件
#include <pcl/filters/passthrough.h>                 //直通滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>      //半径滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h> //统计滤波器头文件
#include <pcl/filters/voxel_grid.h>                  //体素滤波器头文件
#include <algorithm>

using namespace std;

Pcd2GridMap::Pcd2GridMap(std::string name) : Node(name) 
{
    // Load parameters
    this->declare_parameter<double>("resolution", 0.1);
    this->declare_parameter<double>("min_z", 0.0);
    this->declare_parameter<double>("max_z", 1.0);
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<std::string>("pcd_file", "test.pcd");
    this->declare_parameter<std::string>("map_topic", "gridmap");

    this->get_parameter("resolution", resolution);
    this->get_parameter("min_z", min_z);
    this->get_parameter("max_z", max_z);
    this->get_parameter("frame_id", frame_id);
    this->get_parameter("pcd_file", pcd_file);
    this->get_parameter("map_topic", map_topic);

    // qos policys: https://www.shoufei.xyz/p/8b2be5e2.html
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(); // latch the last message to subscribers
    
    // Publisher
    gridmap_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic, qos);
}

Pcd2GridMap::~Pcd2GridMap()
{
    RCLCPP_INFO(this->get_logger(), "has published gridmap in /map topic.");
}

void Pcd2GridMap::load_pcd(const std::string &file, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file, *cloud) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load %s", file.c_str());
        rclcpp::shutdown();
    }
}

void Pcd2GridMap::cloud2gridmap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, nav_msgs::msg::OccupancyGrid &gridmap) {
    // Voxel grid filter
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(resolution, resolution, resolution);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.filter(*cloud_filtered);

    // Get the size of the gridmap
    double min_x = 1e9, min_y = 1e9, max_x = -1e9, max_y = -1e9;
    for (const auto &p : cloud_filtered->points) {
        min_x = std::min(min_x, static_cast<double>(p.x));
        min_y = std::min(min_y, static_cast<double>(p.y));
        max_x = std::max(max_x, static_cast<double>(p.x));
        max_y = std::max(max_y, static_cast<double>(p.y));
    }
    int width = static_cast<int>((max_x - min_x) / resolution) + 1;
    int height = static_cast<int>((max_y - min_y) / resolution) + 1;

    // Initialize the gridmap
    gridmap.header.frame_id = frame_id;
    gridmap.info.resolution = resolution;
    gridmap.info.width = width;
    gridmap.info.height = height;
    gridmap.info.origin.position.x = min_x;
    gridmap.info.origin.position.y = min_y;
    gridmap.info.origin.position.z = 0.0;
    gridmap.info.origin.orientation.x = 0.0;
    gridmap.info.origin.orientation.y = 0.0;
    gridmap.info.origin.orientation.z = 0.0;
    gridmap.info.origin.orientation.w = 1.0;
    gridmap.data.resize(width * height, -1);

    // Fill the gridmap
    for (const auto &p : cloud_filtered->points) {
        int x = static_cast<int>((p.x - min_x) / resolution);
        int y = static_cast<int>((p.y - min_y) / resolution);
        int i = x + y * width;
        if (p.z >= min_z && p.z <= max_z) {
            gridmap.data[i] = 100;
        } else {
            if (gridmap.data[i] == -1) gridmap.data[i] = 0;
        }
    }

    // Debug
    RCLCPP_INFO(this->get_logger(), "Gridmap size: %d x %d", width, height);
}

void Pcd2GridMap::publish_gridmap(const nav_msgs::msg::OccupancyGrid &gridmap) {
    gridmap_pub->publish(gridmap);
}

void Pcd2GridMap::run() {
    // Load a .pcd file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    load_pcd(pcd_file, cloud);

    // Convert the point cloud to a gridmap
    nav_msgs::msg::OccupancyGrid gridmap;
    cloud2gridmap(cloud, gridmap);

    rclcpp::Rate rate(1);
    while (rclcpp::ok()) {
        publish_gridmap(gridmap);
        rate.sleep();
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Pcd2GridMap>("pcd2gridmap_node");
    node->run();
    rclcpp::shutdown();
    return 0;
}