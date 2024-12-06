/*
Pcd2GridMap (Author: sujit-168)
Convert a .pcd file to a gridmap and publish it as a nav_msgs::OccupancyGrid message.
*/

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Pcd2GridMap : public rclcpp::Node {
private:
    int thres_point_count = 10;   //半径滤波的点数阈值
    double resolution;            //网格分辨率
    double min_z, max_z;          //z轴保留范围
    std::string frame_id;         //坐标系
    std::string pcd_file;         //PCD文件路径
    std::string map_topic;        //发布到的话题

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr gridmap_pub; // Publisher for the gridmap

    void load_pcd(const std::string &file, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void cloud2gridmap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, nav_msgs::msg::OccupancyGrid &gridmap);
    void publish_gridmap(const nav_msgs::msg::OccupancyGrid &gridmap);
public:
    explicit Pcd2GridMap(std::string name);
    ~Pcd2GridMap();
    void run();
};