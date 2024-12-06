#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class PointCloud2GridMap : public rclcpp::Node
{
private:
    // parameters
    std::string frame_id;                        // frame id of the grid map
    std::string cloud_topic;                     // sub topic name for the point cloud
    std::string map_topic;                       // pub topic name for the grid map
    std::string pose_topic;                      // sub topic name for the keyframe pose

    float scale_factor = 3;                      // scale factor for the grid map
    float resize_factor = 5;                     // resize factor for the grid map
    float cloud_max_x = 10;               
    float cloud_min_x = -10.0;
    float cloud_max_z = 16;
    float cloud_min_z = -5;
    float free_thresh = 0.55;                    // 占用概率小于此阈值的像素被视为完全未占用
    float occupied_thresh = 0.50;                // 占用概率阈值：大于此阈值的像素被视为完全占用
    float thresh_diff = 0.01;
    float upper_left_x = -1.5;
    float upper_left_y = -2.5;
    float min_z, max_z;
    
    int resolution = 10;                   // resolution of the grid map
    int visit_thresh = 0;
    unsigned int use_local_counters = 0;

    float grid_max_x, grid_min_x, grid_max_z, grid_min_z;
    cv::Mat global_occupied_counter, global_visit_counter;
    cv::Mat local_occupied_counter, local_visit_counter;
    cv::Mat local_map_pt_mask;
    cv::Mat grid_map, grid_map_int, grid_map_thresh, grid_map_thresh_resized;
    float norm_factor_x, norm_factor_z;
    int h, w;
    unsigned int n_kf_received;
    bool loop_closure_being_processed = false;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_grid_map;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_keyframe_poses;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_pts_and_pose;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_all_kf_and_pts;
    
    nav_msgs::msg::OccupancyGrid grid_map_msg;

    float kf_pos_x, kf_pos_z;
    int kf_pos_grid_x, kf_pos_grid_z;

    void updateGridMap(const geometry_msgs::msg::PoseArray::SharedPtr& pts_and_pose);
    void resetGridMap(const geometry_msgs::msg::PoseArray::SharedPtr& pts_and_pose);
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr& pt_cloud);
    void kfCallback(const geometry_msgs::msg::PoseStamped::SharedPtr& camera_pose);
    void saveMap(unsigned int id = 0);
    void ptCallback(const geometry_msgs::msg::PoseArray::SharedPtr& pts_and_pose);
    void loopClosingCallback(const geometry_msgs::msg::PoseArray::SharedPtr& all_kf_and_pts);
    void parseParams(int argc, char **argv);
    void printParams();
    void showGridMap(unsigned int id = 0);
    void getMixMax(const geometry_msgs::msg::PoseArray::SharedPtr& pts_and_pose,
                geometry_msgs::msg::Point& min_pt, geometry_msgs::msg::Point& max_pt);
    void processMapPt(const geometry_msgs::msg::Point &curr_pt, cv::Mat &occupied,
                    cv::Mat &visited, cv::Mat &pt_mask, int kf_pos_grid_x, int kf_pos_grid_z);
    void processMapPts(const std::vector<geometry_msgs::msg::Pose> &pts, unsigned int n_pts,
                    unsigned int start_id, int kf_pos_grid_x, int kf_pos_grid_z);
    void getGridMap();

public:
    explicit PointCloud2GridMap(std::string name);
    ~PointCloud2GridMap();
    void run();
};
