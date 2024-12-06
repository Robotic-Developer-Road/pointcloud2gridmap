#define COMPILEDWITHC11

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "pcd2grid/pointcloud2gridmap.hpp"

using namespace std;

PointCloud2GridMap::PointCloud2GridMap(std::string name) : Node(name)
{
    // Load parameters
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<std::string>("cloud_topic", "points");
    this->declare_parameter<std::string>("pose_topic", "poses");
    this->declare_parameter<std::string>("map_topic", "gridmap");
    this->declare_parameter<double>("cloud_max_x", 10.0);
    this->declare_parameter<double>("cloud_min_x", -10.0);
    this->declare_parameter<double>("cloud_max_z", 16.0);
    this->declare_parameter<double>("cloud_min_z", -5.0);
    this->declare_parameter<double>("free_thresh", 0.55);
    this->declare_parameter<double>("occupied_thresh", 0.50);
    this->declare_parameter<double>("thresh_diff", 0.01);
    this->declare_parameter<double>("upper_left_x", -1.5);
    this->declare_parameter<double>("upper_left_y", -2.5);
    this->declare_parameter<double>("scale_factor", 3.0);
    this->declare_parameter<double>("resize_factor", 5.0);
    this->declare_parameter<int>("resolution", 10);
    this->declare_parameter<int>("visit_thresh", 0);
    this->declare_parameter<int>("use_local_counters", 0);

    this->get_parameter("frame_id", frame_id);
    this->get_parameter("cloud_topic", cloud_topic);
    this->get_parameter("pose_topic", pose_topic);
    this->get_parameter("map_topic", map_topic);
    this->get_parameter("resolution", resolution);
    this->get_parameter("cloud_max_x", cloud_max_x);
    this->get_parameter("cloud_min_x", cloud_min_x);
    this->get_parameter("cloud_max_z", cloud_max_z);
    this->get_parameter("cloud_min_z", cloud_min_z);
    this->get_parameter("free_thresh", free_thresh);
    this->get_parameter("occupied_thresh", occupied_thresh);
    this->get_parameter("thresh_diff", thresh_diff);
    this->get_parameter("upper_left_x", upper_left_x);
    this->get_parameter("upper_left_y", upper_left_y);
    this->get_parameter("scale_factor", scale_factor);
    this->get_parameter("resize_factor", resize_factor);
    this->get_parameter("visit_thresh", visit_thresh);
    this->get_parameter("use_local_counters", use_local_counters);
    
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    // sub_pointcloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic, 1, std::bind(&PointCloud2GridMap::cloudCallback, this, std::placeholders::_1));
    // sub_keyframe_poses = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, 1, std::bind(&PointCloud2GridMap::kfCallback, this, std::placeholders::_1));
    pub_grid_map = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic, qos);

    // Initialize grid map variables
    grid_max_x = cloud_max_x * scale_factor;
    grid_min_x = cloud_min_x * scale_factor;
    grid_max_z = cloud_max_z * scale_factor;
    grid_min_z = cloud_min_z * scale_factor;

    double grid_res_x = grid_max_x - grid_min_x;    // width
    double grid_res_z = grid_max_z - grid_min_z;    // height

    h = grid_res_z;
    w = grid_res_x;
    n_kf_received = 0;

    global_occupied_counter.create(h, w, CV_32SC1);
    global_visit_counter.create(h, w, CV_32SC1);
    global_occupied_counter.setTo(cv::Scalar(0));
    global_visit_counter.setTo(cv::Scalar(0));

    grid_map_msg.data.resize(h * w);
    grid_map_msg.info.width = w;
    grid_map_msg.info.height = h;
    grid_map_msg.info.resolution = 1.0 / scale_factor;

    grid_map_int = cv::Mat(h, w, CV_8SC1, (char*)(grid_map_msg.data.data()));

    grid_map.create(h, w, CV_32FC1);
    grid_map_thresh.create(h, w, CV_8UC1);
    grid_map_thresh_resized.create(h * resize_factor, w * resize_factor, CV_8UC1);

    local_occupied_counter.create(h, w, CV_32SC1);
    local_visit_counter.create(h, w, CV_32SC1);
    local_map_pt_mask.create(h, w, CV_8UC1);

    norm_factor_x = float(grid_res_x - 1) / float(grid_max_x - grid_min_x);
    norm_factor_z = float(grid_res_z - 1) / float(grid_max_z - grid_min_z);
}

void PointCloud2GridMap::printParams() {
    RCLCPP_INFO(this->get_logger(),"scale_factor: %f\n", scale_factor);
    RCLCPP_INFO(this->get_logger(),"resize_factor: %f\n", resize_factor);
    RCLCPP_INFO(this->get_logger(),"cloud_max_x: %f\n", cloud_max_x);
    RCLCPP_INFO(this->get_logger(),"cloud_min_x: %f\n", cloud_min_x);
    RCLCPP_INFO(this->get_logger(),"cloud_max_z: %f\n", cloud_max_z);
    RCLCPP_INFO(this->get_logger(),"cloud_min_z: %f\n", cloud_min_z);
    RCLCPP_INFO(this->get_logger(),"free_thresh: %f\n", free_thresh);
    RCLCPP_INFO(this->get_logger(),"occupied_thresh: %f\n", occupied_thresh);         
    RCLCPP_INFO(this->get_logger(),"thresh_diff: %f\n", thresh_diff);
    RCLCPP_INFO(this->get_logger(),"upper_left_x: %f\n", upper_left_x);
    RCLCPP_INFO(this->get_logger(),"upper_left_y: %f\n", upper_left_y);
    RCLCPP_INFO(this->get_logger(),"resolution: %d\n", resolution);
    RCLCPP_INFO(this->get_logger(),"visit_thresh: %d\n", visit_thresh);
    RCLCPP_INFO(this->get_logger(),"use_local_counters: %d\n", use_local_counters);
    RCLCPP_INFO(this->get_logger(),"norm_factor_x: %f\n", norm_factor_x);
    RCLCPP_INFO(this->get_logger(),"norm_factor_z: %f\n", norm_factor_z);
    RCLCPP_INFO(this->get_logger(),"grid_max: %f, %f\t grid_min: %f, %f\n", grid_max_x, grid_max_z, grid_min_x, grid_min_z);
    RCLCPP_INFO(this->get_logger(),"output_size: (%d, %d)\n", grid_map_thresh_resized.rows, grid_map_thresh_resized.cols);
    RCLCPP_INFO(this->get_logger(),"grid_size: (%d, %d)\n", h, w);
}

PointCloud2GridMap::~PointCloud2GridMap()
{
    RCLCPP_INFO(this->get_logger(), "has published gridmap in /map topic.");
}

void PointCloud2GridMap::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr& pt_cloud){
    RCLCPP_INFO(this->get_logger(), "I heard: [%s]{%d}", pt_cloud->header.frame_id.c_str(),
                pt_cloud->header.stamp.sec);
}

void PointCloud2GridMap::kfCallback(const geometry_msgs::msg::PoseStamped::SharedPtr& camera_pose){
    RCLCPP_INFO(this->get_logger(), "I heard: [%s]{%d}", camera_pose->header.frame_id.c_str(),
                camera_pose->header.stamp.sec);
}

void PointCloud2GridMap::saveMap(unsigned int id) {
    RCLCPP_INFO(this->get_logger(),"saving maps with id: %u\n", id);
    mkdir("results", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (id > 0) {
        cv::imwrite("results//grid_map_" + to_string(id) + ".jpg", grid_map);
        cv::imwrite("results//grid_map_thresh_" + to_string(id) + ".jpg", grid_map_thresh);
        cv::imwrite("results//grid_map_thresh_resized" + to_string(id) + ".jpg", grid_map_thresh_resized);
    }
    else {
        cv::imwrite("results//grid_map.jpg", grid_map);
        cv::imwrite("results//grid_map_thresh.jpg", grid_map_thresh);
        cv::imwrite("results//grid_map_thresh_resized.jpg", grid_map_thresh_resized);
    }
}

void PointCloud2GridMap::ptCallback(const geometry_msgs::msg::PoseArray::SharedPtr& pts_and_pose){
    RCLCPP_INFO(this->get_logger(), "Received points and pose: [%s]{%d}", pts_and_pose->header.frame_id.c_str(),
                pts_and_pose->header.stamp.sec);
    if (loop_closure_being_processed){ return; }

    updateGridMap(pts_and_pose);

    grid_map_msg.info.map_load_time = this->now();
    pub_grid_map->publish(grid_map_msg);
}

void PointCloud2GridMap::loopClosingCallback(const geometry_msgs::msg::PoseArray::SharedPtr& all_kf_and_pts){
    loop_closure_being_processed = true;
    resetGridMap(all_kf_and_pts);
    loop_closure_being_processed = false;
}

void PointCloud2GridMap::getMixMax(const geometry_msgs::msg::PoseArray::SharedPtr& pts_and_pose,
               geometry_msgs::msg::Point& min_pt, geometry_msgs::msg::Point& max_pt) {
    min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<double>::infinity();
    max_pt.x = max_pt.y = max_pt.z = -std::numeric_limits<double>::infinity();
    for (unsigned int i = 0; i < pts_and_pose->poses.size(); ++i){
        const geometry_msgs::msg::Point& curr_pt = pts_and_pose->poses[i].position;
        if (curr_pt.x < min_pt.x) { min_pt.x = curr_pt.x; }
        if (curr_pt.y < min_pt.y) { min_pt.y = curr_pt.y; }
        if (curr_pt.z < min_pt.z) { min_pt.z = curr_pt.z; }

        if (curr_pt.x > max_pt.x) { max_pt.x = curr_pt.x; }
        if (curr_pt.y > max_pt.y) { max_pt.y = curr_pt.y; }
        if (curr_pt.z > max_pt.z) { max_pt.z = curr_pt.z; }
    }
}

void PointCloud2GridMap::processMapPt(const geometry_msgs::msg::Point &curr_pt, cv::Mat &occupied,
                  cv::Mat &visited, cv::Mat &pt_mask, int kf_pos_grid_x, int kf_pos_grid_z) {
    float pt_pos_x = curr_pt.x * scale_factor;
    float pt_pos_z = curr_pt.z * scale_factor;

    int pt_pos_grid_x = int(floor((pt_pos_x - grid_min_x) * norm_factor_x));
    int pt_pos_grid_z = int(floor((pt_pos_z - grid_min_z) * norm_factor_z));

    if (pt_pos_grid_x < 0 || pt_pos_grid_x >= w)
        return;

    if (pt_pos_grid_z < 0 || pt_pos_grid_z >= h)
        return;

    ++occupied.at<int>(pt_pos_grid_z, pt_pos_grid_x);
    pt_mask.at<uchar>(pt_pos_grid_z, pt_pos_grid_x) = 255;

    int x0 = kf_pos_grid_x;
    int y0 = kf_pos_grid_z;
    int x1 = pt_pos_grid_x;
    int y1 = pt_pos_grid_z;
    bool steep = (abs(y1 - y0) > abs(x1 - x0));
    if (steep){
        swap(x0, y0);
        swap(x1, y1);
    }
    if (x0 > x1){
        swap(x0, x1);
        swap(y0, y1);
    }
    int dx = x1 - x0;
    int dy = abs(y1 - y0);
    double error = 0;
    double deltaerr = ((double)dy) / ((double)dx);
    int y = y0;
    int ystep = (y0 < y1) ? 1 : -1;
    for (int x = x0; x <= x1; ++x){
        if (steep) {
            ++visited.at<int>(x, y);
        }
        else {
            ++visited.at<int>(y, x);
        }
        error = error + deltaerr;
        if (error >= 0.5){
            y = y + ystep;
            error = error - 1.0;
        }
    }
}

void PointCloud2GridMap::processMapPts(const std::vector<geometry_msgs::msg::Pose> &pts, unsigned int n_pts,
                   unsigned int start_id, int kf_pos_grid_x, int kf_pos_grid_z) {
    unsigned int end_id = start_id + n_pts;
    if (use_local_counters) {
        local_map_pt_mask.setTo(0);
        local_occupied_counter.setTo(0);
        local_visit_counter.setTo(0);
        for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id){
            processMapPt(pts[pt_id].position, local_occupied_counter, local_visit_counter,
                         local_map_pt_mask, kf_pos_grid_x, kf_pos_grid_z);
        }
        for (int row = 0; row < h; ++row){
            for (int col = 0; col < w; ++col){
                if (local_map_pt_mask.at<uchar>(row, col) == 0) {
                    local_occupied_counter.at<int>(row, col) = 0;
                }
                else {
                    local_occupied_counter.at<int>(row, col) = local_visit_counter.at<int>(row, col);
                }
            }
        }
        global_occupied_counter += local_occupied_counter;
        global_visit_counter += local_visit_counter;
    }
    else {
        for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id){
            processMapPt(pts[pt_id].position, global_occupied_counter, global_visit_counter,
                         local_map_pt_mask, kf_pos_grid_x, kf_pos_grid_z);
        }
    }
}

void PointCloud2GridMap::updateGridMap(const geometry_msgs::msg::PoseArray::SharedPtr& pts_and_pose){
    const geometry_msgs::msg::Point &kf_location = pts_and_pose->poses[0].position;
    kf_pos_x = kf_location.x * scale_factor;
    kf_pos_z = kf_location.z * scale_factor;

    kf_pos_grid_x = int(floor((kf_pos_x - grid_min_x) * norm_factor_x));
    kf_pos_grid_z = int(floor((kf_pos_z - grid_min_z) * norm_factor_z));

    if (kf_pos_grid_x < 0 || kf_pos_grid_x >= w)
        return;

    if (kf_pos_grid_z < 0 || kf_pos_grid_z >= h)
        return;
    ++n_kf_received;
    unsigned int n_pts = pts_and_pose->poses.size() - 1;
    processMapPts(pts_and_pose->poses, n_pts, 1, kf_pos_grid_x, kf_pos_grid_z);

    this->getGridMap();
    showGridMap(pts_and_pose->header.stamp.sec);
}

void PointCloud2GridMap::resetGridMap(const geometry_msgs::msg::PoseArray::SharedPtr& all_kf_and_pts){
    global_visit_counter.setTo(0);
    global_occupied_counter.setTo(0);

    unsigned int n_kf = all_kf_and_pts->poses[0].position.x;
    if ((unsigned int) (all_kf_and_pts->poses[0].position.y) != n_kf ||
        (unsigned int) (all_kf_and_pts->poses[0].position.z) != n_kf) {
        RCLCPP_INFO(this->get_logger(),"resetGridMap :: Unexpected formatting in the keyframe count element\n");
        return;
    }
    RCLCPP_INFO(this->get_logger(),"Resetting grid map with %d key frames\n", n_kf);
#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
    unsigned int id = 0;
    for (unsigned int kf_id = 0; kf_id < n_kf; ++kf_id){
        const geometry_msgs::msg::Point &kf_location = all_kf_and_pts->poses[++id].position;
        unsigned int n_pts = all_kf_and_pts->poses[++id].position.x;
        if ((unsigned int)(all_kf_and_pts->poses[id].position.y) != n_pts ||
            (unsigned int)(all_kf_and_pts->poses[id].position.z) != n_pts) {
            RCLCPP_INFO(this->get_logger(),"resetGridMap :: Unexpected formatting in the point count element for keyframe %d\n", kf_id);
            return;
        }
        float kf_pos_x = kf_location.x * scale_factor;
        float kf_pos_z = kf_location.z * scale_factor;

        int kf_pos_grid_x = int(floor((kf_pos_x - grid_min_x) * norm_factor_x));
        int kf_pos_grid_z = int(floor((kf_pos_z - grid_min_z) * norm_factor_z));

        if (kf_pos_grid_x < 0 || kf_pos_grid_x >= w)
            continue;

        if (kf_pos_grid_z < 0 || kf_pos_grid_z >= h)
            continue;

        if (id + n_pts >= all_kf_and_pts->poses.size()) {
            RCLCPP_INFO(this->get_logger(),"resetGridMap :: Unexpected end of the input array while processing keyframe %u with %u points: only %u out of %u elements found\n",
                   kf_id, n_pts, static_cast<unsigned int>(all_kf_and_pts->poses.size()), id + n_pts);
            return;
        }
        processMapPts(all_kf_and_pts->poses, n_pts, id + 1, kf_pos_grid_x, kf_pos_grid_z);
        id += n_pts;
    }    
    getGridMap();
#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
    double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    RCLCPP_INFO(this->get_logger(),"Done. Time taken: %f secs\n", ttrack);
    pub_grid_map->publish(grid_map_msg);
    showGridMap(all_kf_and_pts->header.stamp.sec);
}

void PointCloud2GridMap::getGridMap() {

    /**
     * grid map: free（0）、occupied（100）和 unknown（-1
     * occupied_thresh: 占用概率阈值：大于此阈值的像素被视为完全占用 
     * free_thresh: 占用概率小于此阈值的像素被视为完全未占用
     * 
    **/ 
    for (int row = 0; row < h; ++row){
        for (int col = 0; col < w; ++col){
            int visits = global_visit_counter.at<int>(row, col);
            int occupieds = global_occupied_counter.at<int>(row, col);

            if (visits <= visit_thresh){
                grid_map.at<float>(row, col) = 0.5;
            }
            else {
                grid_map.at<float>(row, col) = 1.0 - float(occupieds / visits);
            }
            if (grid_map.at<float>(row, col) >= free_thresh) {
                grid_map_thresh.at<uchar>(row, col) = 255;
            }
            else if (grid_map.at<float>(row, col) < free_thresh && grid_map.at<float>(row, col) >= occupied_thresh) {
                grid_map_thresh.at<uchar>(row, col) = 128;     
            }
            else {
                grid_map_thresh.at<uchar>(row, col) = -1;
            }
            grid_map_int.at<char>(row, col) = (1 - grid_map.at<float>(row, col)) * 100;
        }
    }

    cv::resize(grid_map_thresh, grid_map_thresh_resized, grid_map_thresh_resized.size());
}

void PointCloud2GridMap::showGridMap(unsigned int id) {
    cv::Mat grid_map_vis;
    cv::applyColorMap(grid_map_int, grid_map_vis, cv::COLORMAP_JET);
    cv::imshow("grid_map", grid_map_vis);
    cv::waitKey(1);
    if (id > 0) {
        cv::imwrite("results//grid_map_" + to_string(id) + ".jpg", grid_map_vis);
    }
    else {
        cv::imwrite("results//grid_map.jpg", grid_map_vis);
    }
}

void PointCloud2GridMap::run() {
    printParams();
  
    rclcpp::Rate rate(1);
    while (rclcpp::ok()) {
        // sub_pts_and_pose = this->create_subscription<geometry_msgs::msg::PoseArray>("pts_and_pose", 1000, std::bind(&PointCloud2GridMap::ptCallback, this, std::placeholders::_1));
        // sub_all_kf_and_pts = this->create_subscription<geometry_msgs::msg::PoseArray>("all_kf_and_pts", 1000, std::bind(&PointCloud2GridMap::loopClosingCallback, this, std::placeholders::_1));
        rate.sleep();
    }
    this->saveMap();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloud2GridMap>("pointcloud2gridmap_node");
    node->run();
    rclcpp::shutdown();
    return 0;
}