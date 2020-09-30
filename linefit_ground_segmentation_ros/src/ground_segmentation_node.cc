#include <ros/ros.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/point_cloud.h>

#include "ground_segmentation/ground_segmentation.h"
#include <visualization_msgs/Marker.h>

#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include "ground_segmentation/helper.h"

class SegmentationNode {
  ros::Publisher ground_pub_;
  ros::Publisher obstacle_pub_;
  GroundSegmentationParams params_;

  ros::Publisher marker_grid_pub_;
  ros::Publisher cluster_cloud_pub_;
  ros::Publisher roi_cloud_pub_;
  ros::Publisher deletedPoints_pub_;
  ros::Publisher text_pub_1_;
  ros::Publisher text_pub_2_;

  Helper helper;
  int call_back_id = 0;
  int min_point_count_threshold;
  float center_of_gravity_threshold;
  float horizontal_step;
  float vertical_step;
  float mesh_x_start;
  float mesh_x_end;
  float mesh_y_start;
  float mesh_y_end;


public:
  SegmentationNode(ros::NodeHandle& nh,
                   const std::string& ground_topic,
                   const std::string& obstacle_topic,
                   const GroundSegmentationParams& params,
                   const bool& latch = false) : params_(params) {

    ground_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(ground_topic, 1, latch);
    obstacle_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(obstacle_topic, 1, latch);
    marker_grid_pub_ = nh.advertise<visualization_msgs::Marker>("/Line_marker", 10);
    cluster_cloud_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/cluster_cloud", 1, latch);
    roi_cloud_pub_ =  nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/roi_cloud", 1, latch);
    deletedPoints_pub_ =  nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/deleted_points_cloud", 1, latch);
    text_pub_1_ = nh.advertise<visualization_msgs::Marker>("/callback_ID", 10);
    text_pub_2_ = nh.advertise<visualization_msgs::Marker>("/inference_time", 10);

    min_point_count_threshold = params.min_point_count_threshold;
    center_of_gravity_threshold = params.center_of_gravity_threshold;
    horizontal_step = params.horizontal_step;
    vertical_step = params.vertical_step;

    mesh_x_start = params.mesh_x_start;
    mesh_x_end = params.mesh_x_end;
    mesh_y_start = params.mesh_y_start;
    mesh_y_end = params.mesh_y_end;

  }




  void scanCallback(const sensor_msgs::PointCloud2ConstPtr& msg_cloud) {

    cout << "Callback Id: " << call_back_id << endl;

    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg_cloud, cloud);
    GroundSegmentation segmenter(params_);
    std::vector<int> labels;
    auto lines = segmenter.segment(cloud, &labels);
    //lineMarkers(lines, msg_cloud);

    auto clouds = helper.generateClouds(cloud, labels);
    auto ground_cloud = clouds[0];
    auto nonground_cloud = clouds[1];

    auto clouds_ = helper.makeXYZI(ground_cloud, nonground_cloud);
    auto  new_ground_cloud = clouds_[0];
    auto new_nonground_cloud = clouds_[1];

    // Deleting redundant non-ground points with grid mesh
    pcl::PointCloud<pcl::PointXYZI>::Ptr redundant_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    helper.DetectRedundantPoints(redundant_cloud, new_nonground_cloud,
                                 mesh_x_start, mesh_x_end, mesh_y_start, mesh_y_end, horizontal_step, vertical_step,
                                 min_point_count_threshold, cloud, marker_grid_pub_,
                                 center_of_gravity_threshold);


    helper.deletePoints(new_nonground_cloud, redundant_cloud);
    redundant_cloud->header = cloud.header;
    deletedPoints_pub_.publish(redundant_cloud);
    cout << "Redundant Cloud Size: " << redundant_cloud->points.size() << endl;


    //new_nonground_cloud = helper.PassThrough(new_nonground_cloud, "z", 1000, -1.5);

    /*
     // Deleting redundant point with clustering
    // ROI**********************************************
    pcl::PointCloud<pcl::PointXYZI>::Ptr roi_cloud = helper.DefineROI(new_nonground_cloud);
    roi_cloud->header = cloud.header;
    roi_cloud_pub_.publish(roi_cloud);
    // Clustering***************************************
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster = helper.Clustering(roi_cloud, new_nonground_cloud, 0.4, 1, 4);
    // Publish******************************************
    cloud_cluster->header = cloud.header;
    cluster_cloud_pub_.publish(cloud_cluster);
    */

    // Publish Clouds
    new_ground_cloud->header = cloud.header;
    new_nonground_cloud->header = cloud.header;
    ground_pub_.publish(new_ground_cloud);
    obstacle_pub_.publish(new_nonground_cloud);

    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = end - start;
    std::cout << "Calback time:  " << fp_ms.count() << "ms\n";


    // Show Callback ID:
    auto marker1 = helper.setTextMarker(to_string(call_back_id), cloud, 1, 18, 7, "");
    text_pub_1_.publish(marker1);
    auto marker2 = helper.setTextMarker(to_string(fp_ms.count()), cloud, 2, 16,7, "ms");
    text_pub_2_.publish(marker2);


    call_back_id++;
    cout << "******************************************************" << endl;
    //ros::Duration(1.5).sleep();

  }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "segmenter");
  google::InitGoogleLogging(argv[0]);

  ros::NodeHandle nh("~");

  // Do parameter stuff.
  GroundSegmentationParams params;
  nh.param("visualize", params.visualize, params.visualize);
  nh.param("n_bins", params.n_bins, params.n_bins);
  nh.param("n_segments", params.n_segments, params.n_segments);
  nh.param("max_dist_to_line", params.max_dist_to_line, params.max_dist_to_line);
  nh.param("max_slope", params.max_slope, params.max_slope);
  nh.param("long_threshold", params.long_threshold, params.long_threshold);
  nh.param("max_long_height", params.max_long_height, params.max_long_height);
  nh.param("max_start_height", params.max_start_height, params.max_start_height);
  nh.param("sensor_height", params.sensor_height, params.sensor_height);
  nh.param("line_search_angle", params.line_search_angle, params.line_search_angle);
  nh.param("n_threads", params.n_threads, params.n_threads);
  // Extra parameters:
  nh.param("min_point_count_threshold", params.min_point_count_threshold, params.min_point_count_threshold);
  nh.param("center_of_gravity_threshold", params.center_of_gravity_threshold, params.center_of_gravity_threshold);
  nh.param("horizontal_step", params.horizontal_step, params.horizontal_step);
  nh.param("vertical_step", params.vertical_step, params.vertical_step);
  nh.param("mesh_x_start", params.mesh_x_start, params.mesh_x_start);
  nh.param("mesh_x_end", params.mesh_x_end, params.mesh_x_end);
  nh.param("mesh_y_start", params.mesh_y_start, params.mesh_y_start);
  nh.param("mesh_y_end", params.mesh_y_end, params.mesh_y_end);


  // Params that need to be squared.
  double r_min, r_max, max_fit_error;
  if (nh.getParam("r_min", r_min)) {
    params.r_min_square = r_min*r_min;
  }
  if (nh.getParam("r_max", r_max)) {
    params.r_max_square = r_max*r_max;
  }
  if (nh.getParam("max_fit_error", max_fit_error)) {
    params.max_error_square = max_fit_error * max_fit_error;
  }

  std::string ground_topic, obstacle_topic, input_topic;
  bool latch;
  nh.param<std::string>("input_topic", input_topic, "input_cloud");
  nh.param<std::string>("ground_output_topic", ground_topic, "ground_cloud");
  nh.param<std::string>("obstacle_output_topic", obstacle_topic, "obstacle_cloud");
  nh.param("latch", latch, false);

  // Start node.
  SegmentationNode node(nh, ground_topic, obstacle_topic, params, latch);
  ros::Subscriber cloud_sub;
  cloud_sub = nh.subscribe(input_topic, 1, &SegmentationNode::scanCallback, &node);
  ros::spin();
}