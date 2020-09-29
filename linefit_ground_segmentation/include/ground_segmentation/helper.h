//Ekstra
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>

//downsampling
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <ros/ros.h>
//downsampling
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>

using namespace std;

class Helper{
public:

std::vector<pcl::PointCloud<pcl::PointXYZ>> generateClouds(pcl::PointCloud<pcl::PointXYZ> cloud, std::vector<int> labels){
    std::vector<pcl::PointCloud<pcl::PointXYZ>> clouds;
    pcl::PointCloud<pcl::PointXYZ> ground_cloud, obstacle_cloud;
    for (size_t i = 0; i < cloud.size(); ++i) {
        if (labels[i] == 1) ground_cloud.push_back(cloud[i]);
        else obstacle_cloud.push_back(cloud[i]);
    }
    clouds.push_back(ground_cloud);
    clouds.push_back(obstacle_cloud);

    return clouds;
  }

void deletePoints(pcl::PointCloud<pcl::PointXYZI>::Ptr& nonground_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr removed_cloud){
      for(int i=0; i<removed_cloud->points.size(); i++){
          for(int j=0; j<nonground_cloud->points.size(); j++){
              if(removed_cloud->points[i].x == nonground_cloud->points[j].x && removed_cloud->points[i].y == nonground_cloud->points[j].y
                                                    && removed_cloud->points[i].z == nonground_cloud->points[j].z){
                  nonground_cloud->points.erase(nonground_cloud->points.begin()+j);
              }
          }
      }
  }

visualization_msgs::Marker visualizePointCounts(visualization_msgs::MarkerArray& Markerarr, int mesh_point_count, float x_start, float y_start, int mesh_id, pcl::PointCloud<pcl::PointXYZ> cloud_for_frame){

    visualization_msgs::Marker marker;
    marker.header = pcl_conversions::fromPCL(cloud_for_frame.header);
    marker.ns = "basic_shapes" + to_string(mesh_id);
    marker.id = mesh_id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = mesh_id;
    marker.pose.position.y = y_start;
    marker.pose.position.z = -2.3;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.text = "15";

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 1;
    marker.color.g = 1;
    marker.color.b = 1;
    marker.color.a = 1.0;

    return marker;

  }


visualization_msgs::Marker setTextMarker(string text, pcl::PointCloud<pcl::PointXYZ> cloud_for_frame, int marker_id, float x, float y, string extra_text){

      visualization_msgs::Marker marker;
      marker.header = pcl_conversions::fromPCL(cloud_for_frame.header);
      marker.ns = "basic_shapes";
      marker.id = marker_id;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = -2.3;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.text = text + extra_text;

      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;

      marker.color.r = 1;
      marker.color.g = 1;
      marker.color.b = 1;
      marker.color.a = 1.0;

      return marker;

  }

void DetectRedundantPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr& deletedPoints, pcl::PointCloud<pcl::PointXYZI>::Ptr new_ground_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                           float x_start, float x_end, float y_start, float y_end, float horizontal_step, float vertical_step,
                           int threshold, pcl::PointCloud<pcl::PointXYZ> cloud_for_frame, ros::Publisher marker_pub_,
                           float center_of_gravity_threshold)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ROI = PassThrough(cloud, "x", x_start, x_end);
        cloud_ROI = PassThrough(cloud_ROI, "y", y_start, y_end);

        std::vector<float> horizontal_lines;
        std::vector<float> vertical_lines;

        vertical_lines.push_back(y_start);

        //float horizontal_step = (x_start-x_end)/n_x;
        //float vertical_step = (y_start-y_end)/n_y;

        //float horizontal_step = 1.6;
        //float vertical_step = 0.4;



        cout << "Horizontal step: " << horizontal_step << " Vertical step: " << vertical_step << endl;

        int counter = 0;
        while(1){
            float new_line = x_start - counter*horizontal_step;
            if(new_line > x_end){
                horizontal_lines.push_back(new_line);
                counter++;
            }else{break;}
        }

        counter = 0;
        while(1){
            float new_line = y_start - counter*vertical_step;
            if(new_line > y_end){
                vertical_lines.push_back(new_line);
                counter++;
            }else{break;}
        }


        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> grids_nonground;
        visualization_msgs::Marker marker;
        int mesh_id = 0;

        for(int i=0; i<horizontal_lines.size()-1; i++){
            for(int j=0; j<vertical_lines.size()-1; j++){

                float x_start = horizontal_lines[i];
                float x_end = horizontal_lines[i+1];
                float y_start = vertical_lines[j];
                float y_end = vertical_lines[j+1];

                // Mesh for non-ground**********************************************************************************
                pcl::PointCloud<pcl::PointXYZI>::Ptr mesh_nonground = PassThrough(cloud_ROI, "x", x_start, x_end);
                mesh_nonground = PassThrough(mesh_nonground, "y", y_start, y_end);
                int mesh_point_count = mesh_nonground->points.size();
                grids_nonground.push_back(mesh_nonground);
                float average;
                if(mesh_point_count != 0){
                    cout << "Mesh id: " << mesh_id << " | Mesh size: " << mesh_point_count << endl;
                    float sum = 0;
                    for(int i=0; i<mesh_nonground->points.size(); i++)
                        sum = sum + mesh_nonground->points[i].z;
                    average = sum/float(mesh_point_count);
                    cout << "Average z of points in mesh: " << to_string(average) << endl;
                }
                mesh_id++;
                // Condition in order to delete redundant points
                bool condition_1 = mesh_nonground->points.size() < threshold;
                bool condition_2 = average<center_of_gravity_threshold;

                if(condition_1 || condition_2){
                    *deletedPoints = *deletedPoints + *mesh_nonground;
                }

                //******************************************************************************************************

            }
        }
        visualization_msgs::Marker line_list = visualizeGrid(horizontal_lines, vertical_lines, cloud_for_frame);
        marker_pub_.publish(line_list);
      }





pcl::PointCloud<pcl::PointXYZI>::Ptr PassThrough(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, string axis, float max, float min)
  {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
      // Create the filtering object
      pcl::PassThrough<pcl::PointXYZI> pass;
      pass.setInputCloud (cloud_in);
      pass.setFilterFieldName (axis);
      pass.setFilterLimits (min, max);
      //pass.setFilterLimitsNegative (true);
      pass.filter (*cloud_filtered);

      return cloud_filtered;
    }

void printVector(std::vector<float> vector){
      for(int i=0; i<vector.size(); i++)
          cout << vector[i] << " ";
      cout << "\n********************************" << endl;
  }


std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> makeXYZI(pcl::PointCloud<pcl::PointXYZ> ground_cloud, pcl::PointCloud<pcl::PointXYZ> obstacle_cloud){

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds;
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_ground_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_nonground_cloud (new pcl::PointCloud<pcl::PointXYZI>);

    for(int i=0; i<ground_cloud.size(); i++)
    {
        pcl::PointXYZI point_temp;
        point_temp.x = ground_cloud[i].x;
        point_temp.y = ground_cloud[i].y;
        point_temp.z = ground_cloud[i].z;
        point_temp.intensity = 0;
        new_ground_cloud->points.push_back(point_temp);
    }

    for(int i=0; i<obstacle_cloud.size(); i++)
    {
        pcl::PointXYZI point_temp;
        point_temp.x = obstacle_cloud[i].x;
        point_temp.y = obstacle_cloud[i].y;
        point_temp.z = obstacle_cloud[i].z;
        point_temp.intensity = 10000;
        new_nonground_cloud->points.push_back(point_temp);
    }

    clouds.push_back(new_ground_cloud);
    clouds.push_back(new_nonground_cloud);
    return clouds;

}



pcl::PointCloud<pcl::PointXYZI>::Ptr DefineROI(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud){

    pcl::PointCloud<pcl::PointXYZI>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for(auto it = input_cloud->begin(); it!= input_cloud->end(); it++){
        pcl::PointXYZI temp_point = *it; float x = temp_point.x; float y = temp_point.y; float z = temp_point.z;
        float x_boundary = 70;
        float y_boundary = 10;
        if(x<x_boundary && x>-30 && y<y_boundary && y>-y_boundary)
            roi_cloud->points.push_back(temp_point);
    }
    return roi_cloud;
  }


pcl::PointCloud<pcl::PointXYZI>::Ptr Clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& nonground_cloud,float tol, int min, int max)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(input_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;

    ec.setClusterTolerance(tol);
    ec.setMinClusterSize(min);
    ec.setMaxClusterSize(max);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    ec.extract(cluster_indices);

    int j = 0;
    int num_cluster = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {   //cout << *it << endl; // bir clusterdaki noktaların idleri

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            //cout << "Tek noktanın cloud_in içindeki idsi: " << *pit << endl; // cluster içindeki tek nokta
            pcl::PointXYZI point_temp = (*input_cloud)[*pit];
            point_temp.intensity = j;
            cloud_cluster->points.push_back (point_temp);


            //deleting absurd points from non-ground cloud
            for(int i=0; i<nonground_cloud->points.size(); i++ ){
                if(point_temp.x == nonground_cloud->points[i].x && point_temp.y == nonground_cloud->points[i].y && point_temp.z == nonground_cloud->points[i].z){
                    nonground_cloud->points.erase(nonground_cloud->points.begin()+i);
                    //cout << "Point is deleted from non-ground cloud!" << endl;
                }
            }

        }
        j = j+5;
        num_cluster++;
    }
    cout << "Number of cluster: " << num_cluster << endl;

    return cloud_cluster;
}



visualization_msgs::Marker visualizeGrid(std::vector<float> horizontal_lines, std::vector<float> vertical_lines, pcl::PointCloud<pcl::PointXYZ> cloud_for_frame){

    float x_start = horizontal_lines[0];
    float x_end = horizontal_lines.back();
    float y_start = vertical_lines[0];
    float y_end = vertical_lines.back();

    visualization_msgs::Marker line_list;
    line_list.header = pcl_conversions::fromPCL(cloud_for_frame.header);
    line_list.ns = "lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.01;
    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    line_list.color.b = 1.0;
    line_list.color.g = 1.0;

    for (int i = 0; i < horizontal_lines.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = horizontal_lines[i];
        p.y = y_start;
        p.z = -2.3;

        line_list.points.push_back(p);
        p.z = -2.3;
        p.x = horizontal_lines[i];
        p.y = y_end;
        line_list.points.push_back(p);
    }

    for (int i = 0; i < vertical_lines.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = x_start;
        p.y = vertical_lines[i];
        p.z = -2.3;

        line_list.points.push_back(p);
        p.z = -2.3;
        p.x = x_end;
        p.y = vertical_lines[i];
        line_list.points.push_back(p);
    }

    return line_list;
  }

};
