/*
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/common.h>
#include <Eigen/Geometry>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/angles.h>

ros::Publisher pub;
cv::Mat image;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  //creates new clean image every time
  image = cv::Mat();
  image = cv::Mat::zeros(15, 15, CV_8UC1);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);
  

  float fov = 120;
   // Calculate the cosine of the FOV angle
  float cos_fov = cos(pcl::deg2rad(fov));

  // Create a unit z-vector pointing straight up
  Eigen::Vector3f z_vector(0, 0, 1);

  // Create a filter for removing points outside the FOV
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for (int i = 0; i < cloud.points.size(); i++) {
    // Get the xyz coordinates of the point
    Eigen::Vector3f point(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);

    // Calculate the angle between the point and the z-axis using the dot product
    float cos_angle = point.dot(z_vector) / point.norm();

    // Check if the point is within the FOV
    if (cos_angle >= cos_fov) {
      inliers->indices.push_back(i);
    }
  }

  // Create a filter object and apply it to the input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(cloud);
























  // min and max of point cloud values
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(cloud, min_pt, max_pt);

  for (int i = 0; i < cloud.points.size(); i++) {

    // Access the x, y, and z coordinates of the point
    float x = cloud.points[i].x;
    float y = cloud.points[i].y;
    float z = cloud.points[i].z;

    // Map the height of the point to a grayscale value
    int value = (z / 10.0) * 255;
    
    // Store the grayscale value in the image
    int row = ((x - min_pt.x) / (max_pt.x - min_pt.x) * 14); 
    int col = ((y - min_pt.y) / (max_pt.y - min_pt.y) * 14);
    printf("row %i x %f col %i y %f \n", row, x, col, y);
    
    image.at<unsigned char>(row, col) = std::max(0, std::min(255, value));
  
    
  }

  // Resize the image to a specific size (currently aribitrary)
  cv::resize(image, image, cv::Size(15, 15));

  // Convert the image to a ROS message
  sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();

  // Publish the image
  pub.publish(image_msg);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cloud_to_image_node");
  ros::NodeHandle nh;

  // Subscribe to the point cloud topic
  ros::Subscriber sub = nh.subscribe ("mmWaveDataHdl/RScan", 1, cloud_cb);


  // Advertise the image topic
  pub = nh.advertise<sensor_msgs::Image> ("output", 1);

  // Spin
  ros::spin ();
}
*/


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/common.h>
#include <Eigen/Geometry>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/angles.h>

ros::Publisher pub;
cv::Mat image;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  //creates new clean image every time
  image = cv::Mat();
  image = cv::Mat::zeros(15, 15, CV_8UC1);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);
  

  float fov = 60;
  // Calculate the cosine of the FOV angle
  float cos_fov = cos(pcl::deg2rad(fov));

  // Create a unit z-vector pointing straight up
  Eigen::Vector3f z_vector(0, 0, 1);

  // Create a filter for removing points outside the FOV
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  for (int i = 0; i < cloud.points.size(); i++) {
    // Get the xyz coordinates of the point
    Eigen::Vector3f point(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);

    // Calculate the angle between the point and the z-axis using the dot product
    float cos_angle = point.dot(z_vector) / point.norm();

    // Check if the point is within the FOV
    if (cos_angle >= cos_fov) {
      inliers->indices.push_back(i);
    }
  }

  // Create a filter object and apply it to the input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud.makeShared());
  extract.setIndices(inliers);
  extract.setNegative(false);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  extract.filter(*filtered_cloud);

  // min and max of point cloud values
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*filtered_cloud, min_pt, max_pt);

  float x_range = max_pt.x - min_pt.x;
  float y_range = max_pt.y - min_pt.y;
  printf("x range %f y range %f \n", x_range, y_range);
  printf("x min %f y min %f \n", min_pt.x, min_pt.y);
  printf("x max %f y max %f \n", max_pt.x, max_pt.y);
  for (int i = 0; i < filtered_cloud->points.size(); i++) {
    // Access the x, y, and z coordinates of the point
    float x = filtered_cloud->points[i].x;
    float y = filtered_cloud->points[i].y;
    float z = filtered_cloud->points[i].z;

    // Map the height of the point to a grayscale value
    int value = (z / 10.0) * 255;
 // Store the grayscale value in the image
    int row = ((x - min_pt.x) / (max_pt.x - min_pt.x) * 14); 
    int col = ((y - min_pt.y) / (max_pt.y - min_pt.y) * 14);
    //printf("row %i x %f col %i y %f \n", row, x, col, y);
    
    image.at<unsigned char>(row, col) = std::max(0, std::min(255, value));
  
    
  }

  // Resize the image to a specific size (currently aribitrary)
  cv::resize(image, image, cv::Size(400, 400));

  // Convert the image to a ROS message
  sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();

  // Publish the image
  pub.publish(image_msg);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cloud_to_image_node");
  ros::NodeHandle nh;

  // Subscribe to the point cloud topic
  ros::Subscriber sub = nh.subscribe ("mmWaveDataHdl/RScan", 1, cloud_cb);


  // Advertise the image topic
  pub = nh.advertise<sensor_msgs::Image> ("output", 1);

  // Spin
  ros::spin ();
}