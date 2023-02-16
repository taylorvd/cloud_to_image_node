/*
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);

  
  // Convert the point cloud to a grayscale image
  cv::Mat image(cloud.height, cloud.width, CV_8UC1);
  for (int y = 0; y < cloud.height; y++) {
    for (int x = 0; x < cloud.width; x++) {
      pcl::PointXYZ point = cloud.at(x, y);
      // Map the height of the point to a grayscale value
      int value = (point.z / 10.0) * 255;
      image.at<unsigned char>(y, x) = std::max(0, std::min(255, value));
    }
  }

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
  ros::Subscriber sub = nh.subscribe ("os1_cloud_node/points", 1, cloud_cb);

  // Advertise the image topic
  pub = nh.advertise<sensor_msgs::Image> ("output", 1);

  // Spin
  ros::spin ();
}
 

 ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);

  
  // Convert the point cloud to a grayscale image
  cv::Mat image(cloud.height, cloud.width, CV_8UC1);
  for (int y = 0; y < cloud.height; y++) {
    for (int x = 0; x < cloud.width; x++) {
      pcl::PointXYZ point = cloud.at(x, y);
      // Map the height of the point to a grayscale value
      int value = (point.z / 10.0) * 255;
      image.at<unsigned char>(y, x) = std::max(0, std::min(255, value));
    }
  }

  // Convert the image to a ROS message using cv_bridge
  cv_bridge::CvImage image_msg;
  image_msg.header.stamp = ros::Time::now();
  image_msg.header.frame_id = "map";
  image_msg.encoding = "mono8";
  image_msg.image = image;

  // Publish the image
  pub.publish(image_msg.toImageMsg());
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cloud_to_image_node");
  ros::NodeHandle nh;

  // Subscribe to the point cloud topic
  ros::Subscriber sub = nh.subscribe ("os1_cloud_node/points", 1, cloud_cb);

  // Advertise the image topic
  pub = nh.advertise<sensor_msgs::Image> ("output", 1);

  // Spin
  ros::spin ();
}


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

ros::Publisher pub;
int height, width;
cv::Mat image;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);

  height = cloud.height;
  width = cloud.width;

  // Resize the large image if the size of the incoming cloud has changed
  if (height != image.rows || width != image.cols) {
    image = cv::Mat(height, width * 10, CV_8UC1);
  }

  // Convert the point cloud to a grayscale image
  cv::Mat small_image(height, width, CV_8UC1);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      pcl::PointXYZ point = cloud.at(x, y);
      // Map the height of the point to a grayscale value
      int value = (point.z / 10.0) * 255;
      small_image.at<unsigned char>(y, x) = std::max(0, std::min(255, value));
    }
  }

  // Copy the small image into the large image
  cv::Mat roi(image, cv::Rect(0, 0, width, height));
  small_image.copyTo(roi);

  // Convert the large image to a ROS message
  sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();

  // Publish the large image
  pub.publish(image_msg);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cloud_to_image_node");
  ros::NodeHandle nh;

  // Subscribe to the point cloud topic
  ros::Subscriber sub = nh.subscribe ("os1_cloud_node/points", 1, cloud_cb);

  // Advertise the image topic
  pub = nh.advertise<sensor_msgs::Image> ("output", 1);

  // Spin
  ros::spin ();
}
*/
/*
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

ros::Publisher pub;
cv::Mat image;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);

  // Convert the point cloud to a grayscale image
  image.create(cloud.height, cloud.width, CV_8UC1);
  for (int y = 0; y < cloud.height; y++) {
    for (int x = 0; x < cloud.width; x++) {
      pcl::PointXYZ point = cloud.at(x, y);
      // Map the height of the point to a grayscale value
      int value = (point.z / 10.0) * 255;
      image.at<unsigned char>(y, x) = std::max(0, std::min(255, value));
    }
  }

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
  ros::Subscriber sub = nh.subscribe("os1_cloud_node/points", 1, cloud_cb);
  //ros::Subscriber sub = nh.subscribe("mmWaveDataHdl/RScan", 1, cloud_cb);
  
  // Advertise the image topic
  pub = nh.advertise<sensor_msgs::Image> ("output", 1);
  
  // Spin
  ros::spin ();
}


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

ros::Publisher pub;
cv::Mat image;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
pcl::PointCloud<pcl::PointXYZ> cloud;
pcl::fromROSMsg (*input, cloud);

// Reorder the point cloud to ensure it is in row-major format
pcl::PointCloud<pcl::PointXYZ> ordered_cloud;
ordered_cloud.width = cloud.width;
ordered_cloud.height = cloud.height;
ordered_cloud.points.resize(cloud.width * cloud.height);
for (int y = 0; y < cloud.height; y++) {
for (int x = 0; x < cloud.width; x++) {
ordered_cloud.at(x, y) = cloud.at(y * cloud.width + x);
}
}

// Convert the ordered point cloud to a grayscale image
image.create(ordered_cloud.height, ordered_cloud.width, CV_8UC1);
for (int y = 0; y < ordered_cloud.height; y++) {
for (int x = 0; x < ordered_cloud.width; x++) {
pcl::PointXYZ point = ordered_cloud.at(x, y);
// Map the height of the point to a grayscale value
int value = (point.z / 10.0) * 255;
image.at<unsigned char>(y, x) = std::max(0, std::min(255, value));
}
}

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
//ros::Subscriber sub = nh.subscribe ("os1_cloud_node/points", 1, cloud_cb);
ros::Subscriber sub = nh.subscribe("mmWaveDataHdl/RScan", 1, cloud_cb);

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

ros::Publisher pub;
cv::Mat image;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);
  
  // Convert the point cloud to a grayscale image
  image.create(cloud.height, cloud.width, CV_8UC1);
  for (int i = 0; i < cloud.points.size(); i++) {
    // Access the x, y, and z coordinates of the point
    float x = cloud.points[i].x;
    float y = cloud.points[i].y;
    float z = cloud.points[i].z;
    // Map the height of the point to a grayscale value
    int value = (z / 10.0) * 255;
    // Store the grayscale value in the image
    int row = i / cloud.width;
    int col = i % cloud.width;
    image.at<unsigned char>(row, col) = std::max(0, std::min(255, value));
  }

  // Resize the image to a specific size
  cv::resize(image, image, cv::Size(400, 200));

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
  //ros::Subscriber sub = nh.subscribe ("os1_cloud_node/points", 1, cloud_cb);

  // Advertise the image topic
  pub = nh.advertise<sensor_msgs::Image> ("output", 1);

  // Spin
  ros::spin ();
}
