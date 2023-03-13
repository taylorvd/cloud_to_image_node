#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/common.h>

ros::Publisher pub;
cv::Mat image;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  
  image = cv::Mat();
  image = cv::Mat::zeros(15, 15, CV_8UC1);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);
  
   // Compute the bounding box of the point cloud
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(cloud, min_pt, max_pt);

  // Convert the point cloud to a grayscale image
  //image.create(cloud.height, cloud.width, CV_8UC1);
  
  //image.create(350,350, CV_8UC1);
  //printf("height %u width %u \n", cloud.height, cloud.width);
  for (int i = 0; i < cloud.points.size(); i++) {
    // Access the x, y, and z coordinates of the point
    float x = cloud.points[i].x;
    float y = cloud.points[i].y;
    float z = cloud.points[i].z;
    // Map the height of the point to a grayscale value
    int value = (z / 10.0) * 255;
    
    // Store the grayscale value in the image
    int row = ((x - min_pt.x) / (max_pt.x - min_pt.x) * 14); //static_cast<int>(x*10); //i / cloud.width;
    int col = ((y - min_pt.y) / (max_pt.y - min_pt.y) * 14);//static_cast<int>(y*10); //i % cloud.width;
    printf("row %i x %f col %i y %f \n", row, x, col, y);
    
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
