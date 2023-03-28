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

// Image size
int image_size = 10;

// FOV and range
float fov = 120;
float range = 10;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  //creates new clean image every time
  image = cv::Mat();
  image = cv::Mat::zeros(image_size, image_size, CV_8UC1);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //changed to pointer
  pcl::fromROSMsg (*input, *cloud);

  // Calculate the cosine of the FOV angle
  float cos_fov = cos(pcl::deg2rad(fov));

  // Create a unit z-vector pointing straight up
  Eigen::Vector3f z_vector(0, 0, 1);

  for (int i = 0; i < cloud->points.size(); i++) {
    // Get the xyz coordinates of the point
    Eigen::Vector3f point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);

    // Calculate the distance from the origin to the point
    float distance = point.norm();

    // Check if the point is within range
    if (distance > range) {
      continue;
    }

    // Calculate the angle between the point and the z-axis using the dot product
    float cos_angle = point.dot(z_vector) / distance;

    // Check if the point is within the FOV
    if (cos_angle >= cos_fov) {
      // Map the x, y coordinates of the point to the image range
      int row = ((point.x() + range) / (2 * range) * image_size);
      int col = ((point.y() + range) / (2 * range) * image_size);

      // Map the height of the point to a grayscale value
      int value = (point.z() / range) * 255;

      // Store the grayscale value in the image
      row = std::max(0, std::min(image_size - 1, row));
      col = std::max(0, std::min(image_size - 1, col));
      image.at<unsigned char>(row, col) = std::max(0, std::min(255, value));
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
  //ros::Subscriber sub = nh.subscribe ("mmWaveDataHdl/RScan", 1, cloud_cb);
  ros::Subscriber sub = nh.subscribe ("/ti_mmwave/radar_scan_pcl", 1, cloud_cb);

  // Advertise the image topic
  pub = nh.advertise<sensor_msgs::Image> ("output", 1);

  // Spin
  ros::spin ();
}