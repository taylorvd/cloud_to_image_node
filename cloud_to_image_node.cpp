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
#include <math.h>
#include <cstdlib>
#include <ctime>

ros::Publisher pub;
cv::Mat image;

// Image size
int image_size = 16;

// FOV and range
float fov = 2.09; //rad, 120 deg;
float range = 10;



void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

  //creates new clean image every time
  //printf("OpenCV: %s", cv::getBuildInformation().c_str());
  image = cv::Mat();
  image = cv::Mat::zeros(image_size, image_size, CV_8UC1);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //changed to pointer
  pcl::fromROSMsg (*input, *cloud);

  // Calculate the cosine of the FOV angle
  //float cos_fov = cos(pcl::deg2rad(fov));

  // Create a unit z-vector pointing straight up
  Eigen::Vector3f z_vector(0, 0, 1);

  for (int i = 0; i < cloud->points.size(); i++) {
     int random_number = std::rand() % 100;

    // Get the xyz coordinates of the point
    Eigen::Vector3f point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);

    // Calculate the distance from the origin to the point
    float distance = point.norm();

    // Check if the point is within range
    if (distance > range || random_number <70) {
      continue;
    }

    // Calculate the angle between the point and the z-axis using the dot product
    //float cos_angle = point.dot(z_vector) / distance;
    double azimuth = std::atan2(point.y(), point.x());
    double elevation = std::atan2(point.z(), std::sqrt(point.x() * point.x() + point.y() * point.y()));
    //printf("azimuth %f elevation %f", azimuth, elevation);
    // Check if the point is within the FOV
    if (std::abs(azimuth) <= fov/2 && std::abs(elevation) <= fov/2){
  
      //https://towardsdatascience.com/spherical-projection-for-point-clouds-56a2fc258e6c
      int row_dim = 16;
      int col_dim = 16;
      float pitch = std::asin(point.z()/distance);
      float yaw = std::atan2(point.y(), point.x());
      float u = row_dim * (1-(pitch + fov/2)/(fov));
      float v = col_dim*(0.5*((yaw/(fov/2))+1));
      //printf("yaw %f, u %f, v %f\n", yaw, u, v);

      // Round the row and column indices to the nearest integer values
      int image_row = static_cast<int>(std::round(u));
      int image_col = static_cast<int>(std::round(v));  
      //printf("row %i, col %i\n", image_row, image_col);
      // Map the height of the point to a grayscale value
      int value =   (1 - (distance /range)) * 255;

      // Store the grayscale value in the image
      image_row = std::max(0, std::min(image_size , image_row));
      image_col = std::max(0, std::min(image_size , image_col));
      //printf("%i \n",  value);
      //printf("%i \n", std::max(0, std::min(255, value)));
 
      if (image.at<unsigned char>(image_row, col_dim - image_col) < value)
      {

        image.at<unsigned char>(image_row, col_dim - image_col) = std::max(0, std::min(255, value));
      }
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
  printf("starting node\n");

  // Subscribe to the point cloud topic
  //deep colliion predictor
  ros::Subscriber sub = nh.subscribe ("/delta/velodyne_points", 1, cloud_cb);
  //test
  //ros::Subscriber sub = nh.subscribe ("/ti_mmwave/radar_scan_pcl", 1, cloud_cb);
  //calibrate
  //ros::Subscriber sub = nh.subscribe ("/point_cloud", 1, cloud_cb);
  //cavern
  //ros::Subscriber sub = nh.subscribe ("/radar/left/cloud", 1, cloud_cb);
  
  // Advertise the image topic
  pub = nh.advertise<sensor_msgs::Image> ("output", 1);

  // Spin
  ros::spin ();
  // ros::Rate rate(10); // 10 Hz
  // while (ros::ok())
  // {
  //   ros::spinOnce();
  //   rate.sleep();
  // }
}