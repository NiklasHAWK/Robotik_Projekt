#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"


image_transport::Publisher pub;
cv::Mat map1, map2;

void rectifyImageCallback (const sensor_msgs::ImageConstPtr& msg)
{
	// convert ROS message to opencv object
    	cv_bridge::CvImagePtr cv_ptr;
    	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    	
    	// Rectify the image
	cv::Mat undistortedImage = cv::Mat_<double>(960, 720);
	cv::remap(cv_ptr->image, undistortedImage, map1, map2, cv::INTER_LINEAR);
	sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(msg->header, "bgr8", undistortedImage).toImageMsg();
	
	pub.publish(output_msg);
	ROS_INFO("Bild gepublished.");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rectified_image_pub");
	ros::NodeHandle nh;
	
	// Create Rectifying Map
	cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 843.074971, 0, 478.388183, 0, 844.236343, 333.350289, 0, 0, 1);
	cv::Mat identityMatrix = cv::Mat::eye(3, 3, CV_64F);
    	cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << 0.096351, -0.094708, -0.001249, -0.000937, 0);
    	cv::Size imageSize(960, 720);
    	ROS_INFO("Matrizen und Vektoren erstellt.");
    	
    	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, identityMatrix, cameraMatrix, imageSize, CV_32FC1, map1, map2);
	ROS_INFO("Rektifizierungs Maps erstellt.");
	
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("raspicam_node/image", 1, rectifyImageCallback);
	pub = it.advertise("robotik_projekt/images/rectified_image", 1);
	
	ros::spin();
	
	return 0;
}
