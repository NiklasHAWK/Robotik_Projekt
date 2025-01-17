#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"


image_transport::Publisher pub;

void imageCallback (const sensor_msgs::ImageConstPtr& msg)
{
	pub.publish(msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rectified_image_pub");
	ros::NodeHandle nh;
	
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("raspicam_node/image", 1, imageCallback);
	pub = it.advertise("robotik_projekt/images/rectified_image", 1);
	
	ros::spin();
	
	return 0;
}
