#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "sensor_msgs/LaserScan.h"
//#include "sensor_msgs/PointCloud.h"
//#include "laser_geometry/laser_geometry.h"
//#include "pcl/point_cloud.h"
//#include "pcl/point_types.h"
//#include "pcl_conversions/pcl_conversions.h"
//#include "pcl/common/transforms.h"
#include "math.h"



// Kameramatrix
cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
843.074971, 0.0, 478.388183, 
0.0, 844.236343, 333.350289, 
0.0, 0.0, 1.0);


// Homographie
cv::Mat homographyMatrix = (cv::Mat_<double>(3, 3) << 
-0.6844031534226153, -1.450545517122974, 797.897835011137,
 0.02678153217836598, -3.76790286931926, 1825.200883354766,
 1.622492288698972e-05, -0.003072405300756105, 1);


// Globale Variablen für Publisher und Daten
image_transport::Publisher birdseye_lidar_pub;

// Globaler Vektor mit Kamerabildpunkten
std::vector<cv::Point2f>lidar_coordinates_camera;


/// Callback-Funktion zur Verarbeitung der Punktwolke
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    std::vector<cv::Point2f> lidar_points;
	cv::Mat transformedPoint;
	// Schleife durch alle Messwerte im Scan
	for (uint16_t i = 0; i < scan_msg->ranges.size(); i++)
	{
		
		// Berechne den Winkel für jedes Range-Element
		float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
		float range = scan_msg->ranges[i];
				    	
        // Abstand Kamera nach oben zu lidar = 7cm
        // Abstand Kamera nach hinten zu lidar = 7.5cm
        cv::Mat T = (cv::Mat_<double>(4,4) << 
        0.0, -1.0,  0.0,   0.0,
        0.0,  0.0, -1.0,  -0.056,
        1.0,  0.0,  0.0,  -0.075,
        0.0,  0.0,  0.0,   1.0); 

        // Berechne die kartesischen Koordinaten
        if ((angle > 0 && angle < 0.78539816339 || angle >= 5.49778714378 && angle <= 6.28318530718) && range < 0.65 )
        { 
            double x = range * cos(angle);
            double y = range * sin(angle);
            
            // height of lidar above ground
            // for bird_eye_image height above ground, normal image height diff between camera
            double z = -0.168;
            // x,y,z auf y,z,x getauscht
            cv::Mat pointMat4D = (cv::Mat_<double>(4, 1) << x, y, z, 1.0);
            cv::Mat pointMat = T * pointMat4D;
            cv::Mat pointMat3D = (cv::Mat_<double>(3, 1) << pointMat.at<double>(0,0), pointMat.at<double>(1,0), pointMat.at<double>(2,0));

            transformedPoint = cameraMatrix * pointMat3D;
            cv::Point2f single_point((transformedPoint.at<double>(0, 0)/transformedPoint.at<double>(2, 0)),(transformedPoint.at<double>(1, 0)/transformedPoint.at<double>(2,0)));
            // Transformationsmatrix
            if (single_point.x >= 0 && single_point.x < 720 && single_point.y > 0 && single_point.y < 960)
            {
                lidar_points.push_back(single_point);
            }
        }
    }

    lidar_coordinates_camera = lidar_points;	
	
	return;

}

// Callback-Funktion zur Verarbeitung des Bird's Eye View Bildes
void lidarBirdEyeCallback(const sensor_msgs::Image::ConstPtr& img_msg)
{
    // convert ROS message to opencv object
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);


	// cv::Mat aus lidar_coordinates_camera
	// cv::Mat lidarbirdEyePoints;
	for (uint16_t i = 0; i < lidar_coordinates_camera.size(); i++) 
    {
        // Punkte in homogene Koordinaten umwandeln
        cv::Mat point = (cv::Mat_<double>(3, 1) << lidar_coordinates_camera[i].x, lidar_coordinates_camera[i].y, 1);
        
        // Transformation anwenden
        cv::Mat transformed_point = homographyMatrix * point;
        
        // Ergebnis zurück in 2D (normale Koordinaten) umwandeln
        lidar_coordinates_camera[i] = cv::Point2f(transformed_point.at<double>(0,0)/transformed_point.at<double>(2,0),
                                                   transformed_point.at<double>(1,0)/transformed_point.at<double>(2,0));
    }


	for(const auto& lidar_point : lidar_coordinates_camera)
    {
		circle(cv_ptr->image, cv::Point2f(lidar_point.x, lidar_point.y), 2, cv::Scalar(0, 0, 255), -1);
	}
    	
    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(img_msg->header, "bgr8", cv_ptr->image).toImageMsg();
    	
    birdseye_lidar_pub.publish(output_msg); 
    	
	
	
	
	cv::imshow("test", cv_ptr->image);
	cv::waitKey(1);
	return;
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "birdseye_with_lidar_node");
    ros::NodeHandle nh;

    // Subscriber für LaserScan und Bird's Eye View Bild
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 1, scanCallback);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber birdseye_image_sub = it.subscribe("robotik_projekt/images/birdseye_image", 1, lidarBirdEyeCallback);
    birdseye_lidar_pub = it.advertise("robotik_projekt/images/birdseye_with_lidar_image", 1);

    ros::spin();
    return 0;
}
