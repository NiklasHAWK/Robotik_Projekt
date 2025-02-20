#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
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
-0.7041802788118632, -1.449854024507943, 806.9806822887714,
0.02678154125035459, -3.767904210723072, 1825.201492912745,
1.622492389041975e-05, -0.003072405952626527, 1);


// Publisher für das birdseye_with_lidar-Bild
image_transport::Publisher birdseye_lidar_pub;

// Publisher für "Front"-Abstand und "360°"-Abstand
ros::Publisher obstacle_distance_front_pub;
ros::Publisher obstacle_distance_360_pub;

// Globaler Publisher für die Richtungsinformation (links/rechts)
ros::Publisher obstacle_direction_pub;

// Globaler Vektor mit Kamerabildpunkten
std::vector<cv::Point2f>lidar_coordinates_camera;


/// Callback-Funktion zur Verarbeitung der Punktwolke
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    std::vector<cv::Point2f> lidar_points;

    float min_distance_front = std::numeric_limits<float>::infinity();
    float min_distance_360   = std::numeric_limits<float>::infinity();
    float min_distance_left = std::numeric_limits<float>::infinity();
    float min_distance_right   = std::numeric_limits<float>::infinity();

    // Abstand Kamera nach oben zu lidar = 7cm
    // Abstand Kamera nach hinten zu lidar = 7.5cm
    cv::Mat T = (cv::Mat_<double>(4,4) << 
    0.0, -1.0,  0.0,   0.0,
    0.0,  0.0, -1.0,  -0.07,
    1.0,  0.0,  0.0,  -0.075,
    0.0,  0.0,  0.0,   1.0); 
    
    /*
    0.0, -1.0,  0.0,   0.0,
    0.0,  0.0, -1.0,  -0.056,
    1.0,  0.0,  0.0,  -0.075,
    0.0,  0.0,  0.0,   1.0); 

    */

	// Schleife durch alle Messwerte im Scan
	for (uint16_t i = 0; i < scan_msg->ranges.size(); i++)
	{
		// Berechne den Winkel für jedes Range-Element
		float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
		float range = scan_msg->ranges[i];
				    	
        // Überprüfe, ob der Messwert gültig ist
        if (!std::isfinite(range) || range < scan_msg->range_min)
            continue;

        // 1) Minimalen 360°-Abstand aktualisieren
        if (range < min_distance_360) 
        {
            min_distance_360 = range;
        }
        

        // Berechne die kartesischen Koordinaten
        if ((angle > 0.0 && angle < (M_PI / 4) || angle >= (7 * M_PI / 4) && angle <= (2 * M_PI)) && range < 0.65 )
        { 
            // minDistanceFront
            if (range < min_distance_front) 
            {
                min_distance_front = range;
            }

            double x = range * cos(angle);
            double y = range * sin(angle);
            
            // height of lidar above ground
            // for bird_eye_image height above ground, normal image height diff between camera
            double z = -0.168;
            // x,y,z auf y,z,x getauscht
            cv::Mat pointMat4D = (cv::Mat_<double>(4, 1) << x, y, z, 1.0);
            cv::Mat pointMat = T * pointMat4D;
            cv::Mat pointMat3D = (cv::Mat_<double>(3, 1) << pointMat.at<double>(0,0), pointMat.at<double>(1,0), pointMat.at<double>(2,0));

            cv::Mat transformedPoint = cameraMatrix * pointMat3D;
            cv::Point2f single_point((transformedPoint.at<double>(0, 0)/transformedPoint.at<double>(2, 0)),(transformedPoint.at<double>(1, 0)/transformedPoint.at<double>(2,0)));
            // Transformationsmatrix
            if (single_point.x >= 0 && single_point.x < 720 && single_point.y > 0 && single_point.y < 960)
            {
                lidar_points.push_back(single_point);
            }
        }
        else if (angle > (1* M_PI / 4) && angle < (3 * M_PI / 4))
        {
            if (range < min_distance_left) 
            {
                min_distance_left = range;
            }
        }
        else if (angle > (5 * M_PI / 4) && angle < (7 * M_PI / 4))
        {
            if (range < min_distance_right) 
            {
                min_distance_right = range;
            }
        }
        
    }

    lidar_coordinates_camera = lidar_points;

    // Publiziere den minimalen Front-Abstand
    std_msgs::Float32 msg_front;
    if (min_distance_front == std::numeric_limits<float>::infinity())
    {
        msg_front.data = -1.0;  // z. B. -1 => kein Hindernis in FRONT
    }
    else
    {
        msg_front.data = min_distance_front;
    }

    obstacle_distance_front_pub.publish(msg_front);
    //ROS_INFO("obstacle_distance_front: %f", msg_front.data);

    // Publiziere den minimalen 360°-Abstand
    std_msgs::Float32 msg_360;
    if (min_distance_360 == std::numeric_limits<float>::infinity())
    {
        msg_360.data = -1.0;  // kein Hindernis in 360°, sollte praktisch nie passieren
    }
    else
    {
        msg_360.data = min_distance_360;
    }

    obstacle_distance_360_pub.publish(msg_360);
    //ROS_INFO("obstacle_distance_360: %f", msg_360.data);

    // Publiziere die Richtung
    std_msgs::Float32 msg_direction;
    if (min_distance_right < min_distance_left)
    {
        msg_direction.data = 1.0;  // Richtung links
    }
    else
    {
        msg_direction.data = -1.0; // Richtung rechts
    }

    obstacle_direction_pub.publish(msg_direction);
	
	return;

}

// Callback-Funktion zur Verarbeitung des Bird's Eye View Bildes
void lidarBirdEyeCallback(const sensor_msgs::Image::ConstPtr& img_msg)
{
    // convert ROS message to opencv object
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);

    std::vector<cv::Point2f>lidar_coordinates_camera_test;


	// cv::Mat aus lidar_coordinates_camera
	// cv::Mat lidarbirdEyePoints;
	for (uint16_t i = 0; i < lidar_coordinates_camera.size(); i++) 
    {
        // Punkte in homogene Koordinaten umwandeln
        cv::Mat point = (cv::Mat_<double>(3, 1) << lidar_coordinates_camera[i].x, lidar_coordinates_camera[i].y, 1);
        
        // Transformation anwenden
        cv::Mat transformed_point = homographyMatrix * point;
        
        // Ergebnis zurück in 2D (normale Koordinaten) umwandeln
        //lidar_coordinates_camera[i] = cv::Point2f(transformed_point.at<double>(0,0)/transformed_point.at<double>(2,0), transformed_point.at<double>(1,0)/transformed_point.at<double>(2,0));

        lidar_coordinates_camera_test.push_back(cv::Point2f(transformed_point.at<double>(0,0)/transformed_point.at<double>(2,0),
        transformed_point.at<double>(1,0)/transformed_point.at<double>(2,0)));                                       
    }


	for(const auto& lidar_point : lidar_coordinates_camera_test)
    {
		circle(cv_ptr->image, cv::Point2f(lidar_point.x, lidar_point.y), 2, cv::Scalar(0, 0, 255), -1);
	}
    	
    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(img_msg->header, "bgr8", cv_ptr->image).toImageMsg();
    	
    birdseye_lidar_pub.publish(output_msg); 
	
	cv::imshow("birdseye_lidar_pub", cv_ptr->image);
	cv::waitKey(1);
	return;
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "birdseye_with_lidar_node");
    ros::NodeHandle nh;

    ROS_INFO("birdseye_with_lidar_node gestartet.");

    // ImageTransport
    image_transport::ImageTransport it(nh);

    // Subscriber (LaserScan + Bird-Eye-Bild)
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 1, scanCallback);
    image_transport::Subscriber birdseye_image_sub = it.subscribe("robotik_projekt/images/birdseye_image", 1, lidarBirdEyeCallback);

     // Publisher: Visualisiertes Bild
    birdseye_lidar_pub = it.advertise("robotik_projekt/images/birdseye_with_lidar_image", 1);

    // Publisher: 2 Abstände
    obstacle_distance_front_pub = nh.advertise<std_msgs::Float32>("robotik_projekt/obstacle/distance_front", 1);
    obstacle_distance_360_pub   = nh.advertise<std_msgs::Float32>("robotik_projekt/obstacle/distance_360", 1);

     // Erstelle den Publisher für die Richtungsinformation:
     obstacle_direction_pub = nh.advertise<std_msgs::Float32>("robotik_projekt/obstacle/direction", 1);

    ros::spin();
    return 0;
}
