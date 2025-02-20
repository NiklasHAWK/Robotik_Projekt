#include "ros/ros.h" 							// ROS Hauptbibliothek für Node-Management
#include "image_transport/image_transport.h" 	// Bibliothek für den Transport von Bildern in ROS
#include "sensor_msgs/CompressedImage.h" 		// Nachrichtentyp für komprimierte Bilder
#include "opencv2/opencv.hpp" 					// OpenCV-Bibliothek für Bildverarbeitung
#include "cv_bridge/cv_bridge.h" 				// cv_bridge für die Konvertierung zwischen ROS und OpenCV-Bildern


// Publisher für das entzerrte Bild
image_transport::Publisher rectified_image_pub;

// Maps zur Bildentzerrung (Remapping)
cv::Mat map_1, map_2;

// Callback-Funktion zur Verarbeitung eingehender Bilder
void rectifyImageCallback (const sensor_msgs::CompressedImageConstPtr& msg)
{
	// Konvertieren einer ROS-Nachricht in ein OpenCV-Bild
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    	
    // Erstellen einer leeren Matrix für das entzerrte Bild
	cv::Mat undistorted_image = cv::Mat_<double>(960, 720);
	
    // Anwenden der Remap-Funktion, um das Bild zu entzerren
	cv::remap(cv_ptr->image, undistorted_image, map_1, map_2, cv::INTER_LINEAR);
	
    // OpenCV-Bild zurück in eine ROS-Nachricht konvertieren
	sensor_msgs::ImagePtr rectified_image_msg = cv_bridge::CvImage(msg->header, "bgr8", undistorted_image).toImageMsg();
	
	// Veröffentlichen des entzerrten Bildes
	rectified_image_pub.publish(rectified_image_msg);
}

int main(int argc, char **argv)
{
	// Initialisierung des ROS-Nodes
	ros::init(argc, argv, "rectified_image_pub");
	ros::NodeHandle nh;
	
	// Erstellung der Maps für die Bildentzerrung (Remapping)
	cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 843.074971, 0, 478.388183, 0, 844.236343, 333.350289, 0, 0, 1);
    cv::Mat distort_coefficient = (cv::Mat_<double>(5, 1) << 0.096351, -0.094708, -0.001249, -0.000937, 0);
    cv::Mat identity_matrix = cv::Mat::eye(3, 3, CV_64F);
	cv::Size imageSize(960, 720); 
	
    // Berechnung der Mapping-Tabellen für die Entzerrung
    cv::initUndistortRectifyMap(camera_matrix, distort_coefficient, identity_matrix, camera_matrix, imageSize, CV_32FC1, map_1, map_2);
	
	// Erstellung eines ImageTransport für den NodeHandle
	image_transport::ImageTransport it(nh);
	
	// Abonnieren des Topics mit komprimierten Bildern der Kamera
	ros::Subscriber sub = nh.subscribe("raspicam_node/image/compressed", 1, rectifyImageCallback);
	
	// Erstellung eines Publishers für das entzerrte Bild
	rectified_image_pub = it.advertise("robotik_projekt/images/rectified_image", 1);
	
	// ROS-Loop zur Verarbeitung eingehender Nachrichten
	ros::spin();
	
	return 0;
}
