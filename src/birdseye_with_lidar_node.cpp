#include <ros/ros.h> 							// ROS Hauptbibliothek für Node-Management
#include <opencv2/opencv.hpp> 					// OpenCV-Bibliothek für Bildverarbeitung
#include <opencv2/imgproc.hpp> 					// OpenCV-Bibliothek für Bildverarbeitung (Bildmanipulation)
#include <cv_bridge/cv_bridge.h> 				// cv_bridge für die Konvertierung zwischen ROS und OpenCV-Bildern
#include <image_transport/image_transport.h> 	// Bibliothek für den Transport von Bildern in ROS
#include <sensor_msgs/Image.h> 					// Nachrichtentyp für Bilddaten
#include <sensor_msgs/LaserScan.h> 				// Nachrichtentyp für Lidar-Scans
#include <std_msgs/Float32.h> 					// Nachrichtentyp für Float-Werte
#include <math.h> 								// Mathematische Funktionen

// Kameramatrix für die Projektionsberechnungen
cv::Mat CAMERA_MATRIX = (cv::Mat_<double>(3, 3) << 
843.074971, 0.0, 478.388183, 
0.0, 844.236343, 333.350289, 
0.0, 0.0, 1.0);

// Homographie-Matrix für die Perspektivtransformation
cv::Mat HOMOGRAPHY_MATRIX = (cv::Mat_<double>(3, 3) << 
-0.7041802788118632, -1.449854024507943, 806.9806822887714,
0.02678154125035459, -3.767904210723072, 1825.201492912745,
1.622492389041975e-05, -0.003072405952626527, 1);

// Transformationsmatrix zur Anpassung der Lidar-Koordinaten an das Kamerasystem
cv::Mat TRANSFORMATION_MATRIX = (cv::Mat_<double>(4,4) << 
0.0, -1.0,  0.0,   0.0,
0.0,  0.0, -1.0,  -0.056,    // Lidar ist 5,6cm höher als die Kamera
1.0,  0.0,  0.0,  -0.075,   // Lidar ist 7,5cm hinter der Kamera
0.0,  0.0,  0.0,   1.0); 

// Publisher für das Bird's Eye View Bild mit Lidar-Punkten
image_transport::Publisher birds_eye_lidar_pub;

// Publisher für die Hindernisabstände
ros::Publisher obstacle_distance_front_pub;
ros::Publisher obstacle_distance_360_pub;

// Publisher für die Richtungsinformation (links/rechts)
ros::Publisher turn_direction_pub;

// Vektor mit Lidar-Kamerabildpunkten
std::vector<cv::Point2f>lidar_coordinates_camera;


// Callback-Funktion zur Verarbeitung der Punktwolke (Lidar-Daten)
void lidarScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    std::vector<cv::Point2f> lidar_points;

    // Initialisieren der minimalen Distanzen für verschiedene Richtungen
    float min_distance_front    = std::numeric_limits<float>::infinity();
    float min_distance_360      = std::numeric_limits<float>::infinity();
    float min_distance_left     = std::numeric_limits<float>::infinity();
    float min_distance_right    = std::numeric_limits<float>::infinity();

	// Verarbeitung aller Messwerte aus dem Scan
	for (uint16_t i = 0; i < scan_msg->ranges.size(); i++)
	{
		// Berechnung des Winkels für jeden Messpunkt
		float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
		float range = scan_msg->ranges[i];
				    	
        // Überprüfung, ob der Messwert gültig ist
        if (!std::isfinite(range) || range < scan_msg->range_min)
            continue;

         // Aktualisieren des minimalen 360°-Abstands
        if (range < min_distance_360) 
        {
            min_distance_360 = range;
        }
        
        // Berechnung der kartesischen Koordinaten der Lidar-Punkte nach vorne
        if ((angle > 0.0 && angle < (M_PI / 4) || angle >= (7 * M_PI / 4) && angle <= (2 * M_PI)) && range < 0.65 )
        { 
            // Minimalen Front-Abstand aktualisieren
            if (range < min_distance_front) 
            {
                min_distance_front = range;
            }

            double x = range * cos(angle);
            double y = range * sin(angle);
            double z = -0.168; // Höhe des Lidars über dem Boden

            // Transformation der Lidar-Koordinaten ins Kamerasystem
            cv::Mat point_mat_4D = (cv::Mat_<double>(4, 1) << x, y, z, 1.0);
            cv::Mat point_mat = TRANSFORMATION_MATRIX * point_mat_4D;
            cv::Mat point_mat_3D = (cv::Mat_<double>(3, 1) << point_mat.at<double>(0,0), point_mat.at<double>(1,0), point_mat.at<double>(2,0));

            cv::Mat transformed_point = CAMERA_MATRIX * point_mat_3D;
            cv::Point2f single_point((transformed_point.at<double>(0, 0)/transformed_point.at<double>(2, 0)),(transformed_point.at<double>(1, 0)/transformed_point.at<double>(2,0)));

            // Überprüfung, ob die Koordinaten im Bildbereich liegen
            if (single_point.x >= 0 && single_point.x < 720 && single_point.y > 0 && single_point.y < 960)
            {
                lidar_points.push_back(single_point);
            }
        }
        // Minimalen Abstand links aktualisieren
        else if (angle > (1* M_PI / 4) && angle < (3 * M_PI / 4))
        {
            if (range < min_distance_left) 
            {
                min_distance_left = range;
            }
        }
        // Minimalen Abstand rechts aktualisieren
        else if (angle > (5 * M_PI / 4) && angle < (7 * M_PI / 4))
        {
            if (range < min_distance_right) 
            {
                min_distance_right = range;
            }
        }
    }

    lidar_coordinates_camera = lidar_points;

    // Publizieren des minimalen Front-Abstand
    std_msgs::Float32 msg_front;
    if (min_distance_front == std::numeric_limits<float>::infinity())
    {
        msg_front.data = -1.0;  // kein Hindernis in Fahrtrichtung
    }
    else
    {
        msg_front.data = min_distance_front;
    }
    obstacle_distance_front_pub.publish(msg_front);

    // Publizieren des minimalen 360°-Abstand
    std_msgs::Float32 msg_360;
    if (min_distance_360 == std::numeric_limits<float>::infinity())
    {
        msg_360.data = -1.0;  // kein Hindernis in 360°
    }
    else
    {
        msg_360.data = min_distance_360;
    }
    obstacle_distance_360_pub.publish(msg_360);

    // Publizieren der Richtung (links/rechts)
    std_msgs::Float32 msg_direction;
    if (min_distance_right < min_distance_left) // Wenn die Entfernung zum Hindernis auf der rechten Seite < als auf der linken Seite
    {
        msg_direction.data = 1.0;  // Drehrichtung links
    }
    else
    {
        msg_direction.data = -1.0; // Drehrichtung rechts
    }
    turn_direction_pub.publish(msg_direction);
	
	return;

}

// Callback-Funktion zur Verarbeitung des Bird's Eye View Bildes
void lidarBirdEyeCallback(const sensor_msgs::Image::ConstPtr& image_msg)
{
    // Konvertieren der ROS-Bildnachricht in OpenCV-Bild
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

    std::vector<cv::Point2f>lidar_points_birds_eye;


    // Transformation der Lidar-Koordinaten ins Bird's Eye View Bild
	for (uint16_t i = 0; i < lidar_coordinates_camera.size(); i++) 
    {
        // Punkte in homogene Koordinaten umwandeln
        cv::Mat hom_point = (cv::Mat_<double>(3, 1) << lidar_coordinates_camera[i].x, lidar_coordinates_camera[i].y, 1);
        
        // Transformation anwenden
        cv::Mat transformed_point = HOMOGRAPHY_MATRIX * hom_point;

        // Umwandeln der transformierten homogenen Koordinaten in normale 2D-Koordinaten
        lidar_points_birds_eye.push_back(cv::Point2f(transformed_point.at<double>(0,0)/transformed_point.at<double>(2,0),
        transformed_point.at<double>(1,0)/transformed_point.at<double>(2,0)));                                       
    }

    // Zeichnen der Lidar-Punkte ins Bild
	for(const auto& lidar_point : lidar_points_birds_eye)
    {
		circle(cv_ptr->image, cv::Point2f(lidar_point.x, lidar_point.y), 2, cv::Scalar(0, 0, 255), -1);
	}
    
    // OpenCV-Bild zurück in eine ROS-Nachricht konvertieren
    sensor_msgs::ImagePtr birds_eye_image_with_lidar_msg = cv_bridge::CvImage(image_msg->header, "bgr8", cv_ptr->image).toImageMsg();
    
    // Veröffentlichen des Bildes mit Vogelperspektive und Lidarpunkten
    birds_eye_lidar_pub.publish(birds_eye_image_with_lidar_msg); 
	
	cv::imshow("birds_eye_lidar_pub", cv_ptr->image);
	cv::waitKey(1);

	return;
}

int main(int argc, char** argv) 
{
    // Initialisierung des ROS-Nodes
    ros::init(argc, argv, "birdseye_with_lidar_node");
    ros::NodeHandle nh;

    ROS_INFO("birdseye_with_lidar_node gestartet.");

    // Erstellung eines ImageTransport für den NodeHandle
    image_transport::ImageTransport it(nh);

    // Subscriber für Lidar-Scandaten
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 1, lidarScanCallback);

    // Subscriber für das Bird's Eye View Bild
    image_transport::Subscriber birds_eye_image_sub = it.subscribe("robotik_projekt/images/birds_eye_image", 1, lidarBirdEyeCallback);

    // Publisher für das Bird's Eye View Bild mit Lidar-Punkten
    birds_eye_lidar_pub = it.advertise("robotik_projekt/images/birdseye_with_lidar_image", 1);

    // Publisher für den minimalen Hindernisabstand nach vorne
    obstacle_distance_front_pub = nh.advertise<std_msgs::Float32>("robotik_projekt/obstacle/distance_front", 1);
    
    // Publisher für den minimalen Hindernisabstand in 360°
    obstacle_distance_360_pub   = nh.advertise<std_msgs::Float32>("robotik_projekt/obstacle/distance_360", 1);

    // Publisher für die Richtungsinformation (gibt an, ob links oder rechts mehr Platz ist)
    turn_direction_pub = nh.advertise<std_msgs::Float32>("robotik_projekt/obstacle/turn_direction", 1);

    // ROS-Loop zur Verarbeitung eingehender Nachrichten
    ros::spin();

    return 0;
}