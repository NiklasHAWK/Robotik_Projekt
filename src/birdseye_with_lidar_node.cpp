#include <ros/ros.h>
#include "image_transport/image_transport.h"
#include <sensor_msgs/Image.h>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


// Globale Variablen für Publisher und Daten
image_transport::Publisher birdseye_lidar_pub;
laser_geometry::LaserProjection projector;
cv::Mat current_bird_eye; // Das aktuell abonnierte Bird's Eye View

/// Callback-Funktion zur Verarbeitung der Punktwolke
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{

    // Parameter für den gewünschten Sichtbereich
    const double fov_half_angle = 45.0 * M_PI / 180.0; // 70 Grad Sichtfeld, halbes Sichtfeld = 35 Grad
    const double camera_angle_center = 0.0; // Blickrichtung der Kamera in Radianten (z. B. nach vorne)

    // Neue ranges für gefilterte Daten
    std::vector<float> filtered_ranges(scan_msg->ranges.size(), std::numeric_limits<float>::quiet_NaN());

    // Winkel und Reichweite filtern
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        double angle = scan_msg->angle_min + i * scan_msg->angle_increment;

        // Prüfen, ob der Winkel innerhalb des Sichtfelds liegt
        if (angle >= camera_angle_center - fov_half_angle && angle <= camera_angle_center + fov_half_angle) {
            filtered_ranges[i] = scan_msg->ranges[i]; // Punkt beibehalten
        }
    }

    // Neue LaserScan-Nachricht erstellen
    sensor_msgs::LaserScan filtered_scan = *scan_msg;
    filtered_scan.ranges = filtered_ranges;

    // Schritt 1: LaserScan in PointCloud2 umwandeln
    ROS_INFO("Schritt 1: LaserScan in PointCloud2 umwandeln.");
    sensor_msgs::PointCloud2 cloud_msg;
    projector.projectLaser(filtered_scan, cloud_msg);

    // Schritt 2: PointCloud2 in pcl::PointCloud konvertieren
    ROS_INFO("Schritt 2: PointCloud2 in pcl::PointCloud konvertieren.");
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(cloud_msg, cloud);

    // Flippen der Punktwolke (horizontal und vertikal)
    for (auto& point : cloud.points) 
    {
        point.x = -point.x;  // Horizontal flippen
        //point.y = -point.y;  // Vertikal flippen
        // z bleibt unverändert, da es nicht Teil der Orientierung ist (falls benötigt, ebenfalls anpassen)
    }
    for (auto& point : cloud.points) 
    {
        // 90-Grad-Rotation gegen den Uhrzeigersinn
        float new_x = -point.y; // x' = -y
        float new_y = point.x;  // y' = x
        point.x = new_x;
        point.y = new_y;
    }

    // Schritt 3: Überprüfen, ob ein Bird's Eye View Bild existiert
    ROS_INFO("Schritt 3: Überprüfen, ob ein Bird's Eye View Bild existiert.");
    if (current_bird_eye.empty())
    {
        ROS_WARN("No Bird's Eye View image received yet. Skipping...");
        return;
    }

    // Schritt 4: Punkte in das aktuelle Bird's Eye View einzeichnen
    ROS_INFO("Schritt 4: Punkte in das aktuelle Bird's Eye View einzeichnen.");
    cv::Mat bird_eye_with_points = current_bird_eye.clone(); // Arbeitskopie
    // Skalierungsfaktor
    float scale_x = 200.0f;  // z.B. 1.0 für keine Skalierung, oder 2.0 um die Punkte zu vergrößern (50)
    float scale_y = 200.0f;  // z.B. 1.0 für keine Skalierung, oder 2.0 um die Punkte zu vergrößern (50)

    // Offset
    float offset_x = bird_eye_with_points.cols / 2; // z.B. 0.5, um alle Punkte um 0.5 in x-Richtung zu verschieben (bird_eye_with_points.cols / 2)
    float offset_y = bird_eye_with_points.rows / 2; // z.B. -0.3, um alle Punkte um -0.3 in y-Richtung zu verschieben (bird_eye_with_points.rows / 2)

    for (const auto& point : cloud.points)
    {
        int x = static_cast<int>(point.x * scale_x + offset_x); // Maßstab und Offset
        int y = static_cast<int>(point.y * scale_y + offset_y);

        if (x >= 0 && x < bird_eye_with_points.cols && y >= 0 && y < bird_eye_with_points.rows)
        {
            bird_eye_with_points.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0); // Grün für Punkte
        }
    }

    cv::imshow("Bearbeitetes Birds_Eye", bird_eye_with_points);
    cv::waitKey(1);

    // Schritt 5: Bild als sensor_msgs/Image veröffentlichen
    ROS_INFO("Schritt 5: Bild als sensor_msgs/Image veröffentlichen.");
    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(scan_msg->header, "bgr8", bird_eye_with_points).toImageMsg();
	
	birdseye_lidar_pub.publish(output_msg);
	ROS_INFO("birdseye_with_lidar_image gepublished.");
}
// Callback-Funktion zur Verarbeitung des Bird's Eye View Bildes
void birdEyeImageCallback(const sensor_msgs::Image::ConstPtr& img_msg)
{
    try
    {
        ROS_INFO("Konvertiere sensor_msgs/Image in OpenCV-Matrix.");
        // Konvertiere sensor_msgs/Image in OpenCV-Matrix
        current_bird_eye = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Failed to convert Bird's Eye View image: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "birdseye_with_lidar_node");
    ros::NodeHandle nh;

    // Subscriber für LaserScan und Bird's Eye View Bild
    ros::Subscriber laser_sub = nh.subscribe("/scan", 10, scanCallback);
    ros::Subscriber birdseye_image_sub = nh.subscribe("robotik_projekt/images/birdseye_image", 1, birdEyeImageCallback);
    
    image_transport::ImageTransport it(nh);
    birdseye_lidar_pub = it.advertise("robotik_projekt/images/birdseye_with_lidar_image", 1);

    ros::spin();
    return 0;
}
