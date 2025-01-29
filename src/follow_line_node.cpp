#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include <geometry_msgs/Twist.h>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgproc/imgproc.hpp>  // Für Bildbearbeitungsfunktionen
#include <opencv2/highgui/highgui.hpp>  // Für Imshow und Window-Funktionen
#include <cmath>



ros::Publisher cmd_vel_pub;

static int last_middle_x = 480; 


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Konvertiere ROS-Bild in OpenCV-Format
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
/*
    cv::Mat image = cv_ptr->image;


    // Konvertiere in Graustufen
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    // Canny-Kantendetektion
    cv::Mat edges;
    cv::Canny(gray, edges, 50, 150);

    // Hough-Linien-Transformation
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 50, 10);

    // Finde linke und rechte Spurmarkierung
    std::vector<cv::Vec4i> left_lines, right_lines;
    int image_center = img.cols / 2;
    
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];
        int x1 = l[0], y1 = l[1], x2 = l[2], y2 = l[3];

        // Berechne die mittlere X-Position der Linie
        int x_avg = (x1 + x2) / 2;

        // Sortiere die Linien in linke und rechte Spurlinie
        if (x_avg < image_center) {
            left_lines.push_back(l);
        } else {
            right_lines.push_back(l);
        }
    }

    // Berechne die mittlere Position der linken und rechten Linie
    int left_x = 0, right_x = img.cols;
    if (!left_lines.empty()) {
        int sum_x = 0;
        for (const auto& l : left_lines) {
            sum_x += (l[0] + l[2]) / 2;
        }
        left_x = sum_x / left_lines.size();
    }

    if (!right_lines.empty()) {
        int sum_x = 0;
        for (const auto& l : right_lines) {
            sum_x += (l[0] + l[2]) / 2;
        }
        right_x = sum_x / right_lines.size();
    }

    // Berechne den mittleren Punkt der Spur
    int lane_center = (left_x + right_x) / 2;
    int error = lane_center - image_center;

    // Steuerungssignal berechnen
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.2;  // Konstante Vorwärtsgeschwindigkeit
    cmd_vel.angular.z = -0.002 * error;  // Proportionalregelung für Lenkung

    // Begrenzung der Drehgeschwindigkeit
    if (cmd_vel.angular.z > 0.5) cmd_vel.angular.z = 0.5;
    if (cmd_vel.angular.z < -0.5) cmd_vel.angular.z = -0.5;

    cmd_vel_pub.publish(cmd_vel);

    // Debug-Ausgabe
    cv::line(img, cv::Point(left_x, 0), cv::Point(left_x, img.rows), cv::Scalar(255, 0, 0), 2);
    cv::line(img, cv::Point(right_x, 0), cv::Point(right_x, img.rows), cv::Scalar(0, 0, 255), 2);
    cv::line(img, cv::Point(lane_center, 0), cv::Point(lane_center, img.rows), cv::Scalar(0, 255, 0), 2);

    cv::imshow("Lane Detection", img);
    cv::waitKey(1);

    */


   // In HSV-Farbraum konvertieren
    cv::Mat img = cv_ptr->image;
    cv::Mat hsv, mask;

    /*int roi_height = img.rows / 3;  // Nutze nur das untere Drittel
    cv::Rect roi(0, img.rows - roi_height, img.cols, roi_height);
    cv::Mat img_roi = img(roi);
    */
    // In HSV umwandeln
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    // Gelb-Farbsegmentierung
    cv::Scalar lower_yellow(20, 0, 0);
    cv::Scalar upper_yellow(70, 255, 255);
    cv::inRange(hsv, lower_yellow, upper_yellow, mask);

    // Konturen finden
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Konturen ins Bild zeichnen (Rot)
    //cv::drawContours(img, contours, -1, cv::Scalar(0, 0, 255), 2);

    std::vector<int> line_centers;

    int img_center = img.cols / 2;
    int middle_x = img_center;
    
    if(contours.size() > 0){
        for (const auto& contour : contours) {
            cv::Rect bounding_box = cv::boundingRect(contour);
            int x_center = bounding_box.x + bounding_box.width / 2;
            line_centers.push_back(x_center);
        }
        

        // Sortiere x-Koordinaten der Linien
        std::sort(line_centers.begin(), line_centers.end());

        if (line_centers.size() >= 2) {
            
            middle_x = (line_centers[0] + line_centers[1]) / 2;
            last_middle_x = middle_x;
        }
        else {
            // Falls keine Linien gefunden wurden, verwende den letzten Wert
            middle_x = last_middle_x;

        }
    }


    int error = middle_x - img_center;

    // Steuerbefehl berechnen
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.08;  // Konstante Vorwärtsgeschwindigkeit
    cmd_vel.angular.z = -0.001 * error;  // Proportionalregelung für Lenkung

    // Begrenzung der Drehgeschwindigkeit
    cmd_vel.angular.z = std::max(std::min(cmd_vel.angular.z, 0.5), -0.5);

    cmd_vel_pub.publish(cmd_vel);

    // Debug-Anzeige
    if (line_centers.size() > 0) {
        cv::line(img, cv::Point(line_centers[0], 0), cv::Point(line_centers[0], img.rows), cv::Scalar(255, 0, 0), 2);
        cv::line(img, cv::Point(line_centers[1], 0), cv::Point(line_centers[1], img.rows), cv::Scalar(255, 0, 0), 2);
        cv::line(img, cv::Point(middle_x, 0), cv::Point(middle_x, img.rows), cv::Scalar(0, 255, 0), 2);
    }

    cv::imshow("Lane Detection", img);
    cv::imshow("Maske", mask);
    cv::imshow("HSV", hsv);
    //cv::imshow("img_roi", img_roi);
    cv::waitKey(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "follow_line_node");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("robotik_projekt/images/birdseye_image", 1, imageCallback);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    
    ros::spin(); 
    
    return 0;
}
