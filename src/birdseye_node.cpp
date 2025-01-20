#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // Konvertiere ROS-Bildnachricht in OpenCV-Bild
        cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

        // Schachbrettparameter
        const int board_w = 7; // Anzahl der Ecken pro Zeile
        const int board_h = 7; // Anzahl der Ecken pro Spalte
        const cv::Size board_sz(board_w, board_h);
        const int board_n = board_w * board_h;

        // Suche nach Schachbrettecken
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(image, board_sz, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);

        if (!found) {
            ROS_WARN("Couldn't acquire chessboard, only found %lu of %d corners.", corners.size(), board_n);
            return;
        }

        // Verfeinerung der Ecken auf Subpixel-Genauigkeit
        cv::Mat gray_image;
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1));

        // Definiere Bild- und Objektpunkte
        std::vector<cv::Point2f> imgPts(4);
        std::vector<cv::Point2f> objPts(4);
        objPts[0] = cv::Point2f(0, 0);
        objPts[1] = cv::Point2f(board_w - 1, 0);
        objPts[2] = cv::Point2f(0, board_h - 1);
        objPts[3] = cv::Point2f(board_w - 1, board_h - 1);
        imgPts[0] = corners[0];
        imgPts[1] = corners[board_w - 1];
        imgPts[2] = corners[(board_h - 1) * board_w];
        imgPts[3] = corners[(board_h - 1) * board_w + board_w - 1];

        // Zeichne Ecken zur Visualisierung
        cv::drawChessboardCorners(image, board_sz, corners, found);

        // Berechne Homographie
        cv::Mat H = cv::getPerspectiveTransform(objPts, imgPts);

        // Benutzer kann die Z-Höhe anpassen
        float Z = 25.0;
        cv::Mat birds_image;

        while (true) {
            // Passe die Z-Höhe in der Homographiematrix an
            H.at<double>(2, 2) = Z;

            // Erzeuge Vogelperspektive
            cv::warpPerspective(image, birds_image, H, image.size(),
                                cv::INTER_LINEAR | cv::WARP_INVERSE_MAP | cv::WARP_FILL_OUTLIERS);

            // Zeige Bilder an
            cv::imshow("Chessboard", image);
            cv::imshow("Birds_Eye", birds_image);

            // Tastensteuerung
            char key = static_cast<char>(cv::waitKey(0));
            if (key == 27) // ESC
                break;
            if (key == 'u')
                Z += 0.5;
            if (key == 'd')
                Z -= 0.5;
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "birdseye_node");
    ros::NodeHandle nh;

    // Abonniere die Topic mit dem rektifizierten Bild
    ros::Subscriber sub = nh.subscribe("robotik_projekt/images/rectified_image", 1, imageCallback);

    ros::spin();
    return 0;
}

