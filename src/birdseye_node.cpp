#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

// Globale Variable zur Speicherung der Homography-Matrix
cv::Mat H;
bool homography_computed = false;

// Globale Variable für die Z-Höhe
float Z = 11.7;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // Konvertiere ROS-Bildnachricht in OpenCV-Bild
        cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        cv::imshow("Chessboard", image);
        cv::waitKey(1);

        // Schachbrettparameter
        const int board_w = 7; // Anzahl der Ecken pro Zeile
        const int board_h = 6; // Anzahl der Ecken pro Spalte
        const cv::Size board_sz(board_w, board_h);
        const int board_n = board_w * board_h;

        if (!homography_computed) {
            // Suche nach Schachbrettecken
            std::vector<cv::Point2f> corners;
            cv::Mat gray_image;
            cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
		    cv::equalizeHist(gray_image, gray_image); // Kontrast verbessern

		    


            bool found = cv::findChessboardCorners(gray_image, board_sz, corners); //, cv::CALIB_CB_FAST_CHECK);

            if (!found) {
                ROS_WARN("Couldn't acquire chessboard, only found %lu of %d corners.", corners.size(), board_n);
                return;
            }

            // Verfeinerung der Ecken auf Subpixel-Genauigkeit
            // cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
            cv::cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1));

            // Zeichne die gefundenen Ecken in das Originalbild ein
            cv::drawChessboardCorners(image, board_sz, corners, found);

            // Definiere Bild- und Objektpunkte
            std::vector<cv::Point2f> imgPts(4);
            std::vector<cv::Point2f> objPts(4);
           /* objPts[0] = cv::Point2f(0, 0);
            objPts[1] = cv::Point2f((board_w - 1)*10, 0);
            objPts[2] = cv::Point2f(0, (board_h - 1)*10);
            objPts[3] = cv::Point2f((board_w - 1)*10, (board_h - 1)*10);*/
            objPts[0] = cv::Point2f(board_w - 1, board_h - 1);
            objPts[1] = cv::Point2f(0, board_h - 1);
            objPts[2] = cv::Point2f(board_w - 1, 0);
            objPts[3] = cv::Point2f(0, 0);
            imgPts[0] = corners[0];
            imgPts[1] = corners[board_w - 1];
            imgPts[2] = corners[(board_h - 1) * board_w];
            imgPts[3] = corners[(board_h - 1) * board_w + board_w - 1];

            // Ausgabe der objPts 0 bis 3
            // ROS_INFO("objPts[0] = (%f, %f)", imgPts[0].x, imgPts[0].y);
            // ROS_INFO("objPts[1] = (%f, %f)", imgPts[1].x, imgPts[1].y);
            // ROS_INFO("objPts[2] = (%f, %f)", imgPts[2].x, imgPts[2].y);
            // ROS_INFO("objPts[3] = (%f, %f)", imgPts[3].x, imgPts[3].y);

            // Zeichne den ersten Punkt objPts[0] in das gray_image ein
            // cv::circle(image, objPts[0], 20, cv::Scalar(0, 0, 255), -1);//rot
            // cv::circle(image, objPts[1], 20, cv::Scalar(0, 255, 0), -1);//grün
            // cv::circle(image, objPts[2], 20, cv::Scalar(255, 0, 0), -1);//blau
            // cv::circle(image, objPts[3], 20, cv::Scalar(255, 255, 0), -1);//türkis
            // cv::imshow("Grau", gray_image);
            // cv::waitKey(1);

            /*
            std::vector<cv::Point2f> imgPts(4);
            std::vector<cv::Point2f> objPts(4);
            objPts[0] = cv::Point2f(-board_w/2, board_h/2);
            objPts[1] = cv::Point2f(board_w/2, board_h/2);
            objPts[2] = cv::Point2f(-board_w/2, -board_h/2);
            objPts[3] = cv::Point2f(board_w/2, -board_h/2);
            imgPts[0] = corners[0];
            imgPts[1] = corners[board_w - 1];
            imgPts[2] = corners[(board_h - 1) * board_w];
            imgPts[3] = corners[(board_h - 1) * board_w + board_w - 1];
            */

            // Berechne Homographie
            H = cv::getPerspectiveTransform(objPts, imgPts);
            homography_computed = false; //true;

            ROS_INFO("Homography computed successfully.");
        }

        // Benutzer kann die Z-Höhe anpassen
        //float Z = 15.0;
        cv::Mat birds_image;

        // Passe die Z-Höhe in der Homographiematrix an
        cv::Mat H_adjusted = H.clone();
        H_adjusted.at<double>(2, 2) = Z;


        // Erzeuge Vogelperspektive
        cv::warpPerspective(image, birds_image, H_adjusted, image.size(),
                            cv::INTER_LINEAR | cv::WARP_INVERSE_MAP );
                            //| cv::WARP_INVERSE_MAP | cv::WARP_FILL_OUTLIERS
        

        // Zeige Bilder an
        cv::imshow("Chessboard", image);
        cv::imshow("Birds_Eye", birds_image);
        cv::waitKey(1);

        // Tastensteuerung
        char key = static_cast<char>(cv::waitKey(1));
        if (key == 'u') {
            Z += 0.5;
            ROS_INFO("Z= %.1f", Z);
        }
        if (key == 'd') {
            Z -= 0.5;
            ROS_INFO("Z= %.1f", Z);
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
