#include <ros/ros.h>
#include "image_transport/image_transport.h"
#include <sensor_msgs/Image.h>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

// Globale Variable zur Speicherung der Homography-Matrix
cv::Mat homographyMatrix;
bool homography_computed = false;

image_transport::Publisher pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) 
{
    try 
    {
        // Konvertiere ROS-Bildnachricht in OpenCV-Bild
        cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        

        if (!homography_computed) 
        {

             /*
           
            //Zum Bestimmen der Koordinaten wurden folgende Funktionen benutzt. Dazu wurde die untere Kante vom Schachbrett an das untere Kamerabild angepasst

            // Zeichne eine vertikale Linie in der Mitte des Bildes zum Ausrichten des Schachbretts
            cv::line(image, cv::Point(image.cols / 2, 0), cv::Point(image.cols / 2, image.rows), cv::Scalar(0, 255, 0), 1);

            cv::imshow("Chessboard", image);
            cv::waitKey(1);

            // Schachbrettparameter
            const int board_w = 7; // Anzahl der Ecken pro Zeile
            const int board_h = 6; // Anzahl der Ecken pro Spalte
            const cv::Size board_sz(board_w, board_h);
            const int board_n = board_w * board_h;

            // Suche nach Schachbrettecken
            std::vector<cv::Point2f> corners;
            cv::Mat gray_image;
            cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
		    cv::equalizeHist(gray_image, gray_image); // Kontrast verbessern

		    

            bool found = cv::findChessboardCorners(gray_image, board_sz, corners);

            if (!found) {
                ROS_WARN("Couldn't acquire chessboard, only found %lu of %d corners.", corners.size(), board_n);
                return;
            }

            // Verfeinerung der Ecken auf Subpixel-Genauigkeit
            // cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
            cv::cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1));

            // Zeichne die gefundenen Ecken in das Originalbild ein
            cv::drawChessboardCorners(image, board_sz, corners, found);

            // Definiere Bild- und Objektpunkte
            std::vector<cv::Point2f> imgPts(4);
            std::vector<cv::Point2f> objPts(4);
           
           
            objPts[0] = cv::Point2f((image.cols / 2) + 70, (image.rows / 2) + 330);
            objPts[1] = cv::Point2f((image.cols / 2) - 70, (image.rows / 2) + 330);
            objPts[2] = cv::Point2f((image.cols / 2) + 70, (image.rows / 2) + 210);
            objPts[3] = cv::Point2f((image.cols / 2) - 70, (image.rows / 2) + 210);
            
            imgPts[0] = corners[0];
            imgPts[1] = corners[board_w - 1];
            imgPts[2] = corners[(board_h - 1) * board_w];
            imgPts[3] = corners[(board_h - 1) * board_w + board_w - 1];
           

            // Ausgabe der Punkte
            ROS_INFO("objPts[0] = (%f, %f)", objPts[0].x, objPts[0].y);
            ROS_INFO("objPts[1] = (%f, %f)", objPts[1].x, objPts[1].y);
            ROS_INFO("objPts[2] = (%f, %f)", objPts[2].x, objPts[2].y);
            ROS_INFO("objPts[3] = (%f, %f)", objPts[3].x, objPts[3].y);
            ROS_INFO("imgPts[0] = (%f, %f)", imgPts[0].x, imgPts[0].y);
            ROS_INFO("imgPts[1] = (%f, %f)", imgPts[1].x, imgPts[1].y);
            ROS_INFO("imgPts[2] = (%f, %f)", imgPts[2].x, imgPts[2].y);
            ROS_INFO("imgPts[3] = (%f, %f)", imgPts[3].x, imgPts[3].y);
           
            // Zeichne den Punkte in das image ein
             cv::circle(image, objPts[0], 20, cv::Scalar(0, 0, 255), -1);//rot
             cv::circle(image, objPts[1], 20, cv::Scalar(0, 255, 0), -1);//grün
             cv::circle(image, objPts[2], 20, cv::Scalar(255, 0, 0), -1);//blau
             cv::circle(image, objPts[3], 20, cv::Scalar(255, 255, 0), -1);//türkis
             cv::circle(image, imgPts[0], 20, cv::Scalar(0, 0, 255), -1);//rot
             cv::circle(image, imgPts[1], 20, cv::Scalar(0, 255, 0), -1);//grün
             cv::circle(image, imgPts[2], 20, cv::Scalar(255, 0, 0), -1);//blau
             cv::circle(image, imgPts[3], 20, cv::Scalar(255, 255, 0), -1);//türkis
           
            */
            
            // Definiere Bild- und Objektpunkte
            std::vector<cv::Point2f> imgPts(4);
            std::vector<cv::Point2f> objPts(4);
            
            // Die Koordinaten wurden aus der Ausgabe ausgelsen
            objPts[0] = cv::Point2f(550, 690);
            objPts[1] = cv::Point2f(410, 690);
            objPts[2] = cv::Point2f(550, 570);
            objPts[3] = cv::Point2f(410, 570);
            
            imgPts[0] = cv::Point2f(750.075317, 673.502136);
            imgPts[1] = cv::Point2f(217.013840, 668.596802);
            imgPts[2] = cv::Point2f(653.700684, 554.289307);
            imgPts[3] = cv::Point2f(304.199554, 551.544983);
            
            

            // Berechne Homographie
            homographyMatrix = cv::getPerspectiveTransform(objPts, imgPts);
            homography_computed = true;

            ROS_INFO("Homography computed successfully.");
        }

        cv::Mat birds_image;

        // Erzeuge Vogelperspektive
        cv::warpPerspective(image, birds_image, homographyMatrix, image.size(), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP);
        

        // Zeige Bilder an
        //cv::imshow("Chessboard", image);
        //cv::imshow("Birds_Eye", birds_image);
        //cv::waitKey(1);

        sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(msg->header, "bgr8", birds_image).toImageMsg();
        pub.publish(output_msg);
	    ROS_INFO("birdseye_image gepublished.");

        
    } 
    catch (cv_bridge::Exception& e) 
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "birdseye_node");
    ros::NodeHandle nh;

    // Abonniere die Topic mit dem rektifizierten Bild
    ros::Subscriber sub = nh.subscribe("robotik_projekt/images/rectified_image", 1, imageCallback);
    image_transport::ImageTransport it(nh);
    pub = it.advertise("robotik_projekt/images/birdseye_image", 1);

    ros::spin();
    return 0;
}
