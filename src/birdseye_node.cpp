#include <ros/ros.h> 							// ROS Hauptbibliothek für Node-Management
#include <image_transport/image_transport.h> 	// Bibliothek für den Transport von Bildern in ROS
#include <sensor_msgs/Image.h> 					// Nachrichtentyp für Bilder
#include "opencv2/opencv.hpp" 					// OpenCV-Bibliothek für Bildverarbeitung
#include "cv_bridge/cv_bridge.h" 				// cv_bridge für die Konvertierung zwischen ROS und OpenCV-Bildern

// Globale Variable zur Speicherung der Homographie-Matrix
cv::Mat homography_matrix;

// Flag zur Überprüfung, ob die Homographie bereits berechnet wurde
bool homography_computed = false;

// Publisher für das entzerrte Bild in der Vogelperspektive
image_transport::Publisher birds_eye_image_pub;

// Callback-Funktion zur Verarbeitung eingehender Bilder
void birdsEyeImageCallback(const sensor_msgs::ImageConstPtr& msg) 
{
    try 
    {
        // Konvertieren einer ROS-Nachricht in ein OpenCV-Bild
        cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        
        // Berechne die Homographie-Matrix nur einmal
        if (!homography_computed) 
        {

            /*
           
            //Zur Bestimmung der Koordinaten wurden folgende Funktionen benutzt. Dazu wurde die untere Kante vom Schachbrett an das untere Kamerabild angepasst.

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
            cv::cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1));

            // Zeichne die gefundenen Ecken in das Originalbild ein
            cv::drawChessboardCorners(image, board_sz, corners, found);

            // Definiere Bild- und Objektpunkte
            std::vector<cv::Point2f> image_points(4);
            std::vector<cv::Point2f> object_points(4);
           
           
            object_points[0] = cv::Point2f((image.cols / 2) + 70, (image.rows / 2) + 330);
            object_points[1] = cv::Point2f((image.cols / 2) - 70, (image.rows / 2) + 330);
            object_points[2] = cv::Point2f((image.cols / 2) + 70, (image.rows / 2) + 210);
            object_points[3] = cv::Point2f((image.cols / 2) - 70, (image.rows / 2) + 210);
            
            image_points[0] = corners[0];
            image_points[1] = corners[board_w - 1];
            image_points[2] = corners[(board_h - 1) * board_w];
            image_points[3] = corners[(board_h - 1) * board_w + board_w - 1];
           

            // Ausgabe der Punkte
            ROS_INFO("object_points[0] = (%f, %f)", object_points[0].x, object_points[0].y);
            ROS_INFO("object_points[1] = (%f, %f)", object_points[1].x, object_points[1].y);
            ROS_INFO("object_points[2] = (%f, %f)", object_points[2].x, object_points[2].y);
            ROS_INFO("object_points[3] = (%f, %f)", object_points[3].x, object_points[3].y);
            ROS_INFO("image_points[0] = (%f, %f)", image_points[0].x, image_points[0].y);
            ROS_INFO("image_points[1] = (%f, %f)", image_points[1].x, image_points[1].y);
            ROS_INFO("image_points[2] = (%f, %f)", image_points[2].x, image_points[2].y);
            ROS_INFO("image_points[3] = (%f, %f)", image_points[3].x, image_points[3].y);
           
            // Zeichne den Punkte in das image ein
             cv::circle(image, object_points[0], 20, cv::Scalar(0, 0, 255), -1);//rot
             cv::circle(image, object_points[1], 20, cv::Scalar(0, 255, 0), -1);//grün
             cv::circle(image, object_points[2], 20, cv::Scalar(255, 0, 0), -1);//blau
             cv::circle(image, object_points[3], 20, cv::Scalar(255, 255, 0), -1);//türkis
             cv::circle(image, image_points[0], 20, cv::Scalar(0, 0, 255), -1);//rot
             cv::circle(image, image_points[1], 20, cv::Scalar(0, 255, 0), -1);//grün
             cv::circle(image, image_points[2], 20, cv::Scalar(255, 0, 0), -1);//blau
             cv::circle(image, image_points[3], 20, cv::Scalar(255, 255, 0), -1);//türkis
           
            */
            
            // Definieren von Bild- und Objektpunkten für die Transformation
            std::vector<cv::Point2f> image_points(4);
            std::vector<cv::Point2f> object_points(4);

            // Bildpunkte definieren (ursprüngliche Kamerakoordinaten)
            image_points[0] = cv::Point2f(750.075317, 673.502136);
            image_points[1] = cv::Point2f(217.013840, 668.596802);
            image_points[2] = cv::Point2f(653.700684, 554.289307);
            image_points[3] = cv::Point2f(304.199554, 551.544983);

            // Objektpunkte definieren (Zielkoordinaten in der Vogelperspektive)
            object_points[0] = cv::Point2f(480 + 180, 675 - 20);
            object_points[1] = cv::Point2f(480 - 180, 675 - 20);
            object_points[2] = cv::Point2f(480 + 180, 675 - 300 - 20);
            object_points[3] = cv::Point2f(480 - 180, 675 - 300 - 20);

            // Berechnung der Homographie-Matrix für die Transformation
            homography_matrix = cv::getPerspectiveTransform(image_points, object_points);
            homography_computed = true;

            ROS_INFO("Homography computed successfully.");
        }

        cv::Mat birds_eye_image;

        // Anwenden der Perspektivtransformation für die Vogelperspektive
        cv::warpPerspective(image, birds_eye_image, homography_matrix, image.size(), cv::INTER_LINEAR);
        
        // Ausgrauen des nicht erfassten Bereichs
        cv::Vec3b gray_value(105, 105, 105);  
	    cv::Vec3b black_value(0, 0, 0);  
	    for(int y=0; y<birds_eye_image.rows; y++)
	    {	
    	    for(int x=0; x<birds_eye_image.cols; x++)
    	    {
	        	if(birds_eye_image.at<cv::Vec3b>(y,x) == black_value)
                {
	        		birds_eye_image.at<cv::Vec3b>(cv::Point(x,y)) = gray_value;
	         	}
		    }
	    }

        // OpenCV-Bild zurück in eine ROS-Nachricht konvertieren
        sensor_msgs::ImagePtr birds_eye_image_msg = cv_bridge::CvImage(msg->header, "bgr8", birds_eye_image).toImageMsg();
        
        // Veröffentlichen des Bildes mit Vogelperspektive
        birds_eye_image_pub.publish(birds_eye_image_msg);
    } 
    catch (cv_bridge::Exception& e) 
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) 
{
    // Initialisierung des ROS-Nodes
    ros::init(argc, argv, "birdseye_node");
    ros::NodeHandle nh;

    // Erstellung eines ImageTransport für den NodeHandle
    image_transport::ImageTransport it(nh);

    // Abonnieren des Topics mit dem entzerrten Bild
    image_transport::Subscriber sub = it.subscribe("robotik_projekt/images/rectified_image", 1, birdsEyeImageCallback);
    
    // Erstellung eines Publishers für das Bild in der Vogelperspektive
    birds_eye_image_pub = it.advertise("robotik_projekt/images/birds_eye_image", 1);

    // ROS-Loop zur Verarbeitung eingehender Nachrichten
    ros::spin();
    
    return 0;
}