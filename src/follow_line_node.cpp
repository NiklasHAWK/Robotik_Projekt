#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include <geometry_msgs/Twist.h>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>

// ----- Parameter, die du anpassen musst -----
static const double PIXEL_TO_METER = 0.18 / 360.0; // Meter in der Breite / Pixel in der Breite
static const double LOOKAHEAD_X    = 0.304;     // Lookahead in m nach vorn, ab wo das Sichtfeld beginnt
static const double MIN_OMEGA      = -0.5;     // Begrenzung Winkelgeschwindigkeit
static const double MAX_OMEGA      =  0.5;
static const double LINEAR_SPEED   =  0.08;    // Konstante Vorwärtsgeschwindigkeit
// --------------------------------------------

double omega = 0.0;

ros::Publisher cmd_vel_pub;

// Letzter ermittelter Mittelwert für die Spurmitte (Tiefpass-Filter)
static int last_middle_x = 480;

// Hilfsfunktion zum Finden der linken/rechten Bounding Box
std::pair<cv::Rect, cv::Rect> findLeftAndRightContours(const std::vector<cv::Rect>& boundingBoxes)
{
    cv::Rect leftRect, rightRect;
    int minX = std::numeric_limits<int>::max();
    int maxX = -1;

    for (auto& box : boundingBoxes)
    {
        int centerX = box.x + box.width / 2;
        if (centerX < minX)
        {
            minX = centerX;
            leftRect = box;
        }
        if (centerX > maxX)
        {
            maxX = centerX;
            rightRect = box;
        }
    }
    return std::make_pair(leftRect, rightRect);
}

// -------------------------------------------------------------
// Pure-Pursuit-Funktion: Berechne Winkelgeschwindigkeit
// -------------------------------------------------------------
double purePursuitOmega(double xL, double yL, double v)
{
    // L = Abstand zum Lookahead-Punkt
    double L = std::sqrt(xL*xL + yL*yL);
    // Falls L sehr klein oder Null, vermeiden wir Div/0
    if (L < 1e-6) return 0.0;

    // Krümmung kappa = 2 * yL / L^2
    double kappa = 2.0 * yL / (L * L);

    // Winkelgeschwindigkeit w = v * kappa
    double gain  = 3.0; // Verstärkungsfaktor
    double w = gain * v * kappa;
    return w;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // 1) ROS-Bild in OpenCV-Bild konvertieren
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } 
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat img = cv_ptr->image;

    // 2) In Graustufen konvertieren
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    // 3) Gauß-Filter zum Glätten
    cv::Mat gray_blur;
    cv::GaussianBlur(gray, gray_blur, cv::Size(5, 5), 0);

    // 4) Otsu-Threshold (binäre Umwandlung)
    cv::Mat binary;
    cv::threshold(gray_blur, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // 5) Morphologische Operationen (Öffnen und Schließen)
    cv::Mat morph;
    cv::Mat kernel_morph = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(binary, morph, cv::MORPH_OPEN, kernel_morph);
    cv::morphologyEx(morph, morph, cv::MORPH_CLOSE, kernel_morph);

    // Ecken übermalen
    // Definiere die Punkte für die linke Begrenzungslinie
    cv::Point left_pt1(0, 335);       // Linker Punkt
    cv::Point left_pt2(210, 710);     // Rechter Punkt

    // Definiere die Punkte für die rechte Begrenzungslinie
    cv::Point right_pt1(960, 335);    // Linker Punkt
    cv::Point right_pt2(740, 710);    // Rechter Punkt

    // Erstelle eine Maske in der gleichen Größe wie das Bild
    cv::Mat mask = cv::Mat::zeros(morph.size(), CV_8UC1);

    // Zeichne das Polygon für die linke untere Ecke in Weiß
    std::vector<cv::Point> left_polygon = {
        cv::Point(0, morph.rows), 
        left_pt1, 
        left_pt2, 
        cv::Point(morph.cols / 2, morph.rows) // Annahme: Bildmitte zwischen den beiden Polygons
    };
    cv::fillConvexPoly(mask, left_polygon, cv::Scalar(255));

    // Zeichne das Polygon für die rechte untere Ecke in Weiß
    std::vector<cv::Point> right_polygon = {
        cv::Point(morph.cols, morph.rows),
        right_pt1,
        right_pt2,
        cv::Point(morph.cols / 2, morph.rows)
    };
    cv::fillConvexPoly(mask, right_polygon, cv::Scalar(255));

    // Zeichne den Bereich unterhalb von y = 710 in Weiß (falls gewünscht)
    cv::rectangle(mask, cv::Point(0, 710), cv::Point(morph.cols, morph.rows), cv::Scalar(255), cv::FILLED);

    // Setze die Bereiche in morph auf Weiß basierend auf der Maske
    morph.setTo(0, mask);

    // ----------------------------------------------------------------------
    // 6) Konturensuche (nach dem Übermalen)
    // ----------------------------------------------------------------------
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(morph, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 7) Bounding Boxes ermitteln und filtern (nach Fläche)
    std::vector<cv::Rect> boundingBoxes;
    double minArea = 1000.0; 
    for (const auto& contour : contours)
    {
        double area = cv::contourArea(contour);
        if (area > minArea)
        {
            cv::Rect box = cv::boundingRect(contour);
            boundingBoxes.push_back(box);
        }
    }

    // 8) Fahrspurmitte bestimmen
    int img_center = img.cols / 2;
    int middle_x = img_center;  // Fallback, falls keine Kontur

    if (boundingBoxes.size() >= 2)
    {
        auto pairLR = findLeftAndRightContours(boundingBoxes);
        cv::Rect leftRect  = pairLR.first;
        cv::Rect rightRect = pairLR.second;

        int leftCenterX  = leftRect.x + leftRect.width / 2;
        int rightCenterX = rightRect.x + rightRect.width / 2;
        middle_x = (leftCenterX + rightCenterX) / 2;
    }
    else if (boundingBoxes.size() == 1)
    {
        // Nur eine Kontur => verwende deren Mittelpunkt
        cv::Rect singleBox = boundingBoxes[0];
        middle_x = singleBox.x + singleBox.width / 2;
    }

    // 9) Tiefpass-Filter für sanfte Steuerung
    middle_x = 0.2 * last_middle_x + 0.8 * middle_x;
    last_middle_x = middle_x;

    /*
    // 10) Steuerfehler und Bewegung
    int error = middle_x - img_center;
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.08;               
    cmd_vel.angular.z = -0.0005 * error;   
    cmd_vel.angular.z = std::max(std::min(cmd_vel.angular.z, 0.5), -0.5);
    */

    // ---------------------------------------------------
    // 10) Pure-Pursuit-Berechnung der Winkelgeschwindigkeit
    // ---------------------------------------------------

    // 10.1) Pixel-Offset vom Bildzentrum in x-Richtung
    int offset_px = middle_x - img_center;

    // 10.2) Skaliere Pixel in Meter (y-Achse im Roboter-KS)
    //       Annahme: y nach links/rechts, x vorwärts
    double yL = -offset_px * PIXEL_TO_METER;

    // 10.3) xL = konstanter Lookahead in m (z.B. 0.30 m vor dem Roboter)
    double xL = LOOKAHEAD_X + PIXEL_TO_METER * 360;

    // 10.4) Winkelgeschwindigkeit via Pure-Pursuit-Formel
    omega = purePursuitOmega(xL, yL, LINEAR_SPEED);

    // Begrenzen
    if (omega > MAX_OMEGA)  omega =  MAX_OMEGA;
    if (omega < MIN_OMEGA)  omega =  MIN_OMEGA;

    // 11) Debug-Visualisierung
    // Zeichne Bounding Boxes
    for (auto& box : boundingBoxes)
    {
        cv::rectangle(img, box, cv::Scalar(0, 0, 255), 2);
    }
    // Zeichne linke/rechte Linie (blau) und Mittel-Linie (grün)
    if (boundingBoxes.size() >= 2)
    {
        auto pairLR = findLeftAndRightContours(boundingBoxes);
        int lx = pairLR.first.x + pairLR.first.width  / 2;
        int rx = pairLR.second.x + pairLR.second.width / 2;
        cv::line(img, cv::Point(lx, 0), cv::Point(lx, img.rows), cv::Scalar(255,0,0), 2);
        cv::line(img, cv::Point(rx, 0), cv::Point(rx, img.rows), cv::Scalar(255,0,0), 2);
    }
    cv::line(img, cv::Point(middle_x, 0), cv::Point(middle_x, img.rows), cv::Scalar(0,255,0), 2);
    cv::line(img, cv::Point(0, 360), cv::Point(959, 360), cv::Scalar(0,255,0), 2);

    // Anzeigen
    cv::imshow("Original mit Bounding-Boxen", img);
    cv::imshow("Morph (nach Ecken-Übermalen)", morph);
    cv::waitKey(1);
    ROS_INFO("Omega: %f", omega);
}


// Timer-Callback mit z. B. 30 Hz
void timerCallback(const ros::TimerEvent&)
{
    // Erstelle cmd_vel
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x  = LINEAR_SPEED; 
    cmd_vel.angular.z = omega;  
    ROS_WARN("Omega: %f", omega);     

    // Publish
    cmd_vel_pub.publish(cmd_vel);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "follow_line_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // Kamera-Topic abonnieren
    image_transport::Subscriber sub = it.subscribe("robotik_projekt/images/birdseye_image", 1, imageCallback);

    // Publisher für Bewegungsbefehle
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);


    // Starte einen Timer mit z. B. 30 Hz (kannst du anpassen)
    ros::Timer control_timer = nh.createTimer(
        ros::Duration(1.0/30.0), // => 30 Hz
        timerCallback
    );


    ros::spin();
    return 0;
}
