#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include <geometry_msgs/Twist.h>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Float32.h"
#include <cmath>





// -----------------------------------------
//   GLOBALE PARAMETER
// -----------------------------------------
static const double PIXEL_TO_METER = 0.18 / 360.0; // Meter in der Breite / Pixel in der Breite
static const double LOOKAHEAD_X    = 0.304;     // Lookahead in m nach vorn, ab wo das Sichtfeld beginnt
static const double MIN_OMEGA      = -0.5;     // Begrenzung Winkelgeschwindigkeit
static const double MAX_OMEGA      =  0.5;
static const double LINEAR_SPEED   =  0.08;    // Konstante Vorwärtsgeschwindigkeit

// 30 cm = 0.3 m
static const double OBSTACLE_DISTANCE_THRESHOLD = 0.30;

// 5 Sekunden Wartezeit
static const double WAIT_DURATION = 5.0;

// Zeit für 90°-Drehung (z. B. 1.57 s bei angular.z = -1.0)
static const double TURN_90_DURATION = 1.57; 

bool follow_state = false;



// -----------------------------------------
//   ZUSTANDS-MASCHINE
// -----------------------------------------
enum RobotState {
    SEARCH,
    FOLLOW_LINE,
    WAIT_CHECK,
    TURN_90_RIGHT,
    AVOID_OBSTACLE,
};

static RobotState current_state = FOLLOW_LINE;

// Timestamp, wenn WAIT_CHECK startet
static ros::Time wait_start_time;

// Timestamp, wenn TURN_90_RIGHT startet
static ros::Time turn90_start_time;

// Timestamp, wenn FOLLOW_LINE startet
static ros::Time search_start_time;


// -----------------------------------------
//   GLOBALE STEUER-VARIABLEN
// -----------------------------------------
ros::Publisher cmd_vel_pub;
double omega = 0.0;             // Winkelgeschw. aus Pure Pursuit
static int last_middle_x = 480; // Letzter ermittelter Mittelwert für die Spurmitte (Tiefpass-Filter)
bool line_visible = false;      // Linie erkannt?

// Empfange Distanz (Front + 360°) vom LiDAR-Node
float distance_front = std::numeric_limits<float>::infinity();
float distance_360   = std::numeric_limits<float>::infinity();





// -----------------------------------------
//   HILFSFUNKTIONEN
// -----------------------------------------
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





// -----------------------------------------
//   CALLBACKS
// -----------------------------------------
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
    cv::Mat morphOtsu;
    cv::Mat kernel_morph = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(binary, morphOtsu, cv::MORPH_OPEN, kernel_morph);
    cv::morphologyEx(morphOtsu, morphOtsu, cv::MORPH_CLOSE, kernel_morph);




    // =============== NEU: HSV-Teil für Gelb-Erkennung ===============
    // a) In HSV konvertieren
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    // b) Schwellenwerte für Gelb definieren (mögliche Feinjustierung nötig)
    cv::Scalar lowerY(20, 0, 0);   // untere Grenze Gelb (H=20°, S=80, V=80)
    cv::Scalar upperY(60, 255, 255); // obere Grenze Gelb (H=30°, S=255, V=255)
    cv::Mat maskYellow;
    cv::inRange(hsv, lowerY, upperY, maskYellow);

    // c) Morph-Operationen auf maskYellow
    cv::Mat morphHSV;
    cv::Mat kernel_hsv = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
    cv::morphologyEx(maskYellow, morphHSV, cv::MORPH_OPEN, kernel_hsv);
    cv::morphologyEx(morphHSV, morphHSV, cv::MORPH_CLOSE, kernel_hsv);

    // d) Ecken übermalen auch im HSV-Bild (damit identischer Bildbereich ignoriert wird)
    // Definiere die Punkte für die linke Begrenzungslinie
    cv::Point left_pt1(0, 335);       // Linker Punkt
    cv::Point left_pt2(210, 710);     // Rechter Punkt

    // Definiere die Punkte für die rechte Begrenzungslinie
    cv::Point right_pt1(960, 335);    // Linker Punkt
    cv::Point right_pt2(740, 710);    // Rechter Punkt

    cv::Mat hsvMask = cv::Mat::zeros(morphHSV.size(), CV_8UC1);

    // Zeichne das Polygon für die linke untere Ecke in Weiß
    std::vector<cv::Point> left_polygon = {
        cv::Point(0, morphHSV.rows), 
        left_pt1, 
        left_pt2, 
        cv::Point(morphHSV.cols / 2, morphHSV.rows)
    };
    std::vector<cv::Point> right_polygon = {
        cv::Point(morphHSV.cols, morphHSV.rows),
        right_pt1,
        right_pt2,
        cv::Point(morphHSV.cols / 2, morphHSV.rows)
    };
    // Untere Bereiche ausmaskieren
    cv::fillConvexPoly(hsvMask, left_polygon, cv::Scalar(255));
    cv::fillConvexPoly(hsvMask, right_polygon, cv::Scalar(255));
    cv::rectangle(hsvMask, cv::Point(0, 710), cv::Point(morphHSV.cols, morphHSV.rows), cv::Scalar(255), cv::FILLED);

    // Setze diese Bereiche auf 0 in morphHSV (wie bei Otsu)
    morphHSV.setTo(0, hsvMask);

    // =============== Kombinieren: nur wo Otsu UND Gelb erkannt wird ===============
    cv::Mat masksCombined;
    cv::bitwise_and(morphOtsu, morphHSV, masksCombined);
    // => In "combined" ist nur noch weiß, wo BEIDE Masken übereinstimmen

    // ----------------------------------------------------------------------
    // 6) Konturensuche (nach dem Übermalen)
    // ----------------------------------------------------------------------
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(masksCombined, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 7) Bounding Boxes ermitteln und filtern (nach Fläche)
    std::vector<cv::Rect> boundingBoxes;
    double minArea = 1000.0; 
    for (const auto& contour : contours)
    {
        double area = cv::contourArea(contour);
        if (area > minArea)
        {
            boundingBoxes.push_back(cv::boundingRect(contour));
        }
    }

    // 8) Fahrspurmitte bestimmen
    int img_center = img.cols / 2;
    int middle_x = img_center;  // Fallback, falls keine Kontur

    bool anyContourFound = false;

    if (boundingBoxes.size() >= 2)
    {
        auto pairLR = findLeftAndRightContours(boundingBoxes);
        cv::Rect leftRect  = pairLR.first;
        cv::Rect rightRect = pairLR.second;

        int leftCenterX  = leftRect.x + leftRect.width / 2;
        int rightCenterX = rightRect.x + rightRect.width / 2;
        middle_x = (leftCenterX + rightCenterX) / 2;
        anyContourFound = true;
    }
    else if (boundingBoxes.size() == 1)
    {
        // Nur eine Kontur => verwende deren Mittelpunkt
        cv::Rect singleBox = boundingBoxes[0];
        middle_x = singleBox.x + singleBox.width / 2;
        if(middle_x > 480)
        {
            middle_x = middle_x * 1.3;
        }
        else
        {
            middle_x = middle_x * 0.7;
        }
        anyContourFound = true;
    }
    else
    {
        anyContourFound = false;
    }

    line_visible = anyContourFound;

    //ROS_INFO("line_visible: %s", line_visible ? "true" : "false");

//////////////////////////////////////////////////////////////////////////////////////
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
    if (follow_state)
    {
        // 10.1) Pixel-Offset vom Bildzentrum in x-Richtung
        int offset_px = middle_x - img_center;

        double error = -offset_px * PIXEL_TO_METER;

        // ---- Neuer D-Anteil ----
        // Statische Variablen speichern den vorherigen Fehler und Zeitpunkt
        static double prev_error_follow = 0.0;
        static ros::Time prev_follow_time = ros::Time::now();
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - prev_follow_time).toSec();
        double d_error = 0.0;
        if (dt > 0.0)
        {
            d_error = (error - prev_error_follow) / dt;
        }
        // Wähle einen D-Verstärkungsfaktor (Kd) – diesen Wert musst du experimentell anpassen
        double Kp = 0.8;
        double Kd = 1.2;
        double control = Kp * error + Kd * d_error;

        // Begrenze den Reglerausgang
        if (control > MAX_OMEGA)
            control = MAX_OMEGA;
        if (control < MIN_OMEGA)
            control = MIN_OMEGA;
        omega = control;

        // Aktualisiere die statischen Variablen für den nächsten Aufruf
        prev_error_follow = error;
        prev_follow_time = current_time;
    }

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
    
    // Anzeigen
    cv::imshow("Original mit Bounding-Boxen", img);
    // 1) morphOtsu, 2) morphHSV, 3) combined
    //cv::imshow("OTSU morph", morphOtsu);
    //cv::imshow("HSV morph", morphHSV);
    cv::imshow("Combined OTSU+HSV", masksCombined);
    cv::waitKey(1);
    ROS_INFO("Omega: %f", omega);
}

void distanceFrontCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // Wenn msg->data = -1 => kein Hindernis
    if (msg->data < 0.0) 
    {
        distance_front = std::numeric_limits<float>::infinity();
    } else 
    {
        distance_front = msg->data;
    }
}

void distance360Callback(const std_msgs::Float32::ConstPtr& msg)
{
    if (msg->data < 0.0) 
    {
        distance_360 = std::numeric_limits<float>::infinity();
    } else 
    {
        distance_360 = msg->data;
    }
}





// -----------------------------------------
//   STATE MACHINE (Timer-Callback mit z. B. 30 Hz)
// -----------------------------------------
void timerCallback(const ros::TimerEvent&)
{
    // Erstelle cmd_vel
    geometry_msgs::Twist cmd_vel;
    double now = ros::Time::now().toSec();

    switch (current_state)
    {
        case SEARCH:
        {
            // Statische Variablen, um den akkumulierten Drehwinkel und den letzten Zeitstempel zu speichern
            static double accumulated_angle = 0.0;
            static ros::Time last_search_time = ros::Time::now();
            ros::Time current_time = ros::Time::now();
            double dt = (current_time - last_search_time).toSec();
            last_search_time = current_time;
            
            // Falls während der Suche eine Linie erkannt wird, sofort in FOLLOW_LINE wechseln.
            if (line_visible)
            {
                ROS_INFO("Linie gefunden -> FOLLOW_LINE");
                current_state = FOLLOW_LINE;
                // Reset des akkumulierten Winkels für zukünftige Suchen
                accumulated_angle = 0.0;
                break;
            }
            
            // Falls noch nicht 360° gedreht wurde, rotiere weiter
            if (accumulated_angle < 2 * M_PI)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.3;  // langsame konstante Drehgeschwindigkeit
                // Akkumulieren des gedrehten Winkels (Betrag!)
                accumulated_angle += fabs(cmd_vel.angular.z * dt);
                ROS_INFO("SEARCH: Drehwinkel = %f rad", accumulated_angle);
            }
            else
            {
                // Hat er 360° erreicht, dann stoppe und warte.
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                ROS_INFO("360° gedreht, warte auf Linie...");
                // Der Zustand bleibt SEARCH, sodass wenn sich später line_visible ändert, 
                // der Übergang zu FOLLOW_LINE erfolgt.
            }
            break;
        }

        case FOLLOW_LINE:
        {
            follow_state = true;
            // Falls Hindernis im Front-Bereich < 30 cm => WAIT_CHECK
            if (distance_front < OBSTACLE_DISTANCE_THRESHOLD)
            {
                ROS_INFO("Hindernis <30cm vorne -> WAIT_CHECK");
                current_state = WAIT_CHECK;
                wait_start_time = ros::Time::now();
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
            }
            else
            {
                // Normaler Liniefolge
                if (line_visible)
                {
                    cmd_vel.linear.x  = LINEAR_SPEED;
                    cmd_vel.angular.z = omega;
                }
                else
                {
                    ROS_INFO("Linie verloren -> SEARCH");
                    current_state = SEARCH;
                }
            }
            break;
        }

        case WAIT_CHECK:
        {
            follow_state = false;
            double elapsed = now - wait_start_time.toSec();
            // Wenn Hindernis weg, zurück zu FOLLOW_LINE
            if (distance_front >= OBSTACLE_DISTANCE_THRESHOLD)
            {
                ROS_INFO("Hindernis wieder weg -> zurück zu FOLLOW_LINE");
                current_state = FOLLOW_LINE;
            }
            else
            {
                // Hindernis noch da => nach 5 s in TURN_90_RIGHT
                if (elapsed >= WAIT_DURATION)
                {
                    ROS_INFO("5s abgelaufen -> TURN_90_RIGHT");
                    current_state = TURN_90_RIGHT;
                    turn90_start_time = ros::Time::now();
                }
            }
            // Stehen bleiben
            cmd_vel.linear.x  = 0.0;
            cmd_vel.angular.z = 0.0;
            break;
        }

        case TURN_90_RIGHT:
        {
            follow_state = false;
            // Drehe dich für TURN_90_DURATION um ~90° nach rechts
            double turn_elapsed = now - turn90_start_time.toSec();

            if (turn_elapsed < TURN_90_DURATION)
            {
                // Konstante Rechtsdrehung
                cmd_vel.linear.x  = 0.0;
                cmd_vel.angular.z = -1.0; 
            }
            else
            {
                // Fertig => wechsle in AVOID_OBSTACLE
                ROS_INFO("90° Drehung beendet -> AVOID_OBSTACLE");
                current_state = AVOID_OBSTACLE;
            }
            break;
        }

        case AVOID_OBSTACLE:
        {
            // Idee: 
            //  - Prüfe distance_360, damit wir ~30cm halten.
            //  - Solange line_visible == false, manövrieren wir.
            //  - Wenn line_visible == true => zurück zu FOLLOW_LINE (mit leichter Rechtsdrehung).

            follow_state = false;

            if (line_visible && distance_360 > OBSTACLE_DISTANCE_THRESHOLD)
            {
                // Linie gefunden => z. B. kurze Rechtsdrehung oder direkt rein
                ROS_INFO("Linie wieder da -> FOLLOW_LINE");
                current_state = FOLLOW_LINE;
            }
            else
            {
                // Berechne den Fehler (Abweichung vom gewünschten Abstand)
                double error = distance_360 - (OBSTACLE_DISTANCE_THRESHOLD - 0.1);

                // P-Anteil: Hier wurde zuvor 2 * error verwendet.
                // Füge jetzt einen D-Anteil hinzu:
                static double prev_error = 0.0;           // Vorheriger Fehler (statisch, damit er zwischen den Aufrufen erhalten bleibt)
                static ros::Time prev_time = ros::Time::now();  // Vorheriger Zeitpunkt

                ros::Time current_time = ros::Time::now();
                double dt = (current_time - prev_time).toSec();
                double d_error = 0.0;
                if (dt > 0.0)
                {
                    d_error = (error - prev_error) / dt;
                }

                // Regler-Gewichte (Kp und Kd) können je nach gewünschtem Verhalten angepasst werden
                double Kp = 0.8;  // Proportionalanteil
                double Kd = 1.2;  // Differentialanteil

                double control = Kp * error + Kd * d_error;

                // Setze den Befehl: Wir fahren konstant vorwärts (0.05 m/s) und regeln die Winkelgeschwindigkeit
                cmd_vel.linear.x  = 0.05;
                cmd_vel.angular.z = control;

                // Aktualisiere die statischen Variablen für die nächste Iteration
                prev_error = error;
                prev_time  = current_time;
            }
            break;
        }
    }

    // Befehl publizieren
    cmd_vel_pub.publish(cmd_vel);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "follow_line_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // Subscriber für Kamera/Bird's-Eye
    image_transport::Subscriber img_sub = it.subscribe("robotik_projekt/images/birdseye_image", 1, imageCallback);

    // LiDAR-Abstände (Front / 360°)
    ros::Subscriber front_sub = nh.subscribe("robotik_projekt/obstacle/distance_front", 1, distanceFrontCallback);
    ros::Subscriber surround_sub = nh.subscribe("robotik_projekt/obstacle/distance_360", 1, distance360Callback);

    // Publisher für /cmd_vel
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // Timer: State Machine (z. B. 10-30 Hz)
    ros::Timer control_timer = nh.createTimer(ros::Duration(1.0/30.0), timerCallback); // 30 Hz

    ros::spin();
    return 0;
}