#include <ros/ros.h> 							// ROS Hauptbibliothek für Node-Management
#include <opencv2/opencv.hpp> 					// OpenCV-Bibliothek für Bildverarbeitung
#include <opencv2/imgproc/imgproc.hpp> 			// OpenCV-Bibliothek für Bildverarbeitung (Filter, Morphologien)
#include <cv_bridge/cv_bridge.h> 				// cv_bridge für die Konvertierung zwischen ROS und OpenCV-Bildern
#include <image_transport/image_transport.h> 	// Bibliothek für den Transport von Bildern in ROS
#include <sensor_msgs/Image.h> 					// Nachrichtentyp für Bilddaten
#include <geometry_msgs/Twist.h> 				// Nachrichtentyp für Bewegungsbefehle (linear, angular)
#include <nav_msgs/Odometry.h> 					// Nachrichtentyp für Odometrie-Daten
#include <tf/tf.h> 								// Transformationen für Quaternionen und Euler-Winkel
#include <std_msgs/Float32.h> 					// Nachrichtentyp für Float-Werte
#include <math.h> 								// Mathematische Funktionen

#include <opencv2/highgui/highgui.hpp>

// -----------------------------------------
//   GLOBALE KONSTANTEN
// -----------------------------------------
static const double MIN_OMEGA      = -0.5;  // Begrenzung Winkelgeschwindigkeit
static const double MAX_OMEGA      =  0.5;  // Begrenzung Winkelgeschwindigkeit
static const double LINEAR_SPEED   =  0.08; // Konstante Vorwärtsgeschwindigkeit

// Schwellenwerte für Hinderniserkennung (in Metern)
static const double OBSTACLE_DISTANCE_THRESHOLD_FRONT = 0.4; // Hindernis direkt vor dem Roboter
static const double OBSTACLE_DISTANCE_THRESHOLD_AVOIDANCE = 0.2; // Mindestabstand beim Umfahren

static const double WAIT_DURATION = 3.0;    // Wartezeit in Sekunden

// -----------------------------------------
//   ZUSTANDS-MASCHINE
// -----------------------------------------
enum robot_state {
    SEARCH,         // Nach einer Linie suchen
    FOLLOW_LINE,    // Linie folgen
    WAIT_CHECK,     // Warten bei Hindernis
    TURN_90,        // Drehung um 90° zum Umfahren des Hindernisses
    AVOID_OBSTACLE, // Hindernis umfahren
};

static robot_state current_state = SEARCH; // Startzustand: Nach der Linie suchen

// Timestamps für die einzelnen Zustände
static ros::Time wait_start_time;   // Zeitpunkt, wenn WAIT_CHECK startet
static ros::Time turn90_start_time; // Zeitpunkt, wenn TURN_90 startet
static ros::Time search_start_time; // Zeitpunkt, wenn SEARCH startet


// -----------------------------------------
//   GLOBALE PARAMETER
// -----------------------------------------
bool follow_state = false; // Gibt an, ob der Roboter aktiv der Linie folgt

ros::Publisher cmd_vel_pub; // Publisher für Bewegungsbefehle ("/cmd_vel")

// Winkelgeschwindigkeit und letzte Mittelwertspeicherung
double omega = 0.0;             // Winkelgeschwindigkeit
static int last_middle_x = 480; // Tiefpass-gefilterte X-Position der Spurmitte

bool line_visible = false;      // Gibt an, ob eine Linie erkannt wurde

// Lidar-Daten: Abstand zu Hindernissen
float distance_front = std::numeric_limits<float>::infinity(); // Abstand zum nächsten Hindernis vorne
float distance_360   = std::numeric_limits<float>::infinity(); // Minimaler Abstand im gesamten Scanbereich


// Globale Variablen für die Odometry-basierte Drehung
double accumulated_yaw = 0.0; // Aufsummierte Drehung

// Hindernisvermeidung: Standardmäßig nach rechts ausweichen
float turn_direction = -1.0;
float obsticle_direction = -1.0;


// -----------------------------------------
//   HILFSFUNKTIONEN
// -----------------------------------------
// Funktion zur Identifikation der linken und rechten Begrenzungslinien
std::pair<cv::Rect, cv::Rect> findLeftAndRightContours(const std::vector<cv::Rect>& bounding_boxes)
{
    cv::Rect left_rectangle, right_rectangle;
    int min_x = std::numeric_limits<int>::max();
    int max_x = -1;

    for (auto& box : bounding_boxes)
    {
        int center_x = box.x + box.width / 2;
        if (center_x < min_x)
        {
            min_x = center_x;
            left_rectangle = box;
        }
        if (center_x > max_x)
        {
            max_x = center_x;
            right_rectangle = box;
        }
    }
    return std::make_pair(left_rectangle, right_rectangle);
}

// Berechnung der Winkelgeschwindigkeit basierend auf dem Fehler (PD-Regler)
double computeAngularVelocity(double Kp, double Kd, double error, double &prev_error, ros::Time current_time, ros::Time &prev_time)
{
    double dt = (current_time - prev_time).toSec(); // Zeitdifferenz berechnen
    double d_error = 0.0;
    if (dt > 0.0)
    {
        d_error = (error - prev_error) / dt; // Berechnung der Änderungsrate des Fehlers
    }
    double control = Kp * error + Kd * d_error;
    
    // Begrenzung der Winkelgeschwindigkeit
    if (control > MAX_OMEGA)
        control = MAX_OMEGA;
    if (control < MIN_OMEGA)
        control = MIN_OMEGA;
    
    return control;
}


// -----------------------------------------
//   CALLBACKS
// -----------------------------------------
// Callback zur Verarbeitung der Bilddaten
void imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
    // Konvertieren der ROS-Bildnachricht in OpenCV-Bild
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    } 
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat birds_eye_image = cv_ptr->image;

    // Konvertieren des Bildes in den HSV-Farbraum
    cv::Mat hsv;
    cv::cvtColor(birds_eye_image, hsv, cv::COLOR_BGR2HSV);

    // Schwellenwerte für Gelb definieren
    cv::Scalar lower_value(20, 100, 50);
    cv::Scalar upper_value(60, 255, 200);
    cv::Mat mask_yellow;
    cv::inRange(hsv, lower_value, upper_value, mask_yellow);

    // Morph-Operationen auf mask_yellow, um Rauschen zu entfernen
    cv::Mat morph_hsv;
    cv::Mat kernel_hsv = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
    cv::morphologyEx(mask_yellow, morph_hsv, cv::MORPH_OPEN, kernel_hsv);
    cv::morphologyEx(morph_hsv, morph_hsv, cv::MORPH_CLOSE, kernel_hsv);

    // Definiere die Punkte für die linke Begrenzungslinie
    cv::Point left_pt1(0, 335);       // Linker Punkt
    cv::Point left_pt2(210, 710);     // Rechter Punkt

    // Definiere die Punkte für die rechte Begrenzungslinie
    cv::Point right_pt1(960, 335);    // Linker Punkt
    cv::Point right_pt2(740, 710);    // Rechter Punkt

    cv::Mat mask_hsv = cv::Mat::zeros(morph_hsv.size(), CV_8UC1);

    // Zeichnen der Polygone in weiß
    std::vector<cv::Point> left_polygon = {cv::Point(0, morph_hsv.rows), left_pt1, left_pt2, cv::Point(morph_hsv.cols / 2, morph_hsv.rows)};
    std::vector<cv::Point> right_polygon = {cv::Point(morph_hsv.cols, morph_hsv.rows), right_pt1, right_pt2, cv::Point(morph_hsv.cols / 2, morph_hsv.rows)};
    
    // Untere Bereiche ausmaskieren
    cv::fillConvexPoly(mask_hsv, left_polygon, cv::Scalar(255));
    cv::fillConvexPoly(mask_hsv, right_polygon, cv::Scalar(255));
    cv::rectangle(mask_hsv, cv::Point(0, 710), cv::Point(morph_hsv.cols, morph_hsv.rows), cv::Scalar(255), cv::FILLED);

    // Setzt diese Bereiche auf 0 in morph_hsv
    morph_hsv.setTo(0, mask_hsv);

    // Bestimmung der Konturen der gelben Spurmarkierung
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(morph_hsv, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Bounding Boxes ermitteln und filtern nach Fläche
    std::vector<cv::Rect> bounding_boxes;
    double min_area = 1000.0; 
    for (const auto& contour : contours)
    {
        double area = cv::contourArea(contour);
        if (area > min_area)
        {
            bounding_boxes.push_back(cv::boundingRect(contour));
        }
    }

    // Fahrspurmitte bestimmen
    int image_center = birds_eye_image.cols / 2;
    int middle_x = image_center;  // Fallback, falls keine Kontur

    bool any_contour_found = false;

    if (bounding_boxes.size() >= 2) // Falls zwei Linien gefunden wurden
    {
        auto pair_left_right_rectangle = findLeftAndRightContours(bounding_boxes);
        cv::Rect left_rectangle  = pair_left_right_rectangle.first;
        cv::Rect right_rectangle = pair_left_right_rectangle.second;

        int left_rectangle_center_x  = left_rectangle.x + left_rectangle.width / 2;
        int right_rectangle_center_x = right_rectangle.x + right_rectangle.width / 2;
        middle_x = (left_rectangle_center_x + right_rectangle_center_x) / 2;
        any_contour_found = true;
    }
    else if (bounding_boxes.size() == 1) // Falls nur eine Linie sichtbar ist
    {
        cv::Rect single_rectangle = bounding_boxes[0];
        middle_x = single_rectangle.x + single_rectangle.width / 2;
        
        if(middle_x > 480)  // Stärker in die Richtung der Kurve lenken
            middle_x = middle_x * 1.5;
        else
            middle_x = middle_x * 0.5;

        any_contour_found = true;
    }
    else
    {
        any_contour_found = false;
    }

    line_visible = any_contour_found;

    // Tiefpass-Filter für sanftere Steuerung
    middle_x = 0.3 * last_middle_x + 0.7 * middle_x;
    last_middle_x = middle_x;

    if (follow_state)
    {
        // Pixel-Offset vom Bildzentrum in x-Richtung
        int offset_px = middle_x - image_center;
        
        // Übergabeparameter für die Drehwinkelberechnung
        double Kp = 0.00045;
        double Kd = 0.0004;
        double error = -offset_px;
        static double prev_error_follow = 0.0;
        ros::Time current_time = ros::Time::now();
        static ros::Time prev_follow_time = ros::Time::now();

        // Drehwinkelberechnung
        omega = computeAngularVelocity(Kp, Kd, error, prev_error_follow, current_time, prev_follow_time);

        // Aktualisieren der statischen Variablen für den nächsten Aufruf
        prev_error_follow = error;
        prev_follow_time = current_time;
    }

    // Visualisierung
    // Zeichne Bounding Boxes
    for (auto& box : bounding_boxes)
    {
        cv::rectangle(birds_eye_image, box, cv::Scalar(0, 0, 255), 2);
    }
    // Zeichnen linker/rechter Linie (blau) und Mittel-Linie (grün)
    if (bounding_boxes.size() >= 2)
    {
        auto pair_left_right_rectangle = findLeftAndRightContours(bounding_boxes);
        int left_x = pair_left_right_rectangle.first.x + pair_left_right_rectangle.first.width  / 2;
        int right_x = pair_left_right_rectangle.second.x + pair_left_right_rectangle.second.width / 2;
        cv::line(birds_eye_image, cv::Point(left_x, 0), cv::Point(left_x, birds_eye_image.rows), cv::Scalar(255,0,0), 2);
        cv::line(birds_eye_image, cv::Point(right_x, 0), cv::Point(right_x, birds_eye_image.rows), cv::Scalar(255,0,0), 2);
    }
    cv::line(birds_eye_image, cv::Point(middle_x, 0), cv::Point(middle_x, birds_eye_image.rows), cv::Scalar(0,255,0), 2);
    
    // Anzeigen
    cv::imshow("Original mit Bounding-Boxen", birds_eye_image);
    cv::imshow("HSV morph", morph_hsv);
    cv::waitKey(1);
}

// Callback zur Verarbeitung der Entfernungsdaten nach vorne
void distanceFrontCallback(const std_msgs::Float32::ConstPtr& distance_msg)
{
    // Wenn distance_msg->data = -1 => kein Hindernis
    if (distance_msg->data < 0.0) 
        distance_front = std::numeric_limits<float>::infinity();
    else 
        distance_front = distance_msg->data;
}

// Callback zur Verarbeitung der Entfernungsdaten um den Roboter
void distance360Callback(const std_msgs::Float32::ConstPtr& distance_msg)
{
    // Wenn distance_msg->data = -1 => kein Hindernis
    if (distance_msg->data < 0.0) 
        distance_360 = std::numeric_limits<float>::infinity();
    else 
        distance_360 = distance_msg->data;
}

// Callback zur Verarbeitung der Drehrichtung
void directionCallback(const std_msgs::Float32::ConstPtr& direction_msg)
{
    turn_direction = direction_msg->data;
}

// Callback zur Verarbeitung der Odometriedaten
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    // Extrahieren der Werte aus der Quaternion
    tf::Quaternion quaternion(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w);
    tf::Matrix3x3 quaternion_matrix(quaternion);
    double roll, pitch, yaw;
    quaternion_matrix.getRPY(roll, pitch, yaw);

    // Statische Variablen zur Akkumulation
    static bool first_odom = true;
    static double prev_yaw = 0.0;
    
    if (first_odom)
    {
        prev_yaw = yaw;
        first_odom = false;
        accumulated_yaw = 0.0;
    }
    else
    {
        // Berechnen der Änderung zwischen dem aktuellen und dem vorherigen Yaw
        double delta_yaw = yaw - prev_yaw;
        // Normalisieren des Winkels, damit er im Bereich [-π, π] liegt
        if (delta_yaw > M_PI)
            delta_yaw -= 2 * M_PI;
        else if (delta_yaw < -M_PI)
            delta_yaw += 2 * M_PI;
        // Akkumulieren des absoluten Drehbetrages
        accumulated_yaw += fabs(delta_yaw);
        prev_yaw = yaw;
    }
}

// -----------------------------------------
//   STATE MACHINE (Timer-Callback mit 30 Hz)
// -----------------------------------------
void timerCallback(const ros::TimerEvent&)
{
    geometry_msgs::Twist cmd_vel; // Bewegungsbefehl für den Roboter
    double now = ros::Time::now().toSec();

    // Zustandsmaschine zur Steuerung des Roboters
    switch (current_state)
    {
        case SEARCH: // Nach einer Linie suchen
        {
            follow_state = false;
            
            // Falls eine Linie erkannt wurde, wechsle zu FOLLOW_LINE
            if (line_visible)
            {
                ROS_INFO("Linie gefunden -> FOLLOW_LINE");
                current_state = FOLLOW_LINE;
                accumulated_yaw = 0.0; // Zurücksetzen der Drehungsakkumulation
                break;
            }
            if (accumulated_yaw < 2 * M_PI) // Noch nicht 360° gedreht: Weiterdrehen
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.3; // Langsame Drehung
            }
            else // Falls 360° erreicht sind:
            {
                ROS_INFO("360 Grad gedreht, aber keine Linie gefunden. Warte...");
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
            }
            break;
        }

        case FOLLOW_LINE: // Linie folgen
        {
            follow_state = true;
            if (distance_front < OBSTACLE_DISTANCE_THRESHOLD_FRONT) // Falls Hindernis im Front-Bereich  => WAIT_CHECK
            {
                ROS_INFO("Hindernisentfernung vorne < 40cm -> WAIT_CHECK");
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                current_state = WAIT_CHECK;
                wait_start_time = ros::Time::now(); // Starte die Wartezeit
            }
            else // Normaler Liniefolge
            {
                if (line_visible)
                {
                    cmd_vel.linear.x  = LINEAR_SPEED;
                    cmd_vel.angular.z = omega; // PD-kontrollierte Drehung
                }
                else
                {
                    ROS_INFO("Linie verloren -> SEARCH");
                    current_state = SEARCH;
                    accumulated_yaw = 0.0;
                }
            }
            break;
        }

        case WAIT_CHECK: // Warten bei Hindernis
        {
            follow_state = false;
            double elapsed_time = now - wait_start_time.toSec(); // Verstrichene Wartezeit berechnen
            
            if (distance_front >= OBSTACLE_DISTANCE_THRESHOLD_FRONT) // Wenn Hindernis weg, zurück zu FOLLOW_LINE
            {
                ROS_INFO("Hindernis wieder weg -> zurück zu FOLLOW_LINE");
                current_state = FOLLOW_LINE;
            }
            else
            {
                if (elapsed_time >= WAIT_DURATION) // Hindernis nach 3s noch da => TURN_90
                {
                    ROS_INFO("3s abgelaufen -> TURN_90");
                    current_state = TURN_90;
                    accumulated_yaw = 0.0;
                    obsticle_direction = turn_direction;
                    if (obsticle_direction == 1.0)
                        ROS_INFO("Empfangene Richtungsinfo: links -> Objekt rechts");
                    else
                        ROS_INFO("Empfangene Richtungsinfo: rechts -> Objekt links");
                }
            }
            // Stehen bleiben
            cmd_vel.linear.x  = 0.0;
            cmd_vel.angular.z = 0.0;
            break;
        }

        case TURN_90: // Drehung um 90° zum Umfahren des Hindernisses
        {
            follow_state = false;
            
            if (fabs(accumulated_yaw) < M_PI / 2) // Noch nicht 90° gedreht: Weiterdrehen
            {
                // Konstante Drehung
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.6 * obsticle_direction; // Links oder Rechts
            }
            else // Wenn 90° erreicht sind:
            {
                // Fertig => wechsle in AVOID_OBSTACLE
                ROS_INFO("90° Drehung beendet -> AVOID_OBSTACLE");
                current_state = AVOID_OBSTACLE;
                accumulated_yaw = 0.0;
            }
            break;
        }

        case AVOID_OBSTACLE: // Hindernis umfahren
        {
            follow_state = false;

            // Falls die Linie wieder sichtbar ist und der Abstand groß genug ist, wechsle zu FOLLOW_LINE
            if (line_visible && distance_360 > OBSTACLE_DISTANCE_THRESHOLD_AVOIDANCE)
            {
                ROS_INFO("Linie wieder da -> FOLLOW_LINE");
                current_state = FOLLOW_LINE;
            }
            else
            {
                // Solange der Abstand vor dem Roboter groß genug ist, fahre langsam weiter
                if (distance_front > OBSTACLE_DISTANCE_THRESHOLD_AVOIDANCE)
                {
                    // Übergabeparameter für die Drehwinkelberechnung
                    double Kp = 1.2;
                    double Kd = 1.2;
                    double error = distance_360 - (OBSTACLE_DISTANCE_THRESHOLD_AVOIDANCE);
                    static double prev_error_avoid = 0.0;
                    ros::Time current_time = ros::Time::now();
                    static ros::Time prev_avoid_time = ros::Time::now();
                    
                    cmd_vel.linear.x  = 0.05; // langsame Vorwärtsfahrt
                    cmd_vel.angular.z = -obsticle_direction * computeAngularVelocity(Kp, Kd, error, prev_error_avoid, current_time, prev_avoid_time);
            
                    // Aktualisieren der statischen Variablen für den nächsten Aufruf
                    prev_error_avoid = error;
                    prev_avoid_time  = current_time;
                }
                else
                {
                    // Hindernis zu nah => Warte
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    ROS_INFO("Hindernis beim Umfahren erkannt.");
                }
            }
            break;
        }
    }

    //cmd_vel_pub.publish(cmd_vel); // Befehl publizieren
}

int main(int argc, char** argv)
{
    // Initialisierung des ROS-Nodes
    ros::init(argc, argv, "follow_line_node");
    ros::NodeHandle nh;

    ROS_INFO("follow_line_node gestartet.");
    ROS_INFO("Linie suchen -> SEARCH");

    // Erstellung eines ImageTransport für den NodeHandle
    image_transport::ImageTransport it(nh);

    // Subscriber für die Odometrie
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCallback);

    // Subscriber für das Bird's Eye View Bild
    image_transport::Subscriber img_sub = it.subscribe("robotik_projekt/images/birds_eye_image", 1, imageCallback);

    // Subscriber für LiDAR-Abstände und Drehrichtung
    ros::Subscriber distance_front_sub = nh.subscribe("robotik_projekt/obstacle/distance_front", 1, distanceFrontCallback);
    ros::Subscriber distance_360_sub = nh.subscribe("robotik_projekt/obstacle/distance_360", 1, distance360Callback);
    ros::Subscriber turn_direction_sub = nh.subscribe("robotik_projekt/obstacle/turn_direction", 1, directionCallback);

    // Publisher für Fahrtbefehl
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // Timer: State Machine
    ros::Timer control_timer = nh.createTimer(ros::Duration(1.0/30.0), timerCallback); // 30 Hz

    // ROS-Loop zur Verarbeitung eingehender Nachrichten
    ros::spin();

    return 0;
}