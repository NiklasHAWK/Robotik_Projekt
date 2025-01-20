#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include <vector>
#include <iostream>

int main() {
    // Lade das Bild
    cv::Mat image = cv::imread("/home/niklas/Bilder/mein_bild.png");

    if (image.empty()) {
        std::cerr << "Das Bild konnte nicht geladen werden!" << std::endl;
        return -1;
    }

    // Definiere die Quellpunkte (Ecken des markierten Bereichs)
    std::vector<cv::Point2f> source_points = {
        cv::Point2f(100, 100),   // Punkt 1
        cv::Point2f(400, 100),   // Punkt 2
        cv::Point2f(400, 400),   // Punkt 3
        cv::Point2f(100, 400)    // Punkt 4
    };

    // Definiere die Zielpunkte (Ecken im Bird's Eye View)
    std::vector<cv::Point2f> destination_points = {
        cv::Point2f(0, 0),       // Neuer Punkt 1
        cv::Point2f(500, 0),     // Neuer Punkt 2
        cv::Point2f(500, 500),   // Neuer Punkt 3
        cv::Point2f(0, 500)      // Neuer Punkt 4
    };

    // Berechne die Homographie-Matrix
    cv::Mat homography_matrix = cv::findHomography(source_points, destination_points);

    // Zielbildgröße (Vogelperspektive)
    int width = 500;  // Breite des Zielbildes
    int height = 500; // Höhe des Zielbildes

    // Transformiere das Bild zur Bird's Eye View
    cv::Mat transformed_image;
    cv::warpPerspective(image, transformed_image, homography_matrix, cv::Size(width, height));

    // Zeige das transformierte Bild
    cv::imshow("Bird's Eye View", transformed_image);
    cv::waitKey(0);  // Warten, bis eine Taste gedrückt wird

    return 0;
}
