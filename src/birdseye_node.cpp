#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>

int main(int argc, char* argv[]) {
    

    // INPUT PARAMETERS:
    int board_w = 7;
    int board_h = 6;
    int board_n = board_w * board_h;
    cv::Size board_sz(board_w, board_h);

    // Camera parameters (hardcoded)
    const int image_width = 960;
    const int image_height = 720;
    const std::string camera_name = "camera";

    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        843.074971, 0, 478.388183,
        0, 844.236343, 333.350289,
        0, 0, 1);

    cv::Mat distortion_coefficients = (cv::Mat_<double>(1, 5) <<
        0.09635099999999999, -0.094708, -0.001249, -0.000937, 0);

    cv::Mat rectification_matrix = (cv::Mat_<double>(3, 3) <<
        1, 0, 0,
        0, 1, 0,
        0, 0, 1);

    cv::Mat projection_matrix = (cv::Mat_<double>(3, 4) <<
        863.411682, 0, 477.059562, 0,
        0, 864.280518, 332.073349, 0,
        0, 0, 1, 0);

    // Load the image from the specified path
    std::string image_path = "/home/amrl/catkin_ws/src/robotik_projekt_ws24_nvs_js/Homographie/Schachbrett_auf_Boden.png";
    cv::Mat image = cv::imread(image_path);
    if (image.empty()) {
        std::cerr << "Error: Couldn't load image " << image_path << std::endl;
        return -1;
    }

    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    cv::imshow("gray_image", gray_image);
    cv::waitKey(0);

    // UNDISTORT THE IMAGE
    cv::Mat undistorted_image;
    cv::undistort(image, undistorted_image, camera_matrix, distortion_coefficients);
    cv::imshow("undistorted_image", undistorted_image);
    cv::waitKey(0);

    // DETECT CHESSBOARD CORNERS
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(undistorted_image, board_sz, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);

    if (!found) {
        std::cerr << "Couldn't acquire chessboard on " << image_path
                  << ", only found " << corners.size() << " of " << board_n << " corners." << std::endl;
        return -1;
    }

    // Refine corner positions
    cv::cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1));

    // Define object points
    std::vector<cv::Point2f> objPts = {
        {0, 0}, {static_cast<float>(board_w - 1), 0},
        {0, static_cast<float>(board_h - 1)}, {static_cast<float>(board_w - 1), static_cast<float>(board_h - 1)}
    };

    std::vector<cv::Point2f> imgPts = {
        corners[0],
        corners[board_w - 1],
        corners[(board_h - 1) * board_w],
        corners[(board_h - 1) * board_w + board_w - 1]
    };

    // Draw the points
    cv::circle(image, imgPts[0], 9, cv::Scalar(0, 0, 255), 3); // Red
    cv::circle(image, imgPts[1], 9, cv::Scalar(0, 255, 0), 3); // Green
    cv::circle(image, imgPts[2], 9, cv::Scalar(255, 0, 0), 3); // Blue
    cv::circle(image, imgPts[3], 9, cv::Scalar(0, 255, 255), 3); // Yellow

    cv::drawChessboardCorners(image, board_sz, corners, found);
    cv::imshow("Chessboard", image);

    // Calculate homography
    cv::Mat H = cv::getPerspectiveTransform(objPts, imgPts);

    // LET THE USER ADJUST THE Z HEIGHT OF THE VIEW
    float Z = 25.0f;
    int key = 0;
    cv::Mat birds_image;

    cv::namedWindow("Birds_Eye");
    while (key != 27) { // Escape key to exit
        // Adjust homography
        H.at<double>(2, 2) = Z;

        // Warp perspective for bird's eye view
        cv::warpPerspective(undistorted_image, birds_image, H, undistorted_image.size(),
            cv::INTER_LINEAR | cv::WARP_INVERSE_MAP | cv::WARP_FILL_OUTLIERS);

        cv::imshow("Birds_Eye", birds_image);

        key = cv::waitKey();
        if (key == 'u') Z += 0.5;
        if (key == 'd') Z -= 0.5;
    }

    // Save the homography matrix
    cv::FileStorage fs_H("/home/amrl/catkin_ws/src/robotik_projekt_ws24_nvs_js/Homographie/H.xml", cv::FileStorage::WRITE);
    fs_H << "H" << H;

    return 0;
}

