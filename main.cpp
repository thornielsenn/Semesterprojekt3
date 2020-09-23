#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <exception>
#include <vector>
#include <string>

int main(int argc, char **argv) {

    (void)argc;
    (void)argv;

    std::vector<cv::String> fileNames;
    cv::glob("/home/thor1234/Documents/Semesterprojekt/Calibration/Image*.bmp", fileNames, false); //file path til gemte billede fra kamera
    cv::Size patternSize(10 - 1, 7 - 1); // finder kanter på skakbræt, -1 da vi ikke kan finde kanter på de hvide stykker
    std::vector<std::vector<cv::Point2f>> q(fileNames.size());

    if (fileNames.size() < 28) {
        std::cerr << "You need more pictures" << std::endl;
        exit(-1);
    }

    // Detect feature points
    std::size_t i = 0;
    for (i = 0; i < fileNames.size(); i++) {
        //std::cout << std::string(f) << std::endl;

        // 1. Read in the image an call cv::findChessboardCorners()
        cv::Mat img = cv::imread(fileNames [i]); // bruger 'imread' til at læse billeder der ligger i arrayet
        bool success = cv::findChessboardCorners(img, patternSize, q[i]);
        if (success) {
            cv::Mat grey; //
            cv::cvtColor(img, grey, CV_BGR2GRAY);
            // 2. Use cv::cornerSubPix() to refine the found corner detections
            cv::cornerSubPix(grey, q[i], cv::Size(4,4), cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        }
        // Display
        cv::drawChessboardCorners(img, patternSize, q[i], success);
        cv::imshow("chessboard detection", img);
        cv::waitKey(0);
    }


    // 3. Generate checkerboard (world) coordinates Q. The board has 10 x 7
    // fields with a size of 25x25mm
    std::vector<cv::Point3f> QView;
    for (int i = 0; i < 10-1; ++i) {
        for (int j = 0; j < 7-1; ++j) {
            cv::Point3f p3f(static_cast<float>(j)*50,static_cast<float>(i)*50, static_cast<float>(0)*50);
            QView.push_back(p3f);
        }
    }

    cv::Point3f* QViewPtr = QView.data();
    std::vector<std::vector<cv::Point3f>> Q(fileNames.size());
    for (int i = 0; i < fileNames.size(); ++i) { Q[i] = QView; }

    cv::Matx33f K(cv::Matx33f::eye());  // intrinsic camera matrix
    cv::Vec<float, 5> k(0, 0, 0, 0, 0); // distortion coefficients

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
    int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 +
            cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
    cv::Size frameSize(1440, 1080);

    std::cout << "Calibrating..." << std::endl;
    // 4. Call "float error = cv::calibrateCamera()" with the input coordinates
    // and output parameters as declared above...

    float error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, stdIntrinsics, stdExtrinsics, perViewErrors, flags);
    std::cout << "Reprojection error = " << error << "\nK =\n"
              << K << "\nk=\n"
              << k << std::endl;

    // Precompute lens correction interpolation
    cv::Mat mapX, mapY;
    cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1,
                                mapX, mapY);

    // Show lens corrected images
    for (auto const &f : fileNames) {
        std::cout << std::string(f) << std::endl;

        cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);

        cv::Mat imgUndistorted;
        // 5. Remap the image using the precomputed interpolation maps.

        // Display
        cv::imshow("undistorted image", imgUndistorted);
        cv::waitKey(0);
    }

    return 0;
}
