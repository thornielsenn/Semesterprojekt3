#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pylon/PylonIncludes.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string>

using namespace cv;

int main(int argc, char* argv[])
{
    int myExposure = 30000;

    // The exit code of the sample application.
    int exitCode = 0;

    // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm autoInitTerm;

    try
    {
        // Create an instant camera object with the camera device found first.
        Pylon::CInstantCamera camera( Pylon::CTlFactory::GetInstance().CreateFirstDevice());

        // Get a camera nodemap in order to access camera parameters.
        GenApi::INodeMap& nodemap= camera.GetNodeMap();

        // Open the camera before accessing any parameters.
        camera.Open();
        // Create pointers to access the camera Width and Height parameters.
        GenApi::CIntegerPtr width= nodemap.GetNode("Width");
        GenApi::CIntegerPtr height= nodemap.GetNode("Height");

        // The parameter MaxNumBuffer can be used to control the count of buffers
        // allocated for grabbing. The default value of this parameter is 10.
        //camera.MaxNumBuffer = 5;

        // Create a pylon ImageFormatConverter object.
        Pylon::CImageFormatConverter formatConverter;
        // Specify the output pixel format.
        formatConverter.OutputPixelFormat= Pylon::PixelType_BGR8packed;
        // Create a PylonImage that will be used to create OpenCV images later.
        Pylon::CPylonImage pylonImage;

        // Create an OpenCV image.
        cv::Mat openCvImage;


        // Set exposure to manual
        GenApi::CEnumerationPtr exposureAuto( nodemap.GetNode( "ExposureAuto"));
        if ( GenApi::IsWritable( exposureAuto)){
            exposureAuto->FromString("Off");
            std::cout << "Exposure auto disabled." << std::endl;
        }

        // Set custom exposure
        GenApi::CFloatPtr exposureTime = nodemap.GetNode("ExposureTime");
        std::cout << "Old exposure: " << exposureTime->GetValue() << std::endl;
        if(exposureTime.IsValid()) {
            if(myExposure >= exposureTime->GetMin() && myExposure <= exposureTime->GetMax()) {
                exposureTime->SetValue(myExposure);
            }else {
                exposureTime->SetValue(exposureTime->GetMin());
                std::cout << ">> Exposure has been set with the minimum available value." << std::endl;
                std::cout << ">> The available exposure range is [" << exposureTime->GetMin() << " - " << exposureTime->GetMax() << "] (us)" << std::endl;
            }
        }else {

            std::cout << ">> Failed to set exposure value." << std::endl;
            return false;
        }
        std::cout << "New exposure: " << exposureTime->GetValue() << std::endl;

        // Start the grabbing of c_countOfImagesToGrab images.
        // The camera device is parameterized with a default configuration which
        // sets up free-running continuous acquisition.
        camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

        // This smart pointer will receive the grab result data.
        Pylon::CGrabResultPtr ptrGrabResult;

        // image grabbing loop
        int frame = 1;
        while ( camera.IsGrabbing())
        {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            camera.RetrieveResult( 5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                // Access the image data.
                //cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
                //cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;

                // Convert the grabbed buffer to a pylon image.
                formatConverter.Convert(pylonImage, ptrGrabResult);

                // Create an OpenCV image from a pylon image.
                openCvImage= cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());


                //////////////////////////////////////////////////////
                //////////// Here your code begins ///////////////////
                //////////////////////////////////////////////////////

                /// Detect and show objects and targets

                {  std::vector<cv::String> fileNames;
                    cv::glob("/home/thor1234/Documents/Semesterprojekt/Calibration/Image/image*.png", fileNames, false); //file path til gemte billede fra kamera
                    cv::Size patternSize(10 - 1, 7 - 1); // finder kanter på skakbræt, -1 da vi ikke kan finde kanter på de hvide stykker for enderne
                    std::vector<std::vector<cv::Point2f>> q(fileNames.size());


                    if (fileNames.size() < 20) {
                        std::cerr << "You need more pictures" << std::endl;
                        exit(-1);
                    }

                    // Detect feature points
                    for (std::size_t i = 0; i < fileNames.size(); ++i) {
                        //std::cout << std::string(f) << std::endl;

                        // 1. Read in the image an call cv::findChessboardCorners()
                        cv::Mat img = cv::imread(fileNames [i]); // bruger 'imread' til at læse billeder der ligger i arrayet
                        bool success = cv::findChessboardCorners(img, patternSize, q[i]);
                        if (success) {
                            cv::Mat grey; //
                            cv::cvtColor(img, grey, CV_BGR2GRAY);
                            // 2. Use cv::cornerSubPix() to refine the found corner detections
                            cv::cornerSubPix(grey, q[i], cv::Size(5,5), cv::Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                        }
                        cv::drawChessboardCorners(img, patternSize, q[i], success);

                        cv::drawChessboardCorners(img,patternSize,q[i],success);
                        cv::imshow("chessboard detection",img);
                        cv::waitKey(0);
                    }

                    // 3. Generate checkerboard (world) coordinates Q. The board has 10 x 7
                    // fields with a size of 50 x 50mm
                    std::vector<cv::Point3f> QView;
                    for (int i{0}; i < 7-1; i++)
                    {
                        for (int j{0}; j < 10-1; j++)
                        {
                            QView.push_back(cv::Point3f(j*50,i*50,0.0f));
                        }
                    }

                    cv::Point3f* QViewPtr = QView.data();
                    std::vector<std::vector<cv::Point3f>> Q(fileNames.size());
                    for (int i = 0; i < fileNames.size(); ++i) { Q[i] = QView; }

                    cv::Matx33f K(cv::Matx33f::eye());  // intrinsic camera matrix
                    cv::Vec<float, 5> k(0, 0, 0, 0, 0); // distortion coefficients

                    std::vector<cv::Mat> rvecs, tvecs;
                    std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
                    int flags = cv::CALIB_FIX_ASPECT_RATIO;// + cv::CALIB_FIX_K3 +
                    // cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
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
                    cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1, mapX, mapY);

                    // Show lens corrected images
                    for (auto const &f : fileNames) {
                        // std::cout << std::string(f) << std::endl;

                        cv::Mat imgUndistorted;
                        // 5. Remap the image using the precomputed interpolation maps.
                        cv::remap(openCvImage,imgUndistorted,mapX,mapY,1);

                        Mat gray;
                        cvtColor(imgUndistorted, gray, COLOR_BGR2GRAY);
                        medianBlur(gray, gray, 5);
                        std :: vector<Vec3f> circles;
                        HoughCircles(gray, circles, HOUGH_GRADIENT, 1,
                                     gray.rows/16,  // change this value to detect circles with different distances to each other
                                     100, 30, 15, 25 // change the last two parameter // (min_radius & max_radius) to detect larger circles
                                     );
                        for( size_t i = 0; i < circles.size(); i++ )
                        {
                            Vec3i c = circles[i];
                            Point center = Point(c[0], c[1]);
                            // circle center
                            circle( imgUndistorted, center, 1, Scalar(0,100,100), 3, LINE_AA);
                            // circle outline
                            int radius = c[2];
                            circle( imgUndistorted, center, radius, Scalar(57,255,25), 3, LINE_AA);
                            std::cout << i + 1 << " centerpoint " << " x: " << circles[i][0] << "   y: " << circles[i][1] << "    rad: " << circles[i][2] << std::endl;
                        }

                        //cropper billedet til størrelsen af bordpladen.
                        cv::Rect myROI(450, 0 , 600, 900);
                        imshow ("detected circles cropped",imgUndistorted);
                    }





                    ////////////////////////////////////////////////////
                    //////////// Here your code ends ///////////////////
                    ////////////////////////////////////////////////////

                    frame++;

                    int keyPressed;
                    keyPressed = cv::waitKey(0);
                    //std::cout << "Value: " << (char) ::recievedCMD << std::endl;
                    if(keyPressed == 'q' || keyPressed == 'Q') { // quit
                        std::cout << "Shutting down camera..." << std::endl;
                        camera.Close();
                        std::cout << "Camera successfully closed.";
                    }
                }

            }
            else
            {
                std::cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
            }
        }

    }
    catch (GenICam::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
        exitCode = 1;
    }
}
