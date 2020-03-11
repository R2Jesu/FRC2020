/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

void Robot::VisionThread()
{

  // Get the USB camera from CameraServer (0)
  cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);

  // Set the resolution
  camera.SetResolution(640, 480);
  camera.SetExposureManual(5);
  camera.SetBrightness(10);
  //printf("Brightness %d\n", camera.GetBrightness());
   frc::SmartDashboard::PutNumber("brightness", camera.GetBrightness());
  //camera.SetFPS(15);

  // Get a CvSink. This will capture Mats from the Camera
  cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo(camera);
  // Setup a CvSource. This will send images back to the Dashboard
  cs::CvSource outputStream = frc::CameraServer::GetInstance()->PutVideo("Rectangle", 640, 480);

  // Mats are very memory expensive. Lets reuse this Mat.
  cv::Mat mat;
  grip::GripPipeline gp;
  while (true)
  {
   // currentDistance = 0;
    // Tell the CvSink to grab a frame from the camera and
    // put it
    // in the source mat.  If there is an error notify the
    // output.
    if (cvSink.GrabFrame(mat) == 0)
    {
      // Send the output the error.
      outputStream.NotifyError(cvSink.GetError());
      // skip the rest of the current iteration
      printf("notify error\n");
      continue;
    }
    // Put a rectangle on the image
    //      rectangle(mat, cv::Point(100, 100), cv::Point(400, 400),
    //                cv::Scalar(255, 255, 255), 5);
    gp.GripPipeline::Process(mat);
    // Give the output stream a new image to display
    for (size_t i = 0; i < (*gp.GripPipeline::GetFindContoursOutput()).size(); i++)
    {
      float contourArea = cv::contourArea((*gp.GripPipeline::GetFindContoursOutput())[i]);
      if (contourArea > 100000 || contourArea < 40)
      {
        printf("Throw away contour, area: %f\n", contourArea);
        continue;
      }
      printf("Valid countour\n");
      cv::Rect boundRect = cv::boundingRect((*gp.GripPipeline::GetFindContoursOutput())[i]);
      double centerX = boundRect.x + (boundRect.width / 2);
      // We actually want the top middle as the target center as this is only half the goal
      //double centerY = boundRect.y + (boundRect.height / 2);
      double centerY = boundRect.y;
      std::vector<cv::Point2d> ourPointVec;
      std::vector<cv::Point2d> undistortedPointVec;

      ourPointVec.push_back(cv::Point2d(centerX, centerY));
      cv::Point2d ourPoint = ourPointVec[0];
      frc::SmartDashboard::PutNumber("our x", ourPoint.x);
      frc::SmartDashboard::PutNumber("our y", ourPoint.y);
      //ourPointVec[0] = ourPoint;

      cv::Mat camMat = (cv::Mat1d(3, 3) << 667.0055536838427, 0.0, 342.42511872039944, 0.0, 664.985144080759, 237.32436945681167, 0.0, 0.0, 1.0);
      cv::Mat distortion = (cv::Mat1d(1, 5) << 0.15703749174667256, -1.134926997716282, -0.0033293254944312435, 0.0016418473011026258, 2.1006981908434668);
      cv::undistortPoints(ourPointVec, undistortedPointVec, camMat, distortion, cv::noArray(), camMat);
      cv::Point2d undistortedPoint = undistortedPointVec[0];

      //double lengthX = (centerX - 320.00) / 333.82;
      //double lengthY = -(centerY - 240.00) / 333.82;
      frc::SmartDashboard::PutNumber("undist x", undistortedPoint.x);
      frc::SmartDashboard::PutNumber("undist y", undistortedPoint.y);
      double lengthX = (undistortedPoint.x - camMat.at<double>(0, 2)) / camMat.at<double>(0, 0);
      double lengthY = -(undistortedPoint.y - camMat.at<double>(1, 2)) / camMat.at<double>(1, 1);
      frc::SmartDashboard::PutNumber("length y", lengthY);
      frc::SmartDashboard::PutNumber("length x", lengthX);
      frc::SmartDashboard::PutNumber("center y", centerY);
      frc::SmartDashboard::PutNumber("center x", centerX);

      double ax = atan2(lengthX, 1.0);
      double ay = atan2(lengthY * cos(ax), 1.0);
      //You need to remasure the camera angle and set the radians below replacing 0.139626 with whatever
      double ourDist = (98.25 - 28.00) / tan(0.20944 + ay);
      frc::SmartDashboard::PutNumber("DISTANCE", ourDist);
      cv::drawContours(mat, *gp.GripPipeline::GetFindContoursOutput(), i, cv::Scalar(255, 0, 0), 3);
      if ((centerX > 300) && (centerX < 340))
     { rectangle(mat, cv::Point(centerX - 10, centerY - 10), cv::Point(centerX + 10, centerY + 10), cv::Scalar(0, 0, 255), 5);
     } else {
      rectangle(mat, cv::Point(centerX - 10, centerY - 10), cv::Point(centerX + 10, centerY + 10), cv::Scalar(255, 0, 0), 5);}
      currentDistance = ourDist;
      frc::SmartDashboard::PutNumber("current distance", currentDistance);
    }
    outputStream.PutFrame(mat);
  }
}
