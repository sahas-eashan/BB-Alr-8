#ifndef CAMERA_CONTROLLER_HPP
#define CAMERA_CONTROLLER_HPP

#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <iostream>
#include <string>

class CameraController {
private:
  webots::Robot *robot;
  webots::Camera *camera;
  int timeStep;

public:
  // Constructor
  CameraController(webots::Robot *robot);

  // Initialize the camera
  void initializeCamera(const std::string &cameraName);

  // Process the camera feed
  void processCamera();

  // Destructor
  ~CameraController();
};

#endif // CAMERA_CONTROLLER_HPP
