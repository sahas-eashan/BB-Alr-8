#ifndef CAMERA_CONTROLLER_HPP
#define CAMERA_CONTROLLER_HPP

#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <string>
#include <iostream>

class CameraController {
private:
    webots::Camera *frontCamera;
    webots::Camera *downCamera;
    webots::Camera *scanCamera;
    webots::Robot *robot;
    int width, height;
    int timeStep;

public:
    CameraController(webots::Robot *robot);
    ~CameraController();
    void initializeCameras(const std::string& frontCamName, const std::string& downCamName, const std::string& scanCamName);
    int processScanCamera();
    char processDownCamera();
};

#endif // CAMERA_CONTROLLER_HPP