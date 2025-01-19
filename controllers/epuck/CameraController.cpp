#include "CameraController.hpp"

CameraController::CameraController(webots::Robot *robot) : robot(robot) {
  timeStep = (int)robot->getBasicTimeStep();
}

void CameraController::initializeCamera(const std::string &cameraName) {
  camera = robot->getCamera(cameraName);
  if (camera) {
    camera->enable(timeStep);
    std::cout << "Camera " << cameraName << " initialized successfully." << std::endl;
  } else {
    std::cerr << "Error: Camera " << cameraName << " not found!" << std::endl;
    exit(1);
  }
}

void CameraController::processCamera() {
  if (!camera) {
    std::cerr << "Error: Camera not initialized!" << std::endl;
    return;
  }

  const unsigned char *image = camera->getImage();
  int width = camera->getWidth();
  int height = camera->getHeight();

  if (image) {
    int red = camera->imageGetRed(image, width, width / 2, height / 2);
    int green = camera->imageGetGreen(image, width, width / 2, height / 2);
    int blue = camera->imageGetBlue(image, width, width / 2, height / 2);

    std::cout << "Center pixel RGB: (" << red << ", " << green << ", " << blue << ")" << std::endl;
  }
}

CameraController::~CameraController() {
  // Cleanup if necessary
}
