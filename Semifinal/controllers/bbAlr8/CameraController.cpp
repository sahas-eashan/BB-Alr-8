#include "CameraController.hpp"

CameraController::CameraController(webots::Robot *robot) : robot(robot) {
  timeStep = (int)robot->getBasicTimeStep();
  frontCamera = nullptr;
  downCamera = nullptr;
}

// void CameraController::initializeCamera(const std::string &cameraName) {
//   camera = robot->getCamera(cameraName);
//   if (camera) {
//     camera->enable(timeStep);
//     width = camera->getWidth();
//     height = camera->getHeight();
//     std::cout << "Camera " << cameraName << " initialized successfully." << std::endl;
//     std::cout << "Camera resolution: " << width << "x" << height << std::endl;
//   } else {
//     std::cerr << "Error: Camera " << cameraName << " not found!" << std::endl;
//     exit(1);
//   }
// }

void CameraController::initializeCameras(const std::string& frontCamName, const std::string& downCamName, const std::string& scanCamName) {
    frontCamera = robot->getCamera(frontCamName);
    downCamera = robot->getCamera(downCamName);
    scanCamera = robot->getCamera(scanCamName);
    
    if (frontCamera) {
        frontCamera->enable(timeStep);
        std::cout << "Front camera initialized successfully" << std::endl;
    } else {
        std::cerr << "ERROR: Failed to initialize front camera!" << std::endl;
    }
    
    if (downCamera) {
        downCamera->enable(timeStep);
        std::cout << "Down camera initialized successfully" << std::endl;
    } else {
        std::cerr << "ERROR: Failed to initialize down camera!" << std::endl;
    }

        if (scanCamera) {
        scanCamera->enable(timeStep);
        std::cout << "Scan camera initialized successfully" << std::endl;
    } else {
        std::cerr << "ERROR: Failed to initialize scan camera!" << std::endl;
    }

}

char CameraController::processDownCamera() {
  if (!downCamera) {
    std::cerr << "Error: Down camera not initialized!" << std::endl;
    return 'E';
  }

  const unsigned char *image = downCamera->getImage();
  if (!image) {
    std::cerr << "Error: Failed to get camera image!" << std::endl;
    return 'E';
  }
  
  // Get RGB values for pixel at (0,0)
  int r = downCamera->imageGetRed(image, downCamera->getWidth(), 0, 0);
  int g = downCamera->imageGetGreen(image, downCamera->getWidth(), 0, 0);
  int b = downCamera->imageGetBlue(image, downCamera->getWidth(), 0, 0);
  
  std::cout << "Pixel (0,0) RGB values: R=" << r << " G=" << g << " B=" << b << std::endl;

 //   r  g  b
 //R 70 11 15
 //O 70 27 15
 //Y 70 76 50
 //W 73 93 137

    if (r >= 230 && g < 50 && b < 50) {
        return 'R';
    } else if (r >= 230 && g > 100 && g < 150 && b < 50) {
        return 'O';
    } else if (r >= 230 && g >= 230 && b >= 140 && b <= 170) {
        return 'Y';
    } else if (r >= 230 && g >= 230 && b >= 230) {
        return 'W';
    } else {
        return 'U';
    }


 // Process the entire image
 // for (int y = 0; y < height; y++) {
  //  for (int x = 0; x < width; x++) {
      // int red = camera->imageGetRed(image, width, x, y);
      // int green = camera->imageGetGreen(image, width, x, y);
      // int blue = camera->imageGetBlue(image, width, x, y);
      
      // Here you can add your image processing logic

      // For example, detect specific colors, shapes, or objects
   // }
  
  return 'U';
}

char CameraController::processFrontCamera() {
  if (!frontCamera) {
    std::cerr << "Error: Front camera not initialized!" << std::endl;
    return 'E';
  }

  const unsigned char *image = frontCamera->getImage();
  if (!image) {
    std::cerr << "Error: Failed to get camera image!" << std::endl;
    return 'E';
  }
  // Process the entire image
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // int red = camera->imageGetRed(image, width, x, y);
      // int green = camera->imageGetGreen(image, width, x, y);
      // int blue = camera->imageGetBlue(image, width, x, y);
      
      // Here you can add your image processing logic

      // For example, detect specific colors, shapes, or objects
    }
  }
  return 'W';
  
}

CameraController::~CameraController() {
  // Cleanup if necessary
}