# RoboGames 2024 - University Category Preliminary Round Submission | Team BB-Alr-8

Welcome to the GitHub repository for **Team BB-Alr-8**. This repository contains all the files related to our submission for the RoboGames 2024 Completion Round in the University Category.

## Project Overview

Our task was to design a Webots simulation environment and program an E-puck robot to navigate a maze based on a specific color pattern:  
**Red → Yellow → Pink → Brown → Green**.

The robot achieves the goal by:
- Following the colors sequentially in the arena starting from any given position.
- Stopping once the goal is completed.

## Repository Structure

- **`.vscode/`**: Configuration files for the VS Code IDE to streamline the development process.
- **`controllers/epuck/`**: Contains the controller logic for the E-puck robot to execute the task.
- **`protos/`**: Includes the custom prototype objects used in the Webots simulation.
- **`worlds/RoboGames 2024 University Category(BB-Alr-8).wbt`**: The Webots world file designed according to the specifications.
- **`RoboGames 2024 University Category(BB-Alr-8).mp4`**: Simulation demonstration video.

## Arena Specifications

The arena was designed as per the RoboGames guidelines:
- Dimensions: **2.5m x 2.5m** grid.
- Wall spacing: **0.25m** gap between walls.
- Wall dimensions: **Height: 0.1m, Breadth: 0.01m, Length: multiple of 0.25m**.
- Colored walls include: **Red (#FF0000), Yellow (#FFFF00), Pink (#FF00FF), Brown (#A5691E), Green (#00FF00)**.

## How to Run

1. Clone the repository to your local system:
   ```bash
   git clone https://github.com/yourusername/BB-Alr-8.git
2. Open the Webots simulation platform.
3. Load the Webots world file: `RoboGames 2024 University Category(BB-Alr-8).wbt.`
4. Remove intermediate build files.
5. Build the Controller Code
6. Run the simulation and observe the robot perform the task.
