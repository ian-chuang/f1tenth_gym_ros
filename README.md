# UCDavis F1Tenth Algorithms + Simulation

This guide will walk you through the steps to set up and run the UC Davis F1Tenth autonomous driving + simulation using Docker and VS Code. Follow these instructions to get started.

## Prerequisites

1. **GitHub Desktop**: Install GitHub Desktop from [here](https://desktop.github.com/).
2. **Docker Desktop**: Install Docker Desktop from [here](https://www.docker.com/products/docker-desktop).
3. **VS Code**: Install Visual Studio Code from [here](https://code.visualstudio.com/).
4. **VS Code Extensions**: Install the following extensions in VS Code:
    - Remote Development extension pack
    - Docker extension

## Setup

1. **Clone the Repository:**
    - Open GitHub Desktop.
    - Clone the repository [https://github.com/ian-chuang/f1tenth_gym_ros](https://github.com/ian-chuang/f1tenth_gym_ros).

2. **Open the Repository in VS Code:**
    - Open VS Code.
    - Open the cloned repository using `File -> Open Folder`.

3. **Start Docker Container:**
    - Open the terminal in VS Code.
    - Navigate to the repository folder using the `cd /path/to/f1tenth_gym_ros` command.
    - Type `docker-compose up` in the terminal. Try `docker compose up` if the previous command didn't work.

4. **Attach VS Code to Docker Container:**
    - Navigate to the Docker tab in VS Code.
    - Right-click on the container named `f1tenth_gym_ros`.
    - Press "Attach Visual Studio Code" to open a VS Code window within your container.

5. **Open Simulation Workspace:**
    - In the new VS Code window, go to `File -> Open Folder`.
    - Enter the path `/sim_ws` and press Enter.

6. **Launch Autonomous Driving Simulation:**
    - Open a terminal in VS Code (`Terminal -> New Terminal`).
    - Enter the command `ros2 launch f1tenth_gym_ros bringup_launch.py`.
    - Open a web browser and go to [http://localhost:8080/vnc.html](http://localhost:8080/vnc.html).
    - Press "Connect" to view the RViz window of the simulation.
    - Set the location of the car in the simulation by first clicking the "2D Pose Estimate" button in RViz and then click and drag on a location in the map to set the pose.
    - After setting the pose, the car will start at that location and start running the autonomous algorithms.

7. **Run Unit Tests:**
    - Open a terminal in VS Code (`Terminal -> New Terminal`).
    - Enter the command `cd /sim_ws && source ./run_tests.sh`.

## Code Explanation

Our autonomous driving solution consists of multiple parts defined in the ROS2 packages in the `src` folder:

- **f1tenth_gym_ros**: Contains configuration files, racelines, maps, and launch files for the simulation. The main launch file `bringup_launch.py` starts the simulator and core algorithms (pure pursuit, obstacle detection, and gap follow).
- **pure_pursuit**: This contains implementation of pure pursuit in Python for following raceline.
- **gap_follow**: This contains the C++ implementation of gap follow algorithm for avoiding obstacles.
- **obs_detect**: This contains code for an obstacle detection node that detects if an obstacle is in the way of the raceline. We use this node to switch between running the pure pursuit algorithm and gap follow.
- **particle_filter**: We don't run this algorithm in simulation since localization is auto computed in simulation. We do use this however on the real car.
- **emergency_braking**: Safety node to stop the car if it approaches an obstacle too quickly.
