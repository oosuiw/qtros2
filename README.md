# QtROS2 GUI Application
## Project Overview
This project integrates **Qt** and **ROS 2** to create an application that runs a GUI and ROS 2 nodes
simultaneously.
It allows user interaction via GUI while leveraging ROS 2's capabilities for robot control and data
processing.
### Key Features
- **Qt-based GUI**: Provides a user-friendly graphical interface.
- **ROS 2 Integration**: Utilizes ROS 2's powerful messaging and service architecture.
- **Multithreading Support**: Uses `MultiThreadedExecutor` for parallel ROS 2 callback processing.
- **Signal-safe Shutdown**: Handles SIGINT (Ctrl+C) signal for resource-safe application termination.
---
## Components
### 1. **ROS 2 Node (`Ros2Node`)**
- Implements core functionality of a ROS 2 node.
- Handles message publishing/subscribing, service calls, etc.
### 2. **GUI Application (`MainGUI`)**
- Qt-based GUI for user interaction.
- Connects GUI widgets like buttons and input fields with ROS 2 node functionality.
### 3. **Main Loop (`main.cpp`)**
- Entry point of the application, initializing `QApplication` and ROS 2, and running the event loop.
---
## Installation and Execution
### **1. Prerequisites**
To run this project, ensure the following software is installed:
- **ROS 2 (Humble, Iron, Galactic, etc.)**
Refer to the [official guide](https://docs.ros.org/en/rolling/Installation.html) for installation.
- **Qt (>= 5.15)**
Download Qt from the [official Qt page](https://www.qt.io/download).
- **CMake (>= 3.10)**
Visit the [CMake download page](https://cmake.org/download/).
### **2. Build and Run**
1. Clone this repository:
```bash
git clone https://github.com/yourusername/qtros2-gui-app.git
cd qtros2-gui-app
```
2. Set up your ROS 2 workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ln -s .
cd ~/ros2_ws
colcon build
```
3. Source the environment:
```bash
source /opt/ros//setup.bash
source ~/ros2_ws/install/setup.bash
```
4. Run the application:
```bash
ros2 run qtros2_gui_app qtros2_gui_app
```
---
## Usage
### GUI Features
- **Data Display**: Show real-time data received from ROS 2 nodes in the GUI.
- **Control Input**: Send commands to ROS 2 nodes via the GUI.
- **State Monitoring**: Monitor the state of ROS 2 nodes and safely terminate when needed.
### ROS 2 Node Features
- **Topic Publishing/Subscribing**: Publish messages based on GUI inputs and display subscribed
data.
- **Service Calls**: Communicate with ROS 2 services to perform specific tasks.
---

---
## Contribution
1. Fork this repository.
2. Create a new branch:
```bash
git checkout -b feature/new-feature
```
3. Commit your changes:
```bash
git commit -am "Add new feature"
```
4. Push the branch:
```bash
git push origin feature/new-feature
```
5. Open a Pull Request to submit your changes.
---
## Reporting Issues
Report bugs or suggestions via the [Issues
tab](https://github.com/yourusername/qtros2-gui-app/issues).
---
## License
This project is distributed under the [MIT License](LICENSE).
---
## References
- [Qt Documentation](https://doc.qt.io/)
- [ROS 2 Documentation](https://docs.ros.org/)
- [CMake Documentation](https://cmake.org/documentation/)
