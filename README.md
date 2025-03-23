# gz_planar_move

![ROS 2](https://img.shields.io/badge/ROS2%20-Jazzy-blue.svg) [![CI](https://github.com/robotont/robotont_nuc_description/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/robotont/robotont_nuc_description/actions/workflows/industrial_ci_action.yml) ![License](https://img.shields.io/badge/License-Apache_2.0-green.svg)

## **Overview**
Package for controlling a 3-wheeled omnidirectional robot in gz sim 8.
## **Table of Contents**
- [Installation](#installation)
- [Dependencies](#dependencies)
- [Building the Package](#building-the-package)
- [Usage](#launch-files)
- [License](#license)

---

## **Installation**

### **1. Clone the Repository**
```bash
cd ~/<YOUR_WORKSPACE_NAME_HERE>/src
git clone https://github.com/sygism/robotont_nuc_description.git
```

## **Dependencies**
### **1. List of dependencies**
1.1. rclcpp
1.2. geometry_msgs<
1.3. gz-sim8
1.4. gz-transport13

### **2. Install dependencies**
```bash
cd ~/<YOUR_WORKSPACE_NAME_HERE>
rosdep install --from-paths src --ignore-src -r -y
```

## **Building the package**
```bash
cd ~/<YOUR_WORKSPACE_NAME_HERE>
colcon build --packages-select robotont_nuc_description
```

## **Usage**
### **1. Source workspace**
```bash
source ~/<YOUR_WORKSPACE_NAME_HERE>/install/setup.bash
```
### **2. Usage in xacro urdf**
```xml
<plugin name="gz_planar_move::PlanarMove" filename="$(find gz_planar_move)/lib/libgz_planar_move.so">
    <ns>robot</ns> <!-- namespace in gazebo -->
    <back_joint>back_wheel_joint</back_joint> <!-- name of the back wheel joint -->
    <left_joint>left_wheel_joint</left_joint> <!-- name of the left wheel joint -->
    <right_joint>right_wheel_joint</right_joint> <!-- name of the right wheel joint -->
    <wheel_distance>0.14</wheel_distance> <!-- distance from the center to the wheels -->
    <wheel_radius>0.062</wheel_radius> <!-- radius of the wheels -->
    <velocity_cmd_topic>/cmd_vel</velocity_cmd_topic> <!-- topic the plugin should subscribe to for velocity commands (Twist) -->
</plugin>
```

## **License**
This project is licensed under the Apache 2.0 license - see the [LICENSE](LICENSE) file for more information.
