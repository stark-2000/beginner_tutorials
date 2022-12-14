[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
---
# Overview:
 - This repository consists of a basic server-client node implemented in ROS2
 - The server receives input string from client & sends back modifed string to client.
 - The client receives input string from user and sends the request to server and gets back the response which is modified string and prints it.
 - The client also has a class MinimalParam running which accepts command line arguement and modifies the string displayed.
 - Launch file is also created which launches all the nodes at once and modifies one parameter in the client node.
 - Google Style Guide was followed in all the cpp files.
 - CPPCHECK and CPPLINT was run to perform static code analysis.
 - "results" folder contains the output of CPPCHECK and CPPLINT in a text file.
 - doxygen documentation is added to the "docs" folder for all the cpp files.
 - rqt console was run which shows the messages published with 2 different logger levels before and after sending the command line arguements.

## Personnel:
 - Dhinesh Rajasekaran 
    - UID: 119400241

## Dependencies/Requirements: 
 - Laptop
 - Ubuntu 20.04 or higher
 - VS Code/Terminal
 - ROS 2 Humble

## Creating Workspace & Cloning Repo: 
 - Open terminal and source ros2
 ```
 source <ros2_humble_installation_directory>/install/setup.bash
 ```

 - Create a workspace (if not created before)
 ```
 mkdir -p ~/ros2_ws/src
 cd ~/ros2_ws/src
 ```

 - Navigate to your current workspace (if created before)
 - Clone the repo into src directory of your workspace
 ```
 git clone https://github.com/stark-2000/beginner_tutorials.git
 ```

## Build Instructions:
 - Open new terminal & navigate to root of workspace
 ```
 cd <ros2_workspace_name>
 ```
   - or
 ```
 cd ros2_ws
 ```

 - Check for missing dependencies before building the package
 ```
 rosdep install -i --from-path src --rosdistro humble -y
 ```

 - Build the package
 ```
 colcon build --packages-select beginner_tutorials
 ```

## Basic Run Instructions:
  - Running server node:
      - Open new terminal & navigate to root of workspace
      ```
      cd ros2_ws
      ```
        - or
      ```
      cd <ros2_workspace_name>
      ```
      - Source ros2
      ```
      . install/setup.bash
      ```

      - Run server node
      ```
      ros2 run beginner_tutorials node_pubs
      ```
      - Run server node with custom logger level
      ```
      ros2 run beginner_tutorials node_pubs --ros-args --log-level WARN
      ```

  - Running client node (server node should be running):
      - Open new terminal & navigate to root of workspace
      ```
      cd ros2_ws
      ```
        - or
      ```
      cd <ros2_workspace_name>
      ```
      - Source ros2
      ```
      . install/setup.bash
      ```

      - Run client node by changing "string_to_modify" which will be received by client for modification
      ```
      ros2 run beginner_tutorials node_subs <string_to_modify>
      ```
      - Change one parameter in "node_pubs.cpp" from command line
      ```
      ros2 param set /minimal_param_node my_parameter earth
      ```

## Run from Launch File:
 - Run launch file which launches all the nodes at once & modifies one parameter in node_pubs.cpp
 ```
 ros2 launch beginner_tutorials node_launch.py
 ```

## View logs in rqt console:
 - open new terminal, navigate to ROS 2 workspace and source ROS2
 - open rqt console
 ```
 ros2 run rqt_console rqt_console
 ```

 - open another terminal, navigate to ROS 2 workspace and source ROS2
 - Run server node
 ```
 ros2 run beginner_tutorials node_pubs
 ```

 - open another terminal, navigate to ROS 2 workspace and source ROS2
 - Run client node by changing "string_to_modify" which will be received by client for modification
 ```
 ros2 run beginner_tutorials node_subs <string_to_modify>
 ```

## Command to run static code analysis:
 - Navigate to src folder in package
 ```
 cd <ros2_workspace>/src/beginner_tutorials/src
 ```
 - run the following command
 ```
 cppcheck --enable=all --std=c++17 *.cpp --suppress=missingIncludeSystem --suppress=missingInclude --suppress=unmatchedSuppression > ./../results/cppcheckreport
 ```

## Command to check Google Style:
 - Navigate to src folder in package
 ```
 cd <ros2_workspace>/src/beginner_tutorials/src
 ```
 - run the following command
 ```
 cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order node_pubs.cpp node_subs.cpp > ./../results/cpplintreport
 ```

## Command to generate doxygen documentation:
 - Navigate to src folder in package
 ```
 cd <ros2_workspace>/src/beginner_tutorials/src
 ```
 - run the following command
 ```
 doxygen node_subs.cpp
 cd html
 firefox index.html
 ```

## Dependency Installation: 
- ROS 2 Humble:
- Follow the below website instructions to install ROS 2 Humble based on your Ubuntu version
  - Ubuntu 22.04:
    - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#install-ros-2-packages
  
  - Ubuntu 20.04 (binary package):
    - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#install-ros-2-packages

  - Ubuntu 20.04 (build from source):
    - https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html

 - srv package:
    - A custom written srv file is used in this project which is needed for runing this program
    - It's called "tutorial_interfaces/srv/modify_string.hpp". 
    - This folder called "tutorial_interfaces" is a ROS package which has to be placed inside src directory of ros2 workspace along with other packages
    - Then execute the following command:
    ```
    colcon build --packages-select tutorial_interfaces
    ```
    - This will build the dependency of custom service required