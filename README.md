# Overview:
 - This repository consists of a basic publisher-subscriber node implemented in ROS2
 - The publisher publishes a custom string "Hi, This is ROS2 " followed by message count
 - The subscriber receives this data and prints "I got the data: " followed by the received data
 - Google Style Guide was followed in all the cpp files
 - CPPCHECK and CPPLINT was run to perform static code analysis
 - results folder contains the output of CPPCHECK and CPPLINT in a text file

## Personnel:
 - Dhinesh Rajasekaran 
    - UID: 119400241

## Dependencies/Requirements: 
 - Laptop
 - Ubuntu 20.04 or higher
 - VS Code/Terminal
 - ROS 2 Humble

## Build & Run Instructions:
 - open terminal and source ros2
 ```
 source <ros2_humble_installation_directory>/install/setup.bash
 ```

 - create a workspace (if not created before)
 ```
 mkdir -p ~/ros2_ws/src
 cd ~/ros2_ws/src
 ```

 - cd into your current workspace (if created before)
 - clone the repo into src directory of your workspace
 ```
 git clone https://github.com/stark-2000/beginner_tutorials.git
 ```

 - Navigate to root of workspace
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

 - open new terminal
 - Navigate to root of workspace
 ```
 cd ros2_ws
 ```
    - or
 ```
 cd <ros2_workspace_name>
 ```

 - source ros2
 ```
 . install/setup.bash
 ```

 - run publisher node
 ```
 ros2 run beginner_tutorials talker
 ```

 - open new terminal
 - Navigate to root of workspace
 ```
 cd ros2_ws
 ```
    - or
 ```
 cd <ros2_workspace_name>
 ```

 - source ros2
 ```
 . install/setup.bash
 ```

 - run subscriber node
 ```
 ros2 run beginner_tutorials listener
 ```

## Command to run static code analysis:
 - Navigate to src folder in package
 ```
 cd ros2_ws/src/beginner_tutorials/src
 ```
 - run the following command from src folder in package
 ```
 cppcheck --enable=all --std=c++17 *.cpp --suppress=missingIncludeSystem --suppress=missingInclude --suppress=unmatchedSuppression > ./../results/cppcheckreport
 ```

## Command to check Google Style:
 - Navigate to src folder in package
 ```
 cd ros2_ws/src/beginner_tutorials/src
 ```
 - run the following command from src folder in package
 ```
 cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order subscriber.cpp publisher.cpp > ./../results/cpplintreport
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