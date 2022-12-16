/*  MIT License

    Copyright (c) 2022 Dhinesh Rajasekaran

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
    deal in the Software without restriction, including without limitation the
    rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
    sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
    IN THE SOFTWARE.
*/

/**
 * @copyright (c) 2022 Dhinesh Rajasekaran
 * @file talker.cpp
 * @author Dhinesh Rajasekaran (dhinesh@umd.edu)
 * @brief This is a ROS 2 TF2 static broadcaster node
 * string
 * @version 1.0
 * @date 2022-11-15
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "std_msgs/msg/string.hpp"

// Using Namespace to improve code readability
using namespace std;
using namespace rclcpp;

/** 
 * @brief This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer.
 *
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hi, This is ROS2 " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

class StaticFramePublisher : public Node
{
public:
  explicit StaticFramePublisher(char * transformation[])
  : Node("tf2_broadcaster_static")
  {
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->make_transforms(transformation); ///< Publish static transforms once at startup
  }

private:
  void make_transforms(char * transformation[])
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = transformation[1];

    t.transform.translation.x = atof(transformation[2]); ///< Translation vector along x
    t.transform.translation.y = atof(transformation[3]); ///< Translation vector along y
    t.transform.translation.z = atof(transformation[4]); ///< Translation vector along z

    tf2::Quaternion q;
    q.setRPY(
      atof(transformation[5]),
      atof(transformation[6]),
      atof(transformation[7]));
      
    t.transform.rotation.x = q.x(); ///< Rotation vector along x
    t.transform.rotation.y = q.y(); ///< Rotation vector along y
    t.transform.rotation.z = q.z(); ///< Rotation vector along z
    t.transform.rotation.w = q.w(); 

    tf_static_broadcaster_->sendTransform(t); ///< Transformation matrix 
  }

  shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

/**
 * @brief Main function which initialises ROS, receives command line arguement and 
 * checks frame name.
 * @param argc stores the no of arguements to send & receive in command line
 * @param argv stores the actual arguement data to send and receive in command
 * line
 * @return zero
 */
int main(int argc, char * argv[])
{
  auto logger = get_logger("logger");

  if (argc != 8) { ///< Obtain parameters from command line arguments:
    RCLCPP_INFO(
      logger, "Invalid number of parameters\nusage: "
      "$ ros2 run learning_tf2_cpp tf2_broadcaster_static "
      "child_frame_name x y z roll pitch yaw");
    return 1;
  }

  if (strcmp(argv[1], "world") == 0) { ///< As the parent frame of the transform is `world`, it is necessary to check that the frame name passed is different
    RCLCPP_INFO(logger, "Your static bot name cannot be 'world'");
    return 1;
  }

  init(argc, argv); ///< initialise ROS communication framework
  spin(make_shared<MinimalPublisher>());
  spin(make_shared<StaticFramePublisher>(argv)); ///< spin the node
  shutdown(); ///< shutdown or stop the node which was init
  return 0;
}