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
 * @file node_pubs.cpp
 * @author Dhinesh Rajasekaran (dhinesh@umd.edu)
 * @brief This is a ROS server node which uses a custom service to modify a
 * string
 * @version 1.0
 * @date 2022-11-15
 *
 */
#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/modify_string.hpp"

// Using Namespace to improve code readability
using namespace std;
using namespace rclcpp;
using namespace tutorial_interfaces::srv;

/**
 * @brief User defined function to modify incoming string
 * @param request stores the incoming string from command line
 * @param response stores the output string to be sent back to client
 * @return zero
 */
void modify(const shared_ptr<ModifyString::Request> request,
            shared_ptr<ModifyString::Response> response) {
  response->mod_string =
      request->to_modify +
      string(" Hi");  ///< string being modifed with additional "Hi"

  RCLCPP_WARN(get_logger("rclcpp"), "Incoming Request:\nString: %s",
              request->to_modify.c_str());  ///< display the incoming string

  RCLCPP_INFO(get_logger("rclcpp"), "sending back response: %s",
              response->mod_string.c_str());  ///< display the modified string
}

/**
 * @brief Main function which initialises ROS communication and service
 * @param argc stores the no of arguements to send & receive in command line
 * @param argv stores the actual arguement data to send and receive in command
 * line
 * @return zero
 */
int main(int argc, char **argv) {
  init(argc, argv);  ///< initialise ROS communication framework
                     // argc & argv are the 2 main arguments which will store
                     // the no of request and response variables. Here we have 1
                     // input and 1 output Hence, argc is 2 and argv will have
                     // the input and output data

  shared_ptr<Node> node = Node::make_shared(
      "Modify_String_Server");  ///< creating shared pointer
                                ///< called node with the server name

  Service<ModifyString>::SharedPtr service = node->create_service<ModifyString>(
      "Modify_String_Server",
      &modify);  ///< Using the node, declaring the server
                 ///< Server name given in () & srv name in <>
                 ///< binding the server with function call

  RCLCPP_INFO(get_logger("rclcpp"),
              "Ready to Modify String... Waiting for change");

  spin(node);  ///< spin the node
  shutdown();  ///< shutdown or stop the node which was init
  return 0;
}
