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
 * @file node_subs.cpp
 * @author Dhinesh Rajasekaran (dhinesh@umd.edu)
 * @brief This is a ROS client node which uses a custom service to modify a
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
 * @brief This class receives command line parameter and modifies the string
 * it's printing already. It printes "hello world" by default along with the
 * client running in main function and wiats for input from command line through
 * launch file or other methods, when received the string from command line
 * modifies the second word "world" in "hello world".
 *
 */
class MinimalParam : public Node {
 public:
  MinimalParam()  ///< Constructor for default init of tasks
      : Node("minimal_param_node") {
    this->declare_parameter(
        "my_parameter",
        "world");  ///< command line parameter init - parameter name is
                   ///< "my_parameter" & it holds string "world", data type of
                   ///< "my_parameter" is automatic, it takes based on what is
                   ///< stored in it - in this case it is string since "world"
                   ///< is string

    string my_param =
        this->get_parameter("my_parameter")
            .get_parameter_value()
            .get<string>();  ///< get command line parameter entered and store
                             ///< in my_param - if no data entered, it takes
                             ///< default stored data

    RCLCPP_WARN(this->get_logger(), "Going to Modify String from Command Line: ");
    RCLCPP_INFO(this->get_logger(), "Hello %s!",
                my_param.c_str());  ///< display the data stored with "hello" at
                                    ///< the start of the modified string

    vector<Parameter> all_new_parameters{
        Parameter("my_parameter",
                  "world")};  ///< not sure of this line's functionality - maybe
                              ///< storing all parameters listed in a vector
    this->set_parameters(
        all_new_parameters);  ///< not sure of this line's functionality - maybe
                              ///< setup the parameter to show in param server
                              ///< or list after it was declared earlier
  }
};

/**
 * @brief Main function which initialises ROS communication and service
 * @param argc stores the no of arguements to send & receive in command line
 * @param argv stores the actual arguement data to send and receive in command
 * line
 * @return zero
 */
int main(int argc, char **argv) {
  // TestParams obj;
  init(argc, argv);  // initialise ROS communication framework
                     // argc & argv are the 2 main arguments which will store
                     // the no of request and response variables. Here we have 1
                     // input and 1 output Hence, argc is 2 and argv will have
                     // the input and output data

  shared_ptr<Node> node = Node::make_shared(
      "Modify_String_Client");  ///< creating shared pointer
                                ///< called node with the client name

  Client<ModifyString>::SharedPtr client = node->create_client<ModifyString>(
      "Modify_String_Server");  ///< Using the node, declaring the client
                                ///< Server name given in () & srv name in <>

  auto request =
      make_shared<ModifyString::Request>();  ///< storing the ModifyString srv's
                                             ///< object in request

  for (int i = 1; i <= (argc - 1);
       i++) {  ///< Using for loop and storing command
               ///< line data into request variable
    request->to_modify += argv[i];
  }

  while (!client->wait_for_service(
      500ms)) {   ///< Checking for service or server availability every 500ms
    if (!ok()) {  ///< if ctrl+c is pressed in runtime, ok() returns 0
      RCLCPP_FATAL(get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_WARN(get_logger("rclcpp"),
                "service not available, waiting again...");
  }

  auto result = client->async_send_request(
      request);  ///< send the request or input string to server and it returns
                 ///< the modified string or response

  spin_some(
      std::make_shared<MinimalParam>());  ///< spin the class "MinimalParam"
                                          ///< only once and then stop

  if (spin_until_future_complete(node, result) ==
      FutureReturnCode::SUCCESS) {  ///< spin the node until process is
                                    ///< completed and returns SUCCESS
    RCLCPP_INFO(get_logger("rclcpp"), "Modified_String: %s",
                result.get()->mod_string.c_str());  ///< display the modified
                                                    ///< string stored in result
  } else {
    RCLCPP_ERROR(get_logger("rclcpp"),
                 "Failed to call service Modify_String_Server");
    RCLCPP_DEBUG(get_logger("rclcpp"),
                 "Check server node if it's running on new terminal & then "
                 "rerun the client");
  }

  shutdown();  ///< shutdown or stop the node which was init
  return 0;
}
