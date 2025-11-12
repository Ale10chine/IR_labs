// Packages dependencies
#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/add_two_ints.hpp" // for the .srv file to define our services communication

#include <memory>

// Function to return the sum giving the request messages where the folder a,b are setted in the .srv file in interfaces package
void add(const std::shared_ptr<interfaces::srv::AddTwoInts::Request> request,
         std::shared_ptr<interfaces::srv::AddTwoInts::Response> response)
{
    response->sum = request->a + request->b;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld", request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}


int main(int argc, char **argv)
{
    // Intialize ros2 c++ client library
    rclcpp::init(argc, argv);

    // creates a pointer to a node named add_two_ints_server
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

    // Creates a service named add_two_ints for that node and automatically advertises it over the networks with the &add method
    rclcpp::Service<interfaces::srv::AddTwoInts>::SharedPtr service =
        node->create_service<interfaces::srv::AddTwoInts>("add_two_ints", &add);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

    // Spin the node making the service available
    rclcpp::spin(node);
    
    rclcpp::shutdown();
}