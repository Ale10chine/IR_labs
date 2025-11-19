#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"




class ManageLifecycleNodesClient : public rclcpp::Node
{
public:
    ManageLifecycleNodesClient() : Node("lifecycle_nodes_client")
    {
        navigation_client_ = create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_navigation/manage_nodes");
        
        localization_client_ = create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_localization/manage_nodes");
        // 
    
    }

    void send_request()
    {

        
        auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
        // Set request->command as needed
        request->command = 0; // startup nav2 
        
        RCLCPP_INFO(get_logger(), "Wait for service of navigation");
        navigation_client_->wait_for_service();
        
        auto result_future = navigation_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Service call of navigation completed");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Service call of navigation failed");
        }


        RCLCPP_INFO(get_logger(), "Wait for service of localization");
        localization_client_->wait_for_service();
        result_future = localization_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Service call of localization completed");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Service call of localization failed");
        }
    }

   
private:
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr navigation_client_;
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr localization_client_;
    

    
};


// Main function
int main(int argc, char **argv)
{
    // Intialize ros2 cpp library
    rclcpp::init(argc, argv);

    // Create an instance of the burrow node
    std::shared_ptr<ManageLifecycleNodesClient> client_node = std::make_shared<ManageLifecycleNodesClient>();
        
    client_node->send_request();
    

    rclcpp::shutdown();
    return 0;
}