#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/apples.hpp" // Service

#include <chrono>
#include <cstdlib>
#include <memory>

#include <ctime> // To seed the generator using time(0)
#include <random> // To manage the random numbers

using namespace std::chrono_literals;

int rnd_num(int min, int max);

class BurrowClient : public rclcpp::Node
{
public:
    // --- CONSTRUCTOR ---
    BurrowClient()
        : Node("burrow")
    {
        // CLIENT
        client_ = this->create_client<interfaces::srv::Apples>("apples");
        RCLCPP_INFO(this->get_logger(), "The client burrow has beeen initialized\n");

        this->apples_ = rnd_num(0,5);
        this->size_ = rnd_num(this->apples_, 20);
    }

    // request function which returns the response in main
    rclcpp::Client<interfaces::srv::Apples>::SharedFuture send_request()
    {
        // Creation of the request 
        auto request = std::make_shared<interfaces::srv::Apples::Request>();
        request->curr_apples = this->apples_;
        request->burrow_size = this->size_;

        // Giving 1s time to the client to search for a server node
        // while not finding an active one wait again
        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return rclcpp::Client<interfaces::srv::Apples>::SharedFuture(); 
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        // Asynchronous sending of the request
        // Using %d format specifier for int members
        RCLCPP_INFO(this->get_logger(), 
            "Sending request with the status of the burrow: current apples stored %d + maximal size of the burrow %d", 
            this->apples_, this->size_);

        // Send the request and return the result
        auto result = client_->async_send_request(request);
        // use this sintax otherwise a warning will bre produced for a silent cast
        return result.future.share();
    }

private:
    // Moved private members here to fix scope errors
    rclcpp::Client<interfaces::srv::Apples>::SharedPtr client_;  // pointer of a client which use Apples protocol of communication
    int apples_; // num of apples already present in the burrow
    int size_; // Max size/ capacity of the burrow with the property that apples < size always

}; // end of BurrowClient class


// ----- HELPER FUNCTIONS ------

int rnd_num(int min, int max) {
    // generator of casual number (Mersenne Twister Engine) .
    // seed based on time to guarantee different results across different runs .
    static std::mt19937 generator(std::time(0));

    // distribution definition
    std::uniform_int_distribution<int> distribution(min, max);

    return distribution(generator);
}


// Main function
int main(int argc, char **argv)
{
    // Intialize ros2 cpp library
    rclcpp::init(argc, argv);

    // Create an instance of the burrow node
    std::shared_ptr<BurrowClient> client_node = std::make_shared<BurrowClient>();
        
    // Call of the function to make the request
    rclcpp::Client<interfaces::srv::Apples>::SharedFuture result = client_node->send_request();

    // Check if the returned future is valid before spinning
    if (!result.valid()) {
        rclcpp::shutdown();
        return 0; // Exits cleanly if the service waiting was interrupted
    }

    // Wait for the result, the node will spin until it will recive the response
    if (rclcpp::spin_until_future_complete(client_node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
             "Response: %s", 
             result.get()->response ? "true" : "false");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service apples");
    }

    rclcpp::shutdown();
    return 0;
}