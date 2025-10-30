#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <random> // To manage the random numbers
#include <iostream>
#include "interfaces/action/charge.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace custom_action_cpp
{
class CsActionClient : public rclcpp::Node
{
public:
  using Charge = interfaces::action::Charge;
  using GoalHandleCharging = rclcpp_action::ClientGoalHandle<Charge>;

  // ----- CONSTRUCTOR ------
  explicit CsActionClient(const rclcpp::NodeOptions & options)
  : Node("cs_action_client", options)
  {
    // Constructor intialize an ActionClient which requires 3 things
    this->client_ptr_ = rclcpp_action::create_client<Charge>( // Templated action type name
      this, // A ROS 2 node to add the action client to
      "charge"); // The action name

    // Callback Lambda function that execute the send_goal()
    auto timer_callback_lambda = [this](){ return this->send_goal(); };

    // Timer to call the send_goal function, implemented with lambda function 2Hz
    // NOTE : in the implementation below we will see that we call the function just 1 time in reality, so the freq is not so much improtant
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      timer_callback_lambda);

    // Initialize the power of the ChargingStation in a randomic interval
    power_ = rnd_num(80,100);
    RCLCPP_INFO(this->get_logger(), "Power: %d", power_);
  }


  // ----- MEMBER FUNCTIONS -----
  // Note: Keep in mind that the client works asyncrhonously w.r.t the server that is, after done its processing
  // it remains in listenting of the Server to attend a response
  void send_goal()
  {
    using namespace std::placeholders;

    // We want to send just one goal in this implementation aka we require just 1 request of charge the robot 
    // with a certain disponibility by this client (the charging station)
    this->timer_->cancel();

    // Wait to switch online the Robot Server otherwise after a time limit switch off the node
    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    // Goal service request
    auto goal_msg = Charge::Goal();
    goal_msg.power = power_ ; // Cs has assume a random power included inside the interval of 80 - 100% of battery

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    // Object to embed 3 callbacks: 
    auto send_goal_options = rclcpp_action::Client<Charge>::SendGoalOptions();

    // Goal response callback (Wait the acceptance responce of the Goal by the Server)
    send_goal_options.goal_response_callback = [this](const GoalHandleCharging::SharedPtr & goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    };

    // Feedback callback (wait the feedback from Server)
    send_goal_options.feedback_callback = [this](
      GoalHandleCharging::SharedPtr,
      const std::shared_ptr<const Charge::Feedback> feedback)
    {
      std::stringstream ss;
      ss << "Current level of battery of robot during charging: " << feedback->current_level;
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    };

    // Result callbacks (wait for the Result response by the Server)
    send_goal_options.result_callback = [this](const GoalHandleCharging::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          return;
      }
      std::stringstream ss;
      ss << "Result received: "<< result.result->new_battery_level;
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
      rclcpp::shutdown();
    };

    // Final send of the goal
    // the logic is the following : before we have setted the callbacks needed to the node to understand
    // how to react to te response of the server, and than we effective send the goal. This mechanism is 
    // possible thanks to the async_send_goal() function
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
// ----- DATA MEMBERS -----
  rclcpp_action::Client<Charge>::SharedPtr client_ptr_;
  int power_; // current maximal power of the Cs that can give to the robots
  rclcpp::TimerBase::SharedPtr timer_;

// ----- MEMBER FUNCTIONS -----
// function to random initialize an int variable
  static int rnd_num(int min, int max){
    // generator of casual number (Mersenne Twister Engine) .
    // seed based on time to guarantee different results across different runs .
    static std::mt19937 generator(std::time(0));
    // distribution definition
    std::uniform_int_distribution<int> distribution(min, max);
    return distribution(generator);
  }
};  // class CsActionClient


// ----- HELPER FUNCTIONS ------



} // namespace custom_action_cpp

// This macro is used when we deal with not simple nodes but for example with actions
// and allow this client to work 
RCLCPP_COMPONENTS_REGISTER_NODE(custom_action_cpp::CsActionClient)