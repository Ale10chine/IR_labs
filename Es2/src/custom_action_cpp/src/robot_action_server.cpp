#include <functional>
#include <memory>
#include <thread>
#include <chrono>
#include <random> // To manage the random numbers
#include <iostream>

#include "interfaces/action/charge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp" // We are dealin gwith components and not only simple nodes!

#include "custom_action_cpp/visibility_control.h"

using namespace std::chrono_literals;

// To incapsulate the entire code written insdie this scope in case if we need to recall it somewhere else
// in future, but i think that for ros logic this is not so useful since nodes are classes that create object
// that works directly in the .cpp in which they are written
namespace custom_action_cpp
{
class RobotActionServer : public rclcpp::Node
{
public:
  // Alias fot type to simplify the legibility of the code, c++ could be very verbose with so many template
  using Charge = interfaces::action::Charge;
  using GoalHandleCharging = rclcpp_action::ServerGoalHandle<Charge>;


  // As you can see even the initialization of the constructor of the node in more complex, since this is a more complex node when we deal
  // with actions we talk about "Component" and not anymore a "simple node".
  // We use this macro to export the constructor when the node is compiled as shared library ==>  ROS can dinamically charge it (guarantees efficency,
  // often use in practice for complex nodes)
  CUSTOM_ACTION_CPP_PUBLIC
  // ----- CONSTRUCTOR -----
  explicit RobotActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("robot_action_server", options)
  {
    using namespace std::placeholders;
    
    // Callback with lambda function for handling goals, it simply accept all goals
    auto handle_goal = [this]( // args of the lambda, // unique identifier of the goal, // pointer to .action message
      const rclcpp_action::GoalUUID & uuid, 
      std::shared_ptr<const Charge::Goal> goal) 
    { 
      RCLCPP_INFO(this->get_logger(), "Received goal request with power %d", goal->power);
      (void)uuid;// typical trick to suppress warining fo an unused variable

      // The logic here is that if the cs can charge at least 50% and the robot has "low battery" then 
      // the task of charging it makes sense to be done otherwise not, the robot can still have good autonomy or
      // the cs can offer to poor charge
      if (battery_level_ <= 50 && goal->power >= 50){
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // accept the goal and call the second callback handle_accepted()
      }else
        return rclcpp_action::GoalResponse::REJECT; // otherwise reject
      
    };

    // Callback with lambda function for dealing with cancellation, this implementation just tells the client that it accepted the cancellation
    auto handle_cancel = [this](
      
      const std::shared_ptr<GoalHandleCharging> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    };

    // Callback with lambda function for accepting a new goal and starts processing it
    auto handle_accepted = [this](
      const std::shared_ptr<GoalHandleCharging> goal_handle)
    {
      // this needs to return quickly to avoid blocking the executor,
      // so we declare a lambda function to be called inside a new thread
      auto execute_in_thread = [this, goal_handle](){return this->execute(goal_handle);}; // Launch of the execute() function
      std::thread{execute_in_thread}.detach();
    };

    // Constructor instantiate an ActionServer, in this object Server is implemeted the mechanism of binding,
    // so when a message is sent the various callbacks are called
    // In general an action server requires 6 things: 
    this->action_server_ = rclcpp_action::create_server<Charge>( // templated action type name
      this, // A ROS 2 node to add the action to
      "charge", // The action name
      handle_goal, // A callback function for handling goals
      handle_cancel,// A callback function for handling cancellation
      handle_accepted); // A callback function for handling goal accept

    // Initialize a random level of battery for the robot
    battery_level_ = rnd_num(5, 30);
    RCLCPP_INFO(this->get_logger(), "Battery Level: %d", battery_level_);

  }

private:
// ----- DATA MEMEBERS -----
  rclcpp_action::Server<Charge>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_feedback_; // timer to manage separately the usage of the feedback  
  int battery_level_; // battery level of the robot


// ----- MEMBER FUNCTIONS -----

  // Function to random initialize an int variable
  static int rnd_num(int min, int max){
    // generator of casual number (Mersenne Twister Engine) .
    // seed based on time to guarantee different results across different runs .
    static std::mt19937 generator(std::time(0));
    // distribution definition
    std::uniform_int_distribution<int> distribution(min, max);
    return distribution(generator);
  }

/* 
 Remmeber how is build the .action file: 
 Goal - Request - Feedback
*/

  // Function that process the goal (the power of the Cs) required by the Client and response to it
  // giving the final charge level of the robot of 60 sec of charging simulation
  void execute(const std::shared_ptr<GoalHandleCharging> goal_handle) {
    
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    // Recover goal send by the Client
    auto goal = goal_handle->get_goal(); 
    int charge_target;
    if (goal->power + battery_level_ > 100){
      // Es : Suppose that robot has b = 40 and cs = 70
      // ... = 70 - (70 + 40 - 100) = 60, so robot can be charged up until 100 witouth exeecding this threshold
      charge_target = goal->power - (goal->power + battery_level_ - 100);
    }
    else{
      charge_target = goal->power;
    }
    RCLCPP_INFO(this->get_logger(), "Charge target : %d", charge_target);

    // Inizialization of the Feedback and Result as shared pointer
    auto feedback = std::make_shared<Charge::Feedback>(); // Feedback
    auto & curr_b = feedback->current_level; // Creare an Alias 
    curr_b = battery_level_;
    
    // Feedback to manage separately the frequenzy of update of the charging with the effective sending feedback message to the client
    // basically it create a timer that every 1 sec gives back the feedback to the client
    timer_feedback_ = this->create_wall_timer(1000ms,
                                              [this, goal_handle, feedback]()
                                              {
                                                goal_handle->publish_feedback(feedback); 
                                                RCLCPP_INFO(this->get_logger(), "Publish feedback");
                                              });

    auto result = std::make_shared<Charge::Result>(); // Result

    // frequenzy change based on the total amount of energy to transfer
    // we must respect the logic that in any case the time spended to do this is 60 sec
    float freq = charge_target / 60.0;
    rclcpp::Rate loop_rate(freq);

    // Loop that iterates with Hz frequency given by "freq"
    for (int i = 1; (i <= charge_target) && rclcpp::ok(); ++i)
    {
      // Check if there is a cancel request
      // verify an internal status of the Action server, so if simultaneosly the Client has sent
      // a notification of canceling
      if (goal_handle->is_canceling())
      {
        result->new_battery_level = curr_b;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update current level battery of the robot
      curr_b = curr_b + 1; // Update 1% each iteration

      loop_rate.sleep(); // sleep in order to mantain 60 sec of charging in total
    }

    // Check if goal is done
    if (rclcpp::ok()) { // Check if the node is still operative

      timer_feedback_.reset();
      // notice this line below, at left sequence is the field of result described in .action, at right sequence 
      // is the alias to the field partial_sequence of feedback. Indeed at the end the partial sequence correspond with the complete one
      result->new_battery_level = curr_b; 
      goal_handle->succeed(result); // Set goal SUCCEEDED and sent result to Client
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  };
};  // class RobotActionServer

// ----- HELPER FUNCTIONS ------


}  // namespace custom_action_cpp

// This macro is used when we deal with not simple nodes but for example with actions
// and allow this client to work 
RCLCPP_COMPONENTS_REGISTER_NODE(custom_action_cpp::RobotActionServer)

