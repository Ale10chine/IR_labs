
#include <chrono>
#include <memory>
#include <memory>
#include <string>

#include <ctime> // To seed the generator using time(0)
#include <random> // To manage the random numbers
#include <chrono> // To manage the time of cleaning 


#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"


// ROS2 usually converts the name of the message in lower case and add the underscore
// (snake_case) for the C++ convention, indipendently of the CamelCase (CstmInforobot) that you
// have used in the .msg file
#include "interfaces/msg/cstm_inforobot.hpp"
using namespace std::chrono_literals;

// Random generator function
int rnd_num(int min, int max);

class robot : public rclcpp::Node
{
public:
  // ------- CONSTRUCTOR -------
  robot()
  : Node("robot")
  {
    // Publishers
    robot_charge_publisher_ = this->create_publisher<interfaces::msg::CstmInforobot>("charge_status", 10);

    // Frequencies settings : initialize the timer to stabilize the frequenzy on which will work the callback function that publish on the charge_status topic
    timer_ = this->create_wall_timer(200ms, std::bind(&robot::update_robot_status_callback , this));
    action_timer_ = this->create_wall_timer(10000ms, std::bind(&robot::robot_action , this));
    battery_timer_ = this->create_wall_timer(100ms, std::bind(&robot::battery_consumption_sim , this));
  }

private:
  // ------ MEMBER FUCNTIONS ------

  // Publisher callbacks
  //
  void update_robot_status_callback()
  {

    // Define a message of the type of the custum one used for the robot
    auto message = interfaces::msg::CstmInforobot();

    message.room_id = room_id_;
    message.room_name = room_name_;
    message.battery_level = battery_level_;

    //RCLCPP_INFO(this->get_logger(), "Publishing robot status: Room_id ='%d', Room_name ='%s', Battery_level = '%f'",
    //            message.room_id,
    //            message.room_name.c_str(),
    //            message.battery_level);

    robot_charge_publisher_->publish(message);
  }


  void robot_action(){
    
    // Robot actions
    if (battery_level_ < 5.0){
      printf(" The battery is too low, switch off the robot ...");
      // Robot is switched off then cannot loose battery or make any action
      //action_timer_->cancel(); 
      //battery_timer_->cancel();
      rclcpp::shutdown(); // the robot node directly shutdown
      return;
    }
    
    int random_act = rnd_num(1,5);

    room_id_ = random_act;
    room_name_ = room_names_[room_id_];
  }

  void battery_consumption_sim(){
    // Only a safty check
    if (battery_level_ < 0.05){
      battery_timer_->cancel(); 
      return;
    }
    battery_level_ = battery_level_ - 0.05;
  }


  // ------ DATA MEMBERS -------
  // Declaration of the timer to manage the frequency of the calls
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr action_timer_;
  rclcpp::TimerBase::SharedPtr battery_timer_;

  // Declaration of the base publisher that will send a message of type string using a shared ptr
  rclcpp::Publisher<interfaces::msg::CstmInforobot>::SharedPtr robot_charge_publisher_; 
  
  // Data member of the robot
  int room_id_;
  std::string room_name_;
  double battery_level_ = 100.0;

  const std::vector<std::string> room_names_ = {
      "Unknown",
      "Robot Vision Lab",      // ID 1
      "SSL Lab",               // ID 2
      "Neurorobotics Lab",     // ID 3
      "IAS_Lab",               // ID 4
      "Autonomus Robotics Lab" // ID 5
  };

};


// ----- HELPER FUNCTIONS ------


int rnd_num(int min, int max) {
    // generator of casual number (Mersenne Twister Engine) .
    // seed based on time to guarantee different results across different runs .
    static std::mt19937 generator(std::time(0));

    // distribution definition
    std::uniform_int_distribution<int> distribution(min, max);

    return distribution(generator);
}





// Main function where the node actually executes
int main(int argc, char * argv[])
{
  // To initialize ROS 2
  rclcpp::init(argc, argv); 

  printf("Robot is now switched on...\n");
  // To start processing data from the node "MinimalPublisher" including callbacks from the timer
  rclcpp::spin(std::make_shared<robot>()); 
  
  rclcpp::shutdown();
  return 0;
}

