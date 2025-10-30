#include <memory>
#include "rclcpp/rclcpp.hpp" // Most common pieces of the ROS 2 system
//#include "std_msgs/msg/string.hpp" // Includes the built-in message type you will use to publish data.

#include "interfaces/msg/cstm_inforobot.hpp"

using std::placeholders::_1;


class charge_station : public rclcpp::Node
{
public:
  // ------- CONSTRUCTOR -------
  charge_station()
  : Node("charge_station") // There is no timer because the subscriber simply responds whenever data is published to the topic 
  {
    // Subscription is always listening in the topic specified
    charge_status_subscription_ = this->create_subscription<interfaces::msg::CstmInforobot>(
      "charge_status", 10, std::bind(&charge_station::read_robot_status_callback, this, _1));
  }

private:
  // ------ MEMBER FUNCTION ------
 
  // Subscribers callbacks
  // Callback that activates when the robot send a status update
  void read_robot_status_callback(const interfaces::msg::CstmInforobot::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Recived: Room_id = '%d' , Room_name = '%s', Battery_level = '%f' ",
     msg->room_id,
     msg->room_name.c_str(),
     msg->battery_level );
  }

  // ------- DATA MEMBER -------
  rclcpp::Subscription<interfaces::msg::CstmInforobot>::SharedPtr charge_status_subscription_;
};









// Main function where the node actually execute 
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // To start processing data from the node "MinimalSubscriber" , so we start listening
  rclcpp::spin(std::make_shared<charge_station>());
  
  rclcpp::shutdown();
  return 0;
}

