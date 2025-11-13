#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"


#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "nav_msgs/msg/path.hpp"
#include <fstream> 


using namespace std::chrono_literals;

class PathTrackerListener : public rclcpp::Node
{
public:

// ------- CONSTRUCTOR -------
  PathTrackerListener()
  : Node("path_tracker_listener")
  {
    // Declare and acquire `target_frame` parameter, but in this case we don't need it
    // We use directly the frame tag
    this->declare_parameter<std::string>("floor_frame", "tag36h11:0");
    this->declare_parameter<std::string>("vc_frame", "tag36h11:2");
    this->declare_parameter<std::string>("cs_frame", "tag36h11:1");

    // Is a memory local buffer to store data which are flowing through the bag
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    // Setup of the listener which is listening on tf and tf_static and is feeding the buffer
    // In practice is an interface with the 2 topic /tf /tf_static, which silent subscribe to them
    // in a higher level way not as usual like when we create a subscriptio to a topic, and then store all data in the buffer
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    // Timer to callback the on_timer callback which listen to the bag
    timer_ = this->create_wall_timer(
      100ms, [this]() {this->on_timer();}); 

  }

  // function called on the main to save the csv log file
  void save_csv(std::string csv_file, std::vector<std::string> csv_data_) {

     // std::string csv_file = "vc_path_projected.csv";
      std::ofstream outfile(csv_file);

      if (!outfile.is_open()) {
          RCLCPP_ERROR(this->get_logger(), "Could not open CSV file for writing!");
          return;
      }

      // Writen the intestation
      outfile << "timestamp,x,y,cs_x,cs_y\n"; 
      // Wirte data
      for (const auto& line : csv_data_) {
          outfile << line << "\n";
      }

      outfile.close();
      RCLCPP_INFO(this->get_logger(), "Path saved to %s", csv_file.c_str());
  }

  // --- PUBLIC DATA MEMBER ---
  // Vector of string to save the row of the csv
  std::vector<std::string> csv_data_1_;
  std::vector<std::string> csv_data_2_;
  

private:

// Bag msg published on /tf represent the rotations matrix to rotate frames of CS and VC w.r.t Camera frame
// We want to express both rotation w.r.t to Floor frame
void on_timer()
  {

    // Frames setup
    // String wich contains the name of the r.f
    std::string floor_frame = this->get_parameter("floor_frame").as_string(); // tag36h11:0 VC frame
    std::string cs_frame = this->get_parameter("cs_frame").as_string();       // tag36h11:1 CS frame
    std::string vc_frame = this->get_parameter("vc_frame").as_string();       // tag36h11:2 Floor frame

    // --- Path vc and cs w.r.t floor_frame with view in the xy plane ---
    
    // geometry messages which contains the frame vc and cs both expressed w.r.t floor frame after the operation 
    // we will do after
    geometry_msgs::msg::TransformStamped t_vc_to_floor; // vc_frame written w.r.t floor_frame
    geometry_msgs::msg::TransformStamped t_cs_to_floor; // cs_frame written w.r.t floor_frame 

    // VC w.r.t Floor frame (vc frame to floor frame)    
    // Try to lookupTransform vc_frame to floor_frame (to_frame - from_frame, are the arguments of the lookupTrasnform in this order)
    try {
      t_vc_to_floor = tf_buffer_->lookupTransform(
        floor_frame, vc_frame,
        tf2::TimePointZero); 
    } catch (const tf2::TransformException & ex) {
      // If the transform is not feasbile log and continue the cycle
      RCLCPP_DEBUG(
        this->get_logger(), "Could not transform %s to %s: %s",
        floor_frame.c_str(), vc_frame.c_str(), ex.what());
      return;
    }

    // CS w.r.t Floor frame (cs frame to floor frame)    
    // Try to lookupTransform cs_frame to floor_frame
    try {
      t_cs_to_floor = tf_buffer_->lookupTransform(
        floor_frame, cs_frame,
        tf2::TimePointZero); 
    } catch (const tf2::TransformException & ex) {
        // If the transform is not feasbile continue the cycle
        return; 
    }

    // Calculation of the time stamp
    double timestamp = t_vc_to_floor.header.stamp.sec +
                       t_vc_to_floor.header.stamp.nanosec * 1e-9;


   csv_data_1_.push_back(
        std::to_string(timestamp) + "," + 
        std::to_string(t_vc_to_floor.transform.translation.x) + "," + 
        std::to_string(t_vc_to_floor.transform.translation.y) + "," + 
        std::to_string(t_cs_to_floor.transform.translation.x) + "," + 
        std::to_string(t_cs_to_floor.transform.translation.y)
    );

    // --- Path vc w.r.t cs_frame with view in the xy plane ---

    // We can arrise directly this relative path of vc w.r.t cs using the previous r.f of cs and vc wich refers to the floor_frame
    double vc_x_floor = t_vc_to_floor.transform.translation.x;
    double vc_y_floor = t_vc_to_floor.transform.translation.y;

    double cs_x_floor = t_cs_to_floor.transform.translation.x;
    double cs_y_floor = t_cs_to_floor.transform.translation.y;

    double vc_x_wrt_cs = vc_x_floor - cs_x_floor;
    double vc_y_wrt_cs = vc_y_floor - cs_y_floor;


    timestamp = t_vc_to_floor.header.stamp.sec +
                        t_vc_to_floor.header.stamp.nanosec * 1e-9;

     csv_data_2_.push_back(
         std::to_string(timestamp) + "," +
         std::to_string(vc_x_wrt_cs) + "," +
         std::to_string(vc_y_wrt_cs));
  }



  // --- DATA MEMBER ---
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


  
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathTrackerListener>();

    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_INFO(node->get_logger(), "Node interrupted by exception: %s", e.what());
    }

    RCLCPP_INFO(node->get_logger(), "Saving data to CSV...");
    node->save_csv("vc_to_floor_path_projected.csv", node->csv_data_1_);
    node->save_csv("vc_to_cs_path_projected.csv", node->csv_data_2_);

    rclcpp::shutdown();
    return 0;
}






  
  