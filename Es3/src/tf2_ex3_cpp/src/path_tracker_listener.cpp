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


    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    // Setup of the listener which is listening on tf and tf_static and is feeding the buffer
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    // Creation of the publisher for the path wrt floor and wrt cs
    path_pub_floor_ = this->create_publisher<nav_msgs::msg::Path>("vc_path_floor", 10);
    path_pub_cs_ = this->create_publisher<nav_msgs::msg::Path>("vc_path_cs", 10);

    // Inizializza i messaggi Path (necessario per definire l'header)
    path_msg_floor_.header.frame_id = this->get_parameter("floor_frame").as_string();
    path_msg_cs_.header.frame_id = this->get_parameter("cs_frame").as_string();

    // Timer to callback the on_timer callback which listen to the bag
    timer_ = this->create_wall_timer(
      50ms, [this]() {this->on_timer();}); // Cambiamo a 50ms (20Hz)

  }

  // function called on the main to save the csv log file
  void save_csv() {
      // Funzione per salvare i dati raccolti nel file CSV
      std::string csv_file = "vc_path_projected.csv";
      std::ofstream outfile(csv_file);

      if (!outfile.is_open()) {
          RCLCPP_ERROR(this->get_logger(), "Could not open CSV file for writing!");
          return;
      }

      // Scrivi l'intestazione
      outfile << "timestamp,x,y,cs_x,cs_y\n"; 
      
      // Scrivi i dati
      for (const auto& line : csv_data_) {
          outfile << line << "\n";
      }

      outfile.close();
      RCLCPP_INFO(this->get_logger(), "Path saved to %s", csv_file.c_str());
  }

private:

// Bag msg published on /tf represent the rotations matrix to rotate frames of CS and VC w.r.t Camera frame
// We want to express both rotation w.r.t to Floor frame
void on_timer()
  {
    // Frames setup
    std::string to_floor_frame = this->get_parameter("floor_frame").as_string(); // tag36h11:0 VC frame
    std::string from_cs_frame = this->get_parameter("cs_frame").as_string(); // // tag36h11:1 CS frame
    std::string from_vc_frame = this->get_parameter("vc_frame").as_string();  // tag36h11:2 Floor frame

    geometry_msgs::msg::TransformStamped t_vc_to_floor; // vc_frame -> floor_frame
    geometry_msgs::msg::TransformStamped t_cs_to_floor; // cs_frame -> floor_frame 

    // VC wrt Floor frame (vc frame to floor frame)    
    // Try to lookupTransform vc_frame to floor_frame
    try {
      t_vc_to_floor = tf_buffer_->lookupTransform(
        to_floor_frame, from_vc_frame,
        tf2::TimePointZero); // L'ultima trasformazione
    } catch (const tf2::TransformException & ex) {
      // If the transform is not feasbile log and continue the cycle
      RCLCPP_DEBUG(
        this->get_logger(), "Could not transform %s to %s: %s",
        to_floor_frame.c_str(), from_vc_frame.c_str(), ex.what());
      return;
    }

    // CS wrt Floor frame (cs frame to floor frame)    
    // Try to lookupTransform cs_frame to floor_frame
    try {
      t_cs_to_floor = tf_buffer_->lookupTransform(
        to_floor_frame, from_cs_frame,
        tf2::TimePointZero); 
    } catch (const tf2::TransformException & ex) {
        // If the transform is not feasbile continue the cycle
        return; 
    }

    // Calculation of the position of vc w.r.t floor frame using the floor projection

    geometry_msgs::msg::PoseStamped vc_pose_floor;
    vc_pose_floor.header.stamp = this->get_clock()->now();
    vc_pose_floor.header.frame_id = to_floor_frame;
    
    // Change translation with the ones already found
    vc_pose_floor.pose.position.x = t_vc_to_floor.transform.translation.x;
    vc_pose_floor.pose.position.y = t_vc_to_floor.transform.translation.y;
    // set z == 0 to project
    vc_pose_floor.pose.position.z = 0.0; 
    // Keep the rotation
    vc_pose_floor.pose.orientation = t_vc_to_floor.transform.rotation;
    
    // Add path of the vc w.r.t floor_frame and publish it on \path_pub_floor_ topic
    path_msg_floor_.poses.push_back(vc_pose_floor);
    path_pub_floor_->publish(path_msg_floor_);

    // === 3. Salva i dati VC e CS per il CSV (VC rispetto al Pavimento) ===

    // Costruisci la riga CSV (timestamp, VC_x, VC_y, CS_x, CS_y)
    double timestamp = vc_pose_floor.header.stamp.sec + 
                       vc_pose_floor.header.stamp.nanosec * 1e-9;
    
    csv_data_.push_back(
        std::to_string(timestamp) + "," + 
        std::to_string(vc_pose_floor.pose.position.x) + "," + 
        std::to_string(vc_pose_floor.pose.position.y) + "," + 
        std::to_string(t_cs_to_floor.transform.translation.x) + "," + 
        std::to_string(t_cs_to_floor.transform.translation.y)
    );


    // === 4. Calcolo Posizione VC rispetto alla CS ===
    // Target Frame è la CS, Source Frame è il VC

    // Frames setup
    std::string to_cs_frame = this->get_parameter("cs_frame").as_string(); // // tag36h11:1 CS frame 
     // tag36h11:0 VC frame already defined above
     
                                                                          

    geometry_msgs::msg::TransformStamped t_vc_to_cs; // vc_frame -> cs_frame

    // VC wrt CS frame (vc frame to cs frame)    
    // Try to lookupTransform vc_frame to cs_frame
    try {
        t_vc_to_cs = tf_buffer_->lookupTransform(to_cs_frame, from_vc_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_DEBUG(this->get_logger(), "Could not transform VC to CS: %s", ex.what());
        return;
    }

    // Calculation of the position of vc w.r.t cs frame using the floor projection
    geometry_msgs::msg::PoseStamped vc_pose_cs;
    vc_pose_cs.header.stamp = this->get_clock()->now();
    vc_pose_cs.header.frame_id = to_cs_frame;
    
    // 
    vc_pose_cs.pose.position.x = t_vc_to_cs.transform.translation.x;
    vc_pose_cs.pose.position.y = t_vc_to_cs.transform.translation.y;
    vc_pose_cs.pose.position.z = 0.0; 
    vc_pose_cs.pose.orientation = t_vc_to_cs.transform.rotation;
    
    // Add path of the vc w.r.t cs_frame and publish it on \path_pub_cs_ topic
    path_msg_cs_.poses.push_back(vc_pose_cs);
    path_pub_cs_->publish(path_msg_cs_);

  } 

  


  // --- DATA MEMBER ---
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  
  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_floor_{nullptr};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_cs_{nullptr};

  // Msgs to accumulate points
  nav_msgs::msg::Path path_msg_floor_;
  nav_msgs::msg::Path path_msg_cs_;

  // Vector of string to save the row of the csv
  std::vector<std::string> csv_data_;
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
    node->save_csv();

    rclcpp::shutdown();
    return 0;
}
