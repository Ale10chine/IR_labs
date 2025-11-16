#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/apples.hpp"  //Service protocol communication
#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp" // Messages written in the topic /apples
#include "geometry_msgs/msg/pose_stamped.hpp" // Pose_array contains Pose_stamped messages
#include "sensor_msgs/msg/laser_scan.hpp" // Messages read from the /scan topic of the lidar

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" //Necessary definitions to make tf2 able to manage PoseStamped msgs

using namespace std::chrono_literals; // for using time expressed in ms



class RobotServer : public rclcpp::Node
{
public:
    // --- CONSTRUCTOR ---
    RobotServer()
    : Node("robot_server")
    {
        
        // The service server listens for requests on the topic "apples"
        // and uses the member function 'handle_client_request' to respond.
        // NOTE: request will be a data full of content, sended by the client
        // while response is empty, is our job to feed it

        // SERVICE
        service_= this->create_service<interfaces::srv::Apples>(
            "apples",
            [this](const interfaces::srv::Apples::Request::SharedPtr request,
                   interfaces::srv::Apples::Response::SharedPtr response) 
            {   
                // the server works as to be in listening for a client request, so
                // this calbback is done asynchronously
                // The body calls the actual member function
                this->handle_client_request(request, response);
            }
        );

        // PUBLISHERS
        apples_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/apples", 10);
        //  SUBSCRIBERS
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            10,     
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
            {
                // activate the callback of lidar msgs each time arrive a new scan
                this->process_lidar_data_callback(msg);
            });

        // TF BUFFER and LISTENER to process transformation between different r.frames

        // Is a memory local buffer to store data which are flowing through the bag
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

        // Setup of the listener which is listening on tf and tf_static and is feeding the buffer
        // In practice is an interface with the 2 topic /tf /tf_static, which silent subscribe to them
        // in a higher level way not as usual like when we create a subscriptio to a topic, and then store all data in the buffer
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

        // Log that the server is configurated correctly
        RCLCPP_INFO(this->get_logger(), "Robot Server for 'apples' service is ready.");
    }

private:

    // Function to manage Service response to the client
    void handle_client_request(
        const interfaces::srv::Apples::Request::SharedPtr request,
        interfaces::srv::Apples::Response::SharedPtr response)
    {
        // Log the received data (for debug/information)
        RCLCPP_INFO(this->get_logger(), "Received Request: %d apples in burrow of size %d",
                    request->curr_apples, request->burrow_size);

        if(this->apples_finded_ >= request->burrow_size - request->curr_apples){
            RCLCPP_INFO(this->get_logger(), "Apples finded: %d", this->apples_finded_);
            response->response = true;
            //  Log the response just sent
            RCLCPP_INFO(this->get_logger(), "Sending Response: true, total number of apples founded is %d, were required %d apples ", this->apples_finded_, request->burrow_size - request->curr_apples);
        } else {
            RCLCPP_INFO(this->get_logger(), "Apples finded: %d", this->apples_finded_);
            response->response = false;
            //  Log the response just sent
            RCLCPP_INFO(this->get_logger(), "Sending Response: false, total number of apples founded is %d, but required %d", this->apples_finded_, request->burrow_size - request->curr_apples);
        }
               
    }

    // Function to process the acquisition by the lidar, find the r.f. of the apples and posting ong /apples topic
    void process_lidar_data_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        
        // parsing of the message to ricave "ranges"
        // distances is a vector of 360 elements
        std::vector<double> distances(msg->ranges.begin(), msg->ranges.end());
      
        // ---------- ### Clustering of the points in order to find the pose of the r.f ### ----------
        double r_i;
        double r_j;
        double threshold = 0.20; // Setted based on empirical estimation w.r.t to our simulation on rviz
        std::vector<int> min_indexes;

        std::vector<std::vector<double>> clusters;
        
        for (size_t i = 0; i < distances.size(); i++)
        {
            // check for the boundary
            if (i >= distances.size() - 1) break;

            r_i = distances[i];
            r_j = distances[i + 1];

            // found the start of the cluster, otherwise keep cicling
            if (std::abs(r_i - r_j) > threshold && r_i > r_j)
            {
                //std::cout<<"Founded cluster at degree: "<<i<<"\n";
                std::vector<double> cluster; // tmp cluster to be filled in the `clusters` vector of vectors
                size_t j = i + 1;

                double current_min_value = distances[j]; 
                int min_index = j; 

                while ( j < distances.size() - 1) // until the distance is similar
                {

                    r_i = distances[j];
                    r_j = distances[j + 1];

                    if(r_i < current_min_value) {
                        min_index = j;
                        current_min_value = r_i;
                    }
                    
                    cluster.push_back(r_i);

                    // found the end of the cluster
                    if (std::abs(r_i - r_j) > threshold && r_i < r_j)
                    {

                        min_indexes.push_back(min_index);
                        clusters.push_back(cluster); // Push the cluster inside the other clusters (which is a vector)

                        i = j;
                        break; 
                    }

                    j++;
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Clusters founded: %zu", clusters.size());
        
        // *** Saving in a datamember variable the num of apples finded ***
        this -> apples_finded_ = clusters.size();


        // ---------- ### Procedure for fixing the r.f on the apples w.r.t base_scan frame and express it w.r.t base_link frame for each apple founded using tf2 ### ----------      

        // Preparation for the message to send in /apples : it's a PoseArray, so contains an array of PoseStamped msgs
        geometry_msgs::msg::PoseArray apples_pose_array;
        std::string target_frame = "base_link";
        apples_pose_array.header.frame_id = target_frame; 
        apples_pose_array.header.stamp = msg->header.stamp; // copy from here, we have the same time stamp

        // Parsing msg to ricave angle_min and angle_increment from the lidar msgs
        double angle_min = msg->angle_min;
        double angle_increment = msg->angle_increment;

        for (size_t i = 0; i < clusters.size(); i++)
        {
            // *** Fixing the r.f of the apples ***
            int k = min_indexes[i];
            double r_min = distances[k];
            double theta = angle_min + k * angle_increment;

            double x_base_scan = r_min * std::cos(theta);
            double y_base_scan = r_min * std::sin(theta);

            geometry_msgs::msg::Pose pose_base_scan;
            pose_base_scan.position.x = x_base_scan;
            pose_base_scan.position.y = y_base_scan;
            pose_base_scan.position.z = 0.0; // on xy plane of the lidar
            // the orientation with quaternion is set as the identity
            pose_base_scan.orientation.x = 0.0;
            pose_base_scan.orientation.y = 0.0;
            pose_base_scan.orientation.z = 0.0;
            pose_base_scan.orientation.w = 1.0;
            
            // PoseStamped msg : preparation of a message with the pose of the apple frame and its time stamp
            geometry_msgs::msg::PoseStamped pose_in_base_scan;
            pose_in_base_scan.header.frame_id = msg->header.frame_id; // copy from here := base_scan r.f
            pose_in_base_scan.header.stamp = msg->header.stamp; // copy from here, we have the same time stamp
            pose_in_base_scan.pose = pose_base_scan; // the pose that we ricaved above

            // *** transformation of the r.f of the apples w.r.t base_link ***
            geometry_msgs::msg::PoseStamped pose_in_base_link;
            try
            {
                pose_in_base_link = tf_buffer_->transform(
                    pose_in_base_scan, // pose w.r.t base_scan r.f
                    target_frame, // pose w.r.t base_link r.f
                    tf2::Duration(100ms));

                //  add the pose transformed in the final PoseArray msg
                apples_pose_array.poses.push_back(pose_in_base_link.pose);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
            }
        }
         
        // ---------- ### Send the messages as PoseArray in the topic apples ### ----------       
        apples_publisher_->publish(apples_pose_array);
       
    }


    // --- DATA MEMBER ---
    // Note: the 'service_' variable holds the technical ROS 2 Service endpoint object, 
    // which listens for incoming client requests on the 'apples' topic.
    // This object is distinct from the logical node (RobotServer) which acts as the Server.
    // Conversely, the 'client_' variable in the BurrowClient node holds the object used to 
    // actively send requests to a Service endpoint.
    rclcpp::Service<interfaces::srv::Apples>::SharedPtr service_; // pointer of a service which use Apples protocol of communication
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_; // /scan subscriber to read lidar msgs
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr apples_publisher_; // /apples publisher to update the pose of the apples in real time 
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    int apples_finded_ = 0; // num of apples finded in such scan


}; // end of RobotServer class



int main(int argc, char **argv)
{
    // Initialize ros2 cpp library
    rclcpp::init(argc, argv);

    // Create an instance of the server node (as a shared_ptr)
    std::shared_ptr<RobotServer> server_node = std::make_shared<RobotServer>();

    // Block the execution and spin the node, allowing it to listen for requests.
    // The node will continue to 'spin' until interrupted (e.g., Ctrl+C).
    rclcpp::spin(server_node);

    // Clean up and shut down the ROS 2 library
    rclcpp::shutdown();
    return 0;
}


// debug print, put after line 163 if is needed
//        for (size_t i = 0; i < clusters.size(); i++)
//        {
//            std::cout<<"num element of the "<<i<<" cluster: "<< clusters[i].size()<<"\n";
//            std::cout<<"min index of the cluster is "<<min_indexes[i]<<"\n\n";
//            
//        }
//        std::cout<<"\n\n";
