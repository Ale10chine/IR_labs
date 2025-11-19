#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp" // Msg needed for the Topic
#include <array> // Necessario per popolare l'array di covarianza


using namespace std::chrono_literals;

class Nav2ActionClient : public rclcpp::Node
{
public:
    using NavigateToPoseAction = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPoseAction>;

    Nav2ActionClient() : Node("nav2_action_client")
    {

        // Publisher for setting the initial pose
        initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose",
            10
        );
        
        // 2. Creazione di un timer per pubblicare il messaggio (lo pubblichiamo una volta)
        timer_ = this->create_wall_timer(
            500ms, 
            std::bind(&Nav2ActionClient::publish_initial_pose, this)
        );

        action_client_ = rclcpp_action::create_client<NavigateToPoseAction>(
            this, "navigate_to_pose");

        
    }

    void send_goal()
    {

        RCLCPP_INFO(this->get_logger(), "Sending goal ");
        auto goal_msg = NavigateToPoseAction::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = 3.0;
        goal_msg.pose.pose.position.y = 1.0;
        goal_msg.pose.pose.orientation.w = 1.0;
        
        action_client_->wait_for_action_server();

        auto send_goal_options = rclcpp_action::Client<NavigateToPoseAction>::SendGoalOptions();

        // 1. Goal Response Callback (RICEZIONE della risposta dal server)
        send_goal_options.goal_response_callback =
            std::bind(&Nav2ActionClient::goal_response_callback, this, std::placeholders::_1);

        // 2. Feedback Callback (RICEZIONE dei messaggi intermedi)
        send_goal_options.feedback_callback =
            std::bind(&Nav2ActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

        // 3. Result Callback (RICEZIONE del risultato finale)
        send_goal_options.result_callback =
            std::bind(&Nav2ActionClient::result_callback, this, std::placeholders::_1);

        // Invia il goal in modo asincrono
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }


      void publish_initial_pose()
    {
        if (published_once_) {
            // Dopo la prima pubblicazione, ferma il timer e il nodo
            timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "Initial pose published once. ");
            // Potresti chiamare rclcpp::shutdown() qui in un'applicazione reale se volessi terminare l'esecuzione.
            return;
        }

        // 3. Creazione e Popolazione del Messaggio
        auto initial_pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();

        // --- HEADER ---
        // frame_id: 'map'
        initial_pose_msg.header.frame_id = "map";
        initial_pose_msg.header.stamp = this->now();

        // --- POSE: POSITION ---
        // position: {x: 0.0, y: 0.0, z: 0.0}
        initial_pose_msg.pose.pose.position.x = 0.0;
        initial_pose_msg.pose.pose.position.y = 0.0;
        initial_pose_msg.pose.pose.position.z = 0.0;

        // --- POSE: ORIENTATION (Quaternion) ---
        // orientation: {z: 0.0, w: 1.0} (Orientamento nullo / nessuna rotazione, z=0.0, w=1.0)
        initial_pose_msg.pose.pose.orientation.x = 0.0;
        initial_pose_msg.pose.pose.orientation.y = 0.0;
        initial_pose_msg.pose.pose.orientation.z = 0.0;
        initial_pose_msg.pose.pose.orientation.w = 1.0;

        // --- COVARIANCE (Array 6x6) ---
        // I valori di covarianza devono essere inseriti in un array di 36 elementi (6x6)
        // I valori che hai fornito indicano un'incertezza sulla posizione (x, y) e sull'orientamento (theta / z-rotation)
        std::array<double, 36> covariance_array = {
            // Row 1 (x-x, x-y, x-z, x-roll, x-pitch, x-yaw)
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            // Row 2 (y-x, y-y, y-z, y-roll, y-pitch, y-yaw)
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            // Row 3 (z-x, z-y, z-z, z-roll, z-pitch, z-yaw)
            0.0, 0.0, 0.068, 0.0, 0.0, 0.0, // Nota: Z è la terza riga
            // Row 4 (roll-x, roll-y, roll-z, roll-roll, roll-pitch, roll-yaw)
            0.0, 0.0, 0.0, 0.068, 0.0, 0.0,
            // Row 5 (pitch-x, pitch-y, pitch-z, pitch-roll, pitch-pitch, pitch-yaw)
            0.0, 0.0, 0.0, 0.0, 0.068, 0.0,
            // Row 6 (yaw-x, yaw-y, yaw-z, yaw-roll, yaw-pitch, yaw-yaw)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.068
        };
        initial_pose_msg.pose.covariance = covariance_array;

        // 4. Pubblicazione del Messaggio
        initial_pose_publisher_->publish(initial_pose_msg);
        RCLCPP_INFO(this->get_logger(), "Published Initial Pose at (0.0, 0.0) in 'map' frame.");
        published_once_ = true;
    }

private:
    rclcpp_action::Client<NavigateToPoseAction>::SharedPtr action_client_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;// Publisher for the /initialpose topic
    rclcpp::TimerBase::SharedPtr timer_;
    bool published_once_;


    // 1. Funzione per la risposta iniziale del Server (molto importante!)
    void goal_response_callback(GoalHandle::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server. Waiting for result and feedback...");
            // Il goal handle ora è vivo e la comunicazione può procedere.
        }
    }
    
    // 2. Funzione per il Feedback (come la tua, ma più informativa)
    void feedback_callback(GoalHandle::SharedPtr, 
                           const std::shared_ptr<const NavigateToPoseAction::Feedback> feedback)
    {
        // Puoi stampare la distanza rimanente, che è un dato utile nel feedback di NavigateToPose
        RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f m", feedback->distance_remaining);
    }

    // 3. Funzione per il Risultato Finale
    void result_callback(const GoalHandle::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal SUCCEEDED!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal ABORTED!");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Goal CANCELED!");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code!");
                break;
        }
        // Il nodo ha terminato il suo compito, può chiudersi.
        rclcpp::shutdown(); 
    }




};


int main(int argc, char **argv)
{
    // Initialize ros2 cpp library
    rclcpp::init(argc, argv);

    // Create an instance of the server node (as a shared_ptr)
    std::shared_ptr<Nav2ActionClient> nav_node = std::make_shared<Nav2ActionClient>();

    nav_node->send_goal();
    // Block the execution and spin the node, allowing it to listen for requests.
    // The node will continue to 'spin' until interrupted (e.g., Ctrl+C).
    rclcpp::spin(nav_node);

    // Clean up and shut down the ROS 2 library
    rclcpp::shutdown();
    return 0;
}