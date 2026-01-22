#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "assignment2_rt_services/srv/threshold.hpp"
#include "assignment2_rt_services/srv/velocity_averages.hpp"
#include <iostream>
#include <string.h>
#include <math.h>

using Threshold = assignment2_rt_services::srv::Threshold;
using VelocityAverages = assignment2_rt_services::srv::VelocityAverages;
using std::placeholders::_1;

class UINode : public rclcpp::Node
{
    public:
        UINode() : Node("ui_node")
        {
            publisher1_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            publisher2_ = this->create_publisher<std_msgs::msg::Float32>("movement_state", 10);
            subscription1_ = this->create_subscription<std_msgs::msg::Float32>("collision_condition", 10, std::bind(&UINode::topic_callback1, this, _1));
            main_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&UINode::main_loop, this));
            threshold_update_client_ = this->create_client<Threshold>("change_threshold");
            velocity_averages_client_ = this->create_client<VelocityAverages>("velocity_averages");
        }
    private:
        
        // Main loop of execution
        void main_loop(){

            if(!waiting_for_input_) 
                return;
            /*
            std::cout << "collision_condition_: " << collision_condition_.load() << " replacing_position_executed_: " << replacing_position_executed_.load() << "\n";
            if(!replacing_position_executed_ && collision_condition_ == 2){
                replacing_position_executed_ = true;

                std::cout << "Restoring previous position due to distance from obstacles being below threshold\n";
                std::cout << "Applied velocity is" << -vel_x_ << "\n";
                drone_twist_.linear.x = -vel_x_;
                publisher1_->publish(drone_twist_);
                waiting_for_input_ = false;

                std::cout << "Vel published\n";
                
                timer3_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&UINode::timer_callback3, this));
                return;
            }
            */
            user_input();
            if(op_code_ == 0){
                rclcpp::shutdown();
                return;
            }
            rotate_turtle();
            waiting_for_input_ = false;
        }

        // Manages textual interface
        void user_input(){
            op_code_ = 0; vel_x_ = 0.0; vel_ang_ = 0.0;
            std::cout << "Type 1 to move the drone, 2 to change the threshold, 3 to compute velocity averages or 0 to quit:\n";
            std::cin >> op_code_;
            if(op_code_ != 1 && op_code_ != 2 && op_code_ != 3 && op_code_ != 0){
                std::cout << "Invalid operation -> retry\n";
                return;
            }
            if(op_code_ == 1){
                // Normal movement operation
                std::cout << "Insert angular speed: \n";
                std::cin >> vel_ang_;
                std::cout << "Insert linear speed on x: \n";
                std::cin >> vel_x_;

                // Storing last 5 velocities for averages computation (not in order)
                input_count_++;
                if(input_count_ == 5)
                    // Collected enough inputs (5)
                    incomplete_data_ = false;
                if(input_count_ > 5)
                    // Keep only last 5 inputs
                    input_count_ = 1; // Will overwrite oldest data
                last_angular_velocities_[input_count_ - 1] = vel_ang_;
                last_linear_velocities_[input_count_ - 1] = vel_x_;
            }
            else if(op_code_ == 2){
                // Changing threshold operation through service

                std::cout << "Insert new threshold value (from 1.0 to 8.0):\n";
                std::cin >> new_threshold_.data;
                waiting_for_input_ = false;  // Pause UI until request completes
                threshold_request();
            }
            else if(op_code_ == 3){
                // Computing velocity averages operation
                std::cout << "Velocity averages computation requested.\n";
                waiting_for_input_ = false;  // Pause UI until request completes
                velocity_averages_request();
            }
            return;
        }

        // Applies turtle rotation
        void rotate_turtle(){
            drone_twist_.angular.z = vel_ang_;
            publisher1_->publish(drone_twist_);
            timer1_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&UINode::timer_callback1, this));
        }

        // Stops turtle rotation after 1 second
        void timer_callback1(){
            drone_twist_.angular.z = 0.0;
            publisher1_->publish(drone_twist_);
            timer1_->cancel();
            move_turtle_x();
            return;
        }

        // Sends request to threshold_change_service for new threshold value
        void threshold_request(){

            if (!threshold_update_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(this->get_logger(), "Waiting for threshold-change service...");
                waiting_for_input_ = true;  // Allow user to retry
                return;
            }

            auto request = std::make_shared<Threshold::Request>();
            const auto requested_threshold = new_threshold_.data;
            request->new_threshold = requested_threshold;

            auto threshold_request_future_ = threshold_update_client_->async_send_request(
                request,
                [this, requested_threshold](rclcpp::Client<Threshold>::SharedFuture future){

                    auto response = future.get();
                    if(response->success){
                        RCLCPP_INFO(this->get_logger(), "Updated threshold from service: %.2f", requested_threshold);
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Updated threshold failed due to: %s", response->message.c_str());
                    }

                    waiting_for_input_ = true;  // Re-enable UI prompt
                }
            );
        }

        // Sends request to velocity_averages_service for velocity averages computation
        void velocity_averages_request(){
            
             if (!velocity_averages_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(this->get_logger(), "Waiting for averages-velocity service...");
                waiting_for_input_ = true;  // Allow user to retry
                return;
            }

            auto request = std::make_shared<VelocityAverages::Request>();
            request->incomplete_data = incomplete_data_;
            for(int i = 0; i < 5; i++){
                request->last_linear_velocities[i] = last_linear_velocities_[i];
                request->last_angular_velocities[i] = last_angular_velocities_[i];
            }
            auto velocity_averages_request_future_ = velocity_averages_client_->async_send_request(
                request,
                [this](rclcpp::Client<VelocityAverages>::SharedFuture future){

                    auto response = future.get();
                    if(response->success){
                        RCLCPP_INFO(this->get_logger(), "Updated velocity averages from service: %.2f, %.2f", response->linear_velocity_mean, response->angular_velocity_mean);
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Velocty averages computation failed due to: %s", response->msg.c_str());
                    }

                    waiting_for_input_ = true;  // Re-enable UI prompt
                }
            );
        }

        // Applies linear velocity along x axis relative to local reference frame of the turtle
        void move_turtle_x(){
            is_moving_ = true;  // Flag to track movement state
            movement_state_.data = 1.0;  // Publish moving state
            publisher2_->publish(movement_state_);
            drone_twist_.linear.x = vel_x_;
            publisher1_->publish(drone_twist_);
            timer2_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&UINode::timer_callback2, this));
        }
        
        // Triggers after 1 second from start of linear movement to stop turtle translation
        void timer_callback2(){
            is_moving_ = false;  // Movement period ended
            movement_state_.data = 0.0;  // Publish stopped state
            publisher2_->publish(movement_state_);
            drone_twist_.linear.x = 0.0;
            publisher1_->publish(drone_twist_);
            timer2_->cancel();

            // Wait for drone to physically stop and sensor to update collision state
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "Movement operation ended\n";
            std::cout << "Drone stopped, checking collision state\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(300));

            // Checking whether to restore previous position
            if(collision_condition_ == 2){
                std::cout << "Restoring previous position due to distance from obstacles being below threshold\n";
                replacing_position_executed_ = true;
                timer4_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&UINode::start_backward_motion, this));
            } else {
                waiting_for_input_ = true;
                std::cout << "Ready for next command\n";
            }
        }

        // Starts backward motion after robot has stopped
        void start_backward_motion(){
            timer4_->cancel();
            
            std::cout << "Applied velocity is " << -vel_x_ << "\n";
            // Move backward with negative velocity for 1 second to undo the forward movement
            drone_twist_.linear.x = -vel_x_;  // Reverse direction
            publisher1_->publish(drone_twist_);

            std::cout << "Vel published\n";
            
            // After 1 second, stop
            timer3_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&UINode::timer_callback3, this));
        }

        // Stops turtle translation for position restoration
        void timer_callback3(){
            drone_twist_.linear.x = 0.0;
            publisher1_->publish(drone_twist_);
            timer3_->cancel();
            replacing_position_executed_ = false;
            waiting_for_input_ = true;
        }

        // Receives condition relative to distance between the drone and obstacles in real time
        void topic_callback1(const std_msgs::msg::Float32::SharedPtr col_cond){
            collision_condition_ = col_cond->data;

            if(is_moving_ && collision_condition_ == 1){
                // Drone is moving and distance from closest obstacle is below threshold
                collision_condition_ = 2; // Updating condition to signal that previous position must be restored
                std::cout << "Stopping motion: minimum sensor range reached\n";
                if(timer2_)
                    timer2_->cancel();
                timer_callback2();
            }
        }

        rclcpp::TimerBase::SharedPtr timer1_,timer2_, timer3_, timer4_, main_timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher2_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription1_;
        rclcpp::Client<Threshold>::SharedPtr threshold_update_client_;
        rclcpp::Client<VelocityAverages>::SharedPtr velocity_averages_client_;
        geometry_msgs::msg::Twist drone_twist_;
        std_msgs::msg::Float32 new_threshold_;
        std_msgs::msg::Float32 movement_state_;
        std::atomic<bool> waiting_for_input_{true}, is_moving_{false}, replacing_position_executed_{false}, incomplete_data_{true};
        std::atomic<int> collision_condition_{0};
        float last_linear_velocities_[5], last_angular_velocities_[5];
        double vel_x_, vel_ang_;
        int op_code_, input_count_{0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UINode>());
    rclcpp::shutdown();
    return 0;
}