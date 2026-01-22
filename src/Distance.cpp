#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <string.h>
#include <math.h>

#define threshold 2.0

using std::placeholders::_1;

class DistanceNode : public rclcpp::Node
{
    public:
        DistanceNode() : Node("distance_node")
        {
            publisher1_ = this->create_publisher<std_msgs::msg::Float32>("collision_condition", 10); // To publish eventual violation of boundaries
            publisher2_ = this->create_publisher<std_msgs::msg::String>("simulation_info", 10); // To publish custom message with distance from colsest obstacle, its direction and velocity state and the threshold
            subscription1_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10,std::bind(&DistanceNode::topic_callback1, this, _1));
            subscription2_ = this->create_subscription<std_msgs::msg::Float32>("updated_threshold", 10,std::bind(&DistanceNode::topic_callback2, this, _1));
            subscription3_ = this->create_subscription<std_msgs::msg::Float32>("movement_state", 10,std::bind(&DistanceNode::topic_callback3, this, _1));
            main_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&DistanceNode::main_loop, this));
        }
    private:
        void topic_callback1(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan){
            laser_scan_ = *laser_scan;
        }

        void topic_callback2(const std_msgs::msg::Float32::SharedPtr new_threshold){
            RCLCPP_INFO(this->get_logger(), "Threshold updated to %.2f", new_threshold->data);
            threshold_ = new_threshold->data;
        }

        void topic_callback3(const std_msgs::msg::Float32::SharedPtr movement_state){
            // 1.0 = moving, 0.0 = stopped
            if(movement_state->data == 1.0)
                is_moving_ = true;
            else
                is_moving_ = false;
        }

        // Computes distance between two turtles and publishes it to ui_node
        void main_loop(){
            collision_condition_.data = 0;
            double min_distance = laser_scan_.range_max;
            float closest_angle = 0.0f;
            int closest_index = -1;
            int i = 0;
            for(auto range : laser_scan_.ranges){
                /*
                if (range < laser_scan_.range_min || range > laser_scan_.range_max) {
                    // Invalid range reading
                    continue;
                }
                */
               // Track closest obstacle
                if(range < min_distance){
                    min_distance = range;
                    closest_index = i;
                    closest_angle = laser_scan_.angle_min + (i * laser_scan_.angle_increment);
                }
                if(range < (laser_scan_.range_min + precision_)){
                    // Drone is dangerously close, stop immediately regardless of state
                    RCLCPP_INFO(this->get_logger(), "Drone too close to obstacle, stopping immediately");
                    collision_condition_.data = 1; // Obstacle too close, need to stop and restore previous position
                    break;
                }
                // Only check threshold if drone is NOT moving
                if(/*!is_moving_ &&*/ range < threshold_){
                    // Used after drone stopped
                    RCLCPP_INFO(this->get_logger(), "Distance from obstacle below threshold %.2f", threshold_);
                    collision_condition_.data = 2; // Distance from obstacle below threshold, restoring previous position
                    break;
                }
                i++;
            }
            publisher1_->publish(collision_condition_);
            std_msgs::msg::String info_msg;
            if(closest_index == -1)
                info_msg.data = "No obstacles detected within sensor range, threshold: " + std::to_string(threshold_);
            else{
                info_msg.data = "Closest obstacle: " + std::to_string(min_distance);
                if(closest_angle < -0.3 && closest_angle > -2.8)
                    info_msg.data += "in direction: RIGHT";
                else if(closest_angle > 0.3 && closest_angle < 2.8)
                    info_msg.data += "in direction: LEFT";
                else if(fabs(closest_angle) <= 0.3)
                    info_msg.data += "in direction: FRONT";
                else
                    info_msg.data += "in direction: BACK";
            }
            info_msg.data += ", threshold: " + std::to_string(threshold_);
            publisher2_->publish(info_msg);
            RCLCPP_INFO(this->get_logger(), "%s", info_msg.data.c_str());
        }

        rclcpp::TimerBase::SharedPtr main_timer_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher1_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher2_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription2_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription3_;
        std_msgs::msg::Float32 collision_condition_;
        sensor_msgs::msg::LaserScan laser_scan_;
        double threshold_ = threshold;
        bool is_moving_ = false;
        const double precision_ = 1.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceNode>());
    rclcpp::shutdown();
    return 0;
}