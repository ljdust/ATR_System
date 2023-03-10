#include <memory>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "atr_interfaces/msg/atr_state_stamped.hpp"
#include "atr_interfaces/msg/atr_state_list_stamped.hpp"

#include <atr_interfaces/convert_extended.h>

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "atr_interfaces/srv/connection_srv.hpp"
#include "atr_interfaces/srv/change_watchdog_srv.hpp"
#include "atr_interfaces/srv/change_list_publisher_srv.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

// for using ms as operator in timer function
using namespace std::chrono_literals;



class ATRTracker : public rclcpp::Node
{
public:
  using Subscriber_list = rclcpp::Subscription<atr_interfaces::msg::ATRStateStamped>::SharedPtr;
  std::vector<Subscriber_list> v_subscribers_;

  ATRTracker() : Node("atr_tracker")
  {
    this->declare_parameter<std::int32_t>("architecture",0);
    this->get_parameter("architecture", system_architecture);
    this->declare_parameter<bool>("debug",0);
    this->get_parameter("debug", debug);
    this->declare_parameter<std::int32_t>("list_publisher_period",500);
    this->get_parameter("list_publisher_period", state_list_period);
    this->declare_parameter<std::int32_t>("num_of_connected_atr",0);
    this->get_parameter("num_of_connected_atr", participants);

    RCLCPP_INFO(this->get_logger(), "Architecture %i",system_architecture);
    RCLCPP_INFO(this->get_logger(), "Participants %i",participants);
    if(system_architecture == 1){
      for (int idx; idx<participants; idx++){
        subscription_ = this->create_subscription<atr_interfaces::msg::ATRStateStamped>(
              "state_"+std::to_string(idx), 10, std::bind(&ATRTracker::topic_callback, this, _1));
        v_subscribers_.push_back(std::move(subscription_));
      }
    }
    else if(system_architecture == 0){
      subscription_ = this->create_subscription<atr_interfaces::msg::ATRStateStamped>(
              "state_", 10, std::bind(&ATRTracker::topic_callback, this, _1));
    }

    if(participants > 0){
      timer_ = this->create_wall_timer(std::chrono::milliseconds(state_list_period), 
          std::bind(&ATRTracker::publisher_callback, this));
    }
    atr_states_msg_.list.time_stamps.resize(participants);
    atr_states_msg_.list.atr_states.resize(participants);
    creation_time_array.resize(participants);
    reception_time_array.resize(participants);
    atr_states_array.resize(participants);

    state_list_publisher = this->create_publisher<atr_interfaces::msg::ATRStateListStamped>("atr_state_list",10);

  }


private:

  
  void topic_callback(const atr_interfaces::msg::ATRStateStamped::SharedPtr msg)
  {
    //RCLCPP_INFO(this->get_logger(), "Received: '%li'", msg->state.atr_id);
    int32_t t_idx = msg->state.atr_id;
    creation_time_array.at(t_idx) = msg->header.stamp;
    reception_time_array.at(t_idx) = now();
    atr_states_array[t_idx] = msg->state;
    atr_states_array[t_idx].full_state = true;
    
  }


  void publisher_callback()
  {
    atr_states_msg_.header.stamp = now();

    for(int i = 0; i<participants;i++)
    {
      int64_t t_id = i;
      int64_t t_idx = i;
      
      atr_states_msg_.list.time_stamps[i].id = t_id;
      atr_states_msg_.list.time_stamps[i].stamp_creation = creation_time_array[t_idx];
      atr_states_msg_.list.time_stamps[i].stamp_reception = reception_time_array[t_idx];
      /*atr_states_msg_.list.time_stamps_creation.stamp[i] = now();//creation_time_array[t_idx];
      atr_states_msg_.list.time_stamps_creation.id[i] = t_id;
      atr_states_msg_.list.time_stamps_reception_tracker.stamp[i] = now();//reception_time_array[t_idx]; 
      atr_states_msg_.list.time_stamps_reception_tracker.id[i] = t_id;*/
      atr_states_msg_.list.atr_states[i].atr_id = t_id;
      atr_states_msg_.list.atr_states[i].full_state = false;
      atr_states_msg_.list.atr_states[i].pose_source = atr_interfaces::msg::ATRState::OPTOM;
      // Generate a OPTOM pose
      tf2::Transform tf;
      tf2::Quaternion q;
      geometry_msgs::msg::TransformStamped ts;

      q.setRPY(0, 0, 0);  // Basic Rotation in z
      q.normalize();

      tf.setOrigin(tf2::Vector3(t_id, 0, 0));
      tf.setRotation((q).normalize());

      ts.transform = tf2::toMsg(tf);

      // Define the optom pose using Ts info
      tf2::convert(ts.transform, atr_states_msg_.list.atr_states[i].pose.optom);
      // Time derivative of the optom pose
      atr_states_msg_.list.atr_states[i].vel.optom.linear.x = 1;
      atr_states_msg_.list.atr_states[i].vel.optom.linear.y = 1;
      atr_states_msg_.list.atr_states[i].vel.optom.linear.z = 0;
      atr_states_msg_.list.atr_states[i].vel.optom.angular.x = 0;
      atr_states_msg_.list.atr_states[i].vel.optom.angular.y = 0;
      atr_states_msg_.list.atr_states[i].vel.optom.angular.z = 0.1 / (0.001 * 10);
      // Getting ODOM pose
      // If the ATR has already published data, we use its info to populate the atr_state_list (odom+fused_odom)
      // We have to guard again the shared object during the writing/reading process
      // std::lock_guard<std::mutex> guard(atr_states_array[t_idx]);

      if (atr_states_array.at(t_idx).full_state)
      {
        atr_states_msg_.list.atr_states[i].pose.fused_odom = atr_states_array.at(t_idx).pose.fused_odom;
        atr_states_msg_.list.atr_states[i].pose.odom = atr_states_array.at(t_idx).pose.odom;
        atr_states_msg_.list.atr_states[i].pose_source = atr_interfaces::msg::ATRState::FULL_POSE;
        atr_states_msg_.list.atr_states[i].full_state = atr_states_array.at(t_idx).full_state;
      }
      // Generating GOAL pose
      q.setRPY(0, 0, M_PI);
      // Just for debugging, we use the atr_id as goal
      double goal = static_cast<double>(t_id);
      tf.setOrigin(tf2::Vector3(goal, 1, 0));
      tf.setRotation(q);
      ts.transform = tf2::toMsg(tf);
      tf2::convert(tf2::toMsg(tf), atr_states_msg_.list.atr_states[i].goal);
    }

    state_list_publisher->publish(atr_states_msg_);
  }

  // DECLARE GLOBAL VARIABLES
  rclcpp::Subscription<atr_interfaces::msg::ATRStateStamped>::SharedPtr subscription_;
  rclcpp::Service<atr_interfaces::srv::ConnectionSrv>::SharedPtr connection_service_;
  rclcpp::Publisher<atr_interfaces::msg::ATRStateListStamped>::SharedPtr state_list_publisher;
  rclcpp::Service<atr_interfaces::srv::ChangeWatchdogSrv>::SharedPtr watchdog_service_;
  rclcpp::Service<atr_interfaces::srv::ChangeListPublisherSrv>::SharedPtr state_list_service_;

  rclcpp::TimerBase::SharedPtr timer_;           ///< defines the frequency of the publisher

  int system_architecture; 
  
  
  int participants;
  int max_id;
  std::vector<std::int32_t> connected;
  std::vector<std::int32_t> id_array;
  bool debug;

  std::vector<rclcpp::Time> reception_time_array;
  std::vector<rclcpp::Time> creation_time_array;
  std::vector<atr_interfaces::msg::ATRState> atr_states_array;
  atr_interfaces::msg::ATRStateListStamped atr_states_msg_;
  std::int32_t state_list_period;
  std::int32_t wd_period;
  std::int32_t max_msg_age;

};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ATRTracker>());
  rclcpp::shutdown();
  return 0;
}