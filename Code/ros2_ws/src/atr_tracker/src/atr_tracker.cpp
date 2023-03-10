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
    participants=-1;
    max_id = -1;

    // get parameters from yaml file
    this->declare_parameter<std::int32_t>("architecture",0);
    this->get_parameter("architecture", system_architecture);
    this->declare_parameter<bool>("debug",0);
    this->get_parameter("debug", debug);
    this->declare_parameter<std::int32_t>("watchdog_period",600);
    this->get_parameter("watchdog_period", wd_period);
    this->declare_parameter<std::int32_t>("max_pos_age",600);
    this->get_parameter("max_pos_age", max_msg_age);
    this->declare_parameter<std::int32_t>("list_publisher_period",500);
    this->get_parameter("list_publisher_period", state_list_period);

    // create connection service, watchdog timer and state list publisher

    connection_service_ = this->create_service<atr_interfaces::srv::ConnectionSrv>("/atr_tracker/ConnectSrv", std::bind(&ATRTracker::connection_service_callback,this,_1,_2));

    //watchdog_service_ = this->create_service<atr_interfaces::srv::ChangeWatchdogSrv>("/atr_tracker/ChangeWatchdogSrv", std::bind(&ATRTracker::update_watchdog_timing_callback,this,_1,_2));

    state_list_publisher = this->create_publisher<atr_interfaces::msg::ATRStateListStamped>("atr_state_list",10);

    watchdog_timer_ = this->create_wall_timer(std::chrono::milliseconds(wd_period), std::bind(&ATRTracker::watchdog_callback, this));

    // service for updating state list period
    state_list_service_  = this->create_service<atr_interfaces::srv::ChangeListPublisherSrv>("/atr_tracker/ChangeListPublisherSrv", std::bind(&ATRTracker::update_state_list_timing_callback,this,_1,_2));

  }


private:

  void update_state_list_timing_callback(const std::shared_ptr<atr_interfaces::srv::ChangeListPublisherSrv::Request> request,std::shared_ptr<atr_interfaces::srv::ChangeListPublisherSrv::Response> response){
    // set the timer period
    if(timer_!=NULL){
      timer_ = this->create_wall_timer(std::chrono::milliseconds(request->period), 
          std::bind(&ATRTracker::publisher_callback, this));
    }
    response->ack = 1;
  }

  void update_watchdog_timing_callback(const std::shared_ptr<atr_interfaces::srv::ChangeWatchdogSrv::Request> request,
    std::shared_ptr<atr_interfaces::srv::ChangeWatchdogSrv::Response> response){
    // Update the watchdog timer period
    max_msg_age = request->msg_age;
    wd_period = request->period;

    RCLCPP_INFO(this->get_logger(), "Incoming request to reset watchdog to %i and max age to %i", wd_period ,max_msg_age);

    if(wd_period == -1){
      // switch off watchdog
      watchdog_timer_ = NULL;
    }
    else{
      watchdog_timer_ = this->create_wall_timer(std::chrono::milliseconds(wd_period), std::bind(&ATRTracker::watchdog_callback, this));
    }

    response->ack = 1;
    RCLCPP_INFO(this->get_logger(), "WATCHDOG RESETTED");
  }

  void connection_service_callback(const std::shared_ptr<atr_interfaces::srv::ConnectionSrv::Request> request,
    std::shared_ptr<atr_interfaces::srv::ConnectionSrv::Response> response)
    {
      auto action = request->action;
      response->ack = request->id;
      // Check whether element is connected
      int atr_connected = 0;
      if(max_id>=request->id){
        if(connected[request->id]!=-1){
          // ATR Already connected
          atr_connected = 1;
        }
      }
    
      
      // CONNECT ACTION 
      if(action == 0){
        RCLCPP_INFO(this->get_logger(), "Incoming request to connect atr_%i", request->id); 
        if(atr_connected == 0){
          // id handling for connection array
          if(request->id > max_id){
            for(int n=max_id+1;n<request->id;n++){
              connected.push_back(-1);
            }
            connected.push_back(participants+1);
            max_id = request->id;
          }
          else{
            connected[request->id]=participants+1;
          }

          // append reception time index, and state list
          reception_time_array.push_back(now());
          creation_time_array.push_back(request->state_stm.header.stamp);
          // WRITE THE ACTUAL STATE OF THE ATR IN THE DATA -> send it with the service
          atr_interfaces::msg::ATRState tmp = request->state_stm.state;
          
          atr_states_array.push_back(tmp);
          atr_states_msg_.list.atr_states.push_back(tmp);
          
          atr_interfaces::msg::TimeStampRobot tmp2;
          tmp2.id = request->id;
          tmp2.stamp_creation = request->state_stm.header.stamp;
          tmp2.stamp_reception = now();
          atr_states_msg_.list.time_stamps.push_back(tmp2);

          // Create subscription regarding system architecture
          if(system_architecture==0 && participants==-1){
            subscription_ = this->create_subscription<atr_interfaces::msg::ATRStateStamped>(
              "state_", 10, std::bind(&ATRTracker::topic_callback, this, _1));
          }
          if(system_architecture==1){
            subscription_ = this->create_subscription<atr_interfaces::msg::ATRStateStamped>(
              "state_"+std::to_string(request->id), 10, std::bind(&ATRTracker::topic_callback, this, _1));
            v_subscribers_.push_back(std::move(subscription_));
          }
          // append robot id to id array
          id_array.push_back(request->id);
          participants++;
          
          // create timer for publishing state list
          timer_ = this->create_wall_timer(std::chrono::milliseconds(state_list_period), 
          std::bind(&ATRTracker::publisher_callback, this));
          
          RCLCPP_INFO(this->get_logger(), "Return Connection with id %li", response->ack);
        }
        else{
          response->ack = -1;
        }
      }
      // DISCONNECT ACTION
      if(action == 1){
        RCLCPP_INFO(this->get_logger(), "Incoming request to disconnect atr_%i", request->id); 
        if(atr_connected==1){
          // Erase subscriber regarding system architecture
          if(system_architecture==0 && participants==0){
            subscription_ = NULL;
          }
          if(system_architecture==1){
            v_subscribers_.erase(v_subscribers_.begin()+int(connected[request->id]));
          }
          // reset timer when no atr left in the system
          if(participants == 0){
            timer_ = NULL;
          }
          RCLCPP_INFO(this->get_logger(), "SUBSCRIBER ERASED atr_%i", request->id);
          // Erase robot from list and id handling
          for(int n = connected[request->id]+1;n<int(id_array.size());n++){
              connected[id_array[n]]-=1;
          }
          
          RCLCPP_INFO(this->get_logger(), "CONNECTED ClEANED atr_%i", request->id);
          
          reception_time_array.erase(reception_time_array.begin()+int(connected[int(request->id)]));
          creation_time_array.erase(creation_time_array.begin()+int(connected[int(request->id)]));
          atr_states_array.erase(atr_states_array.begin()+int(connected[int(request->id)]));
          atr_states_msg_.list.atr_states.erase(atr_states_msg_.list.atr_states.begin()+int(connected[int(request->id)]));
          atr_states_msg_.list.time_stamps.erase(atr_states_msg_.list.time_stamps.begin()+int(connected[int(request->id)]));
    

          id_array.erase(id_array.begin()+int(connected[int(request->id)]));
          connected[int(request->id)]=-1;
          
          RCLCPP_INFO(this->get_logger(), "CONNECTED CLEARED atr_%i", request->id);
          RCLCPP_INFO(this->get_logger(), "CLEAR VARIABLES::_%i", request->id);


          // Allocate ressources in id array
          if(request->id == max_id){
            connected.erase(connected.begin()+int(request->id));
            int n = connected.size()-1;
            int idx = 0;
            while(idx == 0){
              if(n==-1||connected[n]!=-1){
                max_id = n;
                idx=1;
              }
              else{
                connected.pop_back();
              }
              n--;
            }
          }

          participants--;
          RCLCPP_INFO(this->get_logger(), "Return Disconnection with id %li", response->ack);
        }
        else{
          response->ack = -1;
        }
      }
      // DEBUG:
      if(debug){ 
        // print all variables and vectors:
        RCLCPP_INFO(this->get_logger(), "DEBUG CONNECTION HANDLER ATR_%i", request->id);
        RCLCPP_INFO(this->get_logger(), "max_id: %i", max_id);
        RCLCPP_INFO(this->get_logger(), "participants: %i", participants);
        std::string tmp_msg = "connected: [";
        for(int n=0;n<int(connected.size());n++){
          tmp_msg=tmp_msg+" "+std::to_string(connected[n]);
        }
        tmp_msg = tmp_msg+"]";
        RCLCPP_INFO(this->get_logger(), tmp_msg.c_str());

        tmp_msg = "id_array: [";
        for(int n=0;n<int(id_array.size());n++){
          tmp_msg=tmp_msg+" "+std::to_string(id_array[n]);
        }
        tmp_msg = tmp_msg+"]";
        RCLCPP_INFO(this->get_logger(), tmp_msg.c_str());
      }
      RCLCPP_INFO(this->get_logger(), "RETURN TO NODE");
    }

  void topic_callback(const atr_interfaces::msg::ATRStateStamped::SharedPtr msg)
  {
    // Sort state message in state list
    //RCLCPP_INFO(this->get_logger(), "Received: '%li'", msg->state.atr_id);
    int32_t t_idx = connected[msg->state.atr_id];
    creation_time_array.at(t_idx) = msg->header.stamp;
    reception_time_array.at(t_idx) = now();
    atr_states_array[t_idx] = msg->state;
    atr_states_array[t_idx].full_state = true;
    
  }

  void watchdog_callback()
  {
    rclcpp::Time tc = now();
    for (int64_t i = 0; i <= (participants); i++)
    {
      double elapsed_time = (tc - reception_time_array.at(i)).nanoseconds() * 1E-9;

      // If the ATR has not updated its state, then reset the full_state flag
      if (elapsed_time > double(max_msg_age)/1000)
      {
        int64_t t_idx = id_array[i];
        atr_states_array[t_idx].full_state = false;
        // DEADLINE MISS!!
        RCLCPP_INFO(this->get_logger(), "DEADLINE MISS ATR_%li", t_idx);
      }
    }
  }

  void publisher_callback()
  {
    atr_states_msg_.header.stamp = now();

    for(int i = 0; i<=participants;i++)
    {
      int64_t t_id = id_array.at(i);
      int64_t t_idx = connected[t_id];

      
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
  rclcpp::TimerBase::SharedPtr watchdog_timer_;  ///< defines the frequency of the watchdog

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
