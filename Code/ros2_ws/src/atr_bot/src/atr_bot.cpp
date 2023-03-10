#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "atr_interfaces/msg/atr_state_stamped.hpp"
#include <atr_interfaces/srv/wakeup_srv.hpp>
#include <atr_interfaces/srv/connection_srv.hpp>

#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <atr_interfaces/convert_extended.h>
//#include "tf2_ros/static_transform_broadcaster.h"
//#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

class ATRPublisher : public rclcpp::Node
{
public:
  ATRPublisher()
  : Node("atr_node")
  {
    startup = true;
    action_id = 0;
    this->declare_parameter<std::int32_t>("id",0);
    this->get_parameter("id", atr_id);
    this->declare_parameter<bool>("auto_connect", false );
    this->get_parameter("auto_connect", connected);
    this->declare_parameter<std::string>("state_topic_name", "state_0");  //(paramName, default)
    this->get_parameter("state_topic_name", param_state_topic_name_);              //(paramName, type)

    this->declare_parameter<std::int32_t>("x", 0);  //(paramName, default)
    this->get_parameter("x", param_atr_pos_x_);              //(paramName, type)
    this->declare_parameter<std::int32_t>("y", 0);  //(paramName, default)
    this->get_parameter("y", param_atr_pos_y_);              //(paramName, type)
    this->declare_parameter<std::int32_t>("z", 0);  //(paramName, default)
    this->get_parameter("z", param_atr_pos_z_);              //(paramName, type)

    this->declare_parameter<std::int32_t>("state_publisher_period", 500); 
    this->get_parameter("state_publisher_period", state_pub_period); 

    atr_pose_x = param_atr_pos_x_+atr_id;
    atr_pose_y = param_atr_pos_y_;
    atr_pose_z = param_atr_pos_z_;

    if(connected){
      atr_state_publisher = this->create_publisher<atr_interfaces::msg::ATRStateStamped>(param_state_topic_name_, 10);
      RCLCPP_INFO(this->get_logger(), "START TIMER FOR PUBLISHING %i",atr_id);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(state_pub_period), std::bind(&ATRPublisher::timer_callback, this));
      RCLCPP_INFO(this->get_logger(), "TIMER CREATED %i",atr_id);
    }


    // broadcaster to publish the ATR tf
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  }

private:
  //void timer_callback(){}
  void create_state()
  {
    atr_interfaces::msg::ATRStateStamped message;
    geometry_msgs::msg::TransformStamped ts;
    std::vector<geometry_msgs::msg::TransformStamped> v_ts;
    // Generate a dummy pose
    // double step = 0.01;
    // double theta = count_ * step;

    //double x = atr_pose_x;
    //double y = atr_pose_y;

    double theta = 0;

    // Using TF. Coordinate frame dummy with respect to map rotating by angle theta

    tf2::Transform tf;
    tf2::Quaternion q, q_z_pi2;

    // RPY = Rot_x*Rot_y*Rot_z
    q.setRPY(0, 0, theta);  // Basic Rotation in z
    q.normalize();

    // q_z_pi2.setRPY(0, 0, M_PI_2);
    // q_z_pi2.normalize();

    // Quaternion to Rotation Matrix (3x3)
    // tf2::Matrix3x3 Rh_m(q);
    // tf.setOrigin(Rh_m * tf2::Vector3(1, 0, 0));

    // x=cos(theta), y=sin(theta)
    //            Rh_m * [1,0,0]^T
    // tf.setOrigin(tf2::Matrix3x3(q) * tf2::Vector3(atr_radius_, 0, 0));

    tf.setOrigin(tf2::Vector3(atr_id, 0, 0));

    // tf.setRotation((q * q_z_pi2).normalize());
    tf.setRotation(q);

    ts.transform = tf2::toMsg(tf);

    ts.header.frame_id = "map";
    ts.header.stamp = now();
    ts.child_frame_id = "atr_"+std::to_string(atr_id);
    v_ts.push_back(ts);

    message.header.frame_id = "map";
    message.header.stamp = ts.header.stamp;
    message.state.atr_id = atr_id;

    // Setting the pose fused_odom using the ts information
    tf2::convert(ts.transform, message.state.pose.fused_odom);
    // Dummy ATR is in Node 1 (charging station)
    message.state.pose.pose_id = 1;
    // Name of the current Node
    message.state.pose.name = "Charging Station";
    // Description of the node
    message.state.pose.description = "Charging station next to the main entrance";
    // Pose Type (what is the ATR doing in this Node)
    message.state.pose.type.id = atr_interfaces::msg::ATRPoseType::IDLE;

    // Time derivative of the fused_odom
    message.state.vel.fused_odom.linear.x = 1;
    message.state.vel.fused_odom.linear.y = 1;
    message.state.vel.fused_odom.linear.z = 0;
    message.state.vel.fused_odom.angular.x = 0;
    message.state.vel.fused_odom.angular.y = 0;
    message.state.vel.fused_odom.angular.z = 0.01 / (0.001 * 10);

    // message.state.full_state = false; Not needed since is set to false by default
    message.state.pose_source = atr_interfaces::msg::ATRState::ODOM;

    // Dummy goal
    double goal_x = atr_id;
    double goal_y = 1;

    q.setRPY(0, 0, M_PI);
    // tf.setOrigin(tf2::Vector3(v_atr_goal_.at(0), v_atr_goal_.at(1), 0));
    tf.setOrigin(tf2::Vector3(goal_x, goal_y, 0));
    tf.setRotation(q);
    ts.transform = tf2::toMsg(tf);
    ts.header.frame_id = "map";
    ts.header.stamp = now();
    ts.child_frame_id = "atr_"+std::to_string(atr_id) + "_goal";
    v_ts.push_back(ts);

    tf2::convert(tf2::toMsg(tf), message.state.goal);

    // Over all state of the ATR
    message.state.overall.status.push_back(atr_interfaces::msg::ATRStateOverall::CHARGING);

    // ATR mission
    message.state.mission.status = atr_interfaces::msg::ATRStateMission::ARRIVED;

    // ATR Load status
    message.state.load.status = atr_interfaces::msg::ATRStateLoad::UNLOADED;

    // ATR Signals (which signals should be activated in the ATR)
    message.state.signal.types.push_back(atr_interfaces::msg::ATRStateSignals::CHARGING);

    // ATR Actuator
    message.state.actuator.type = atr_interfaces::msg::ATRStateActuator::LINEAR;
    message.state.actuator.status = atr_interfaces::msg::ATRStateActuator::CLOSED;
    message.state.actuator.value = 0.0;

    // ATR Emergency stopped?
    message.state.emerg_stop = false;

    // ATR Battery
    message.state.battery.status = atr_interfaces::msg::ATRBatteryState::FULL;
    // Battery charge (100%)
    message.state.battery.charge_state = 1.0;
    // Battery duration (5 hrs)
    message.state.battery.life_time = 5.0;
    // Battery capacity, in this case, we assume the battery has 100% capacity
    message.state.battery.health_state = 1.0;

    // Collision state

    // Collision sensors. We assume two type of sensors (two ultrasonic and one bumper)
    atr_interfaces::msg::ATRCollisionState collision_state;

    // No collision
    collision_state.status = atr_interfaces::msg::ATRCollisionState::NONE;
    // minimal distance to an obstacle (no collision we use the maximum value)
    collision_state.distance = 1000;
    collision_state.description = "Collision Free";

    atr_interfaces::msg::ATRCollisionSensor aux_sensor;

    // Ultrasonic 1
    aux_sensor.id = 0;
    aux_sensor.type = atr_interfaces::msg::ATRCollisionSensor::ULTRA_SONIC;
    aux_sensor.data = 1000.0;
    collision_state.sensors.push_back(aux_sensor);

    // Ultrasonic 2
    aux_sensor.id = 1;
    aux_sensor.type = atr_interfaces::msg::ATRCollisionSensor::ULTRA_SONIC;
    aux_sensor.data = 1000.0;
    collision_state.sensors.push_back(aux_sensor);

    message.state.collisions.push_back(collision_state);

    // Bumper (populating the message without aux instances)
    // add a new collision state
    message.state.collisions.push_back(atr_interfaces::msg::ATRCollisionState());
    // Fill in the variables
    message.state.collisions[1].status = atr_interfaces::msg::ATRCollisionState::NONE;
    message.state.collisions[1].distance = 1000;
    message.state.collisions[1].description = "Bumper off";

    // add a new sensor for this collision state
    message.state.collisions[1].sensors.push_back(atr_interfaces::msg::ATRCollisionSensor());
    // Fill in the parameters of the new sensor
    message.state.collisions[1].sensors[0].type = atr_interfaces::msg::ATRCollisionSensor::BUMPER;
    message.state.collisions[1].sensors[0].id = 3;
    message.state.collisions[1].sensors[0].data = 0;

    v_ts_global = v_ts;
    message_global = message;

  }

  void timer_callback(){
    create_state();
    // Publish transformation (ATR reference frame relative to frame_id_)
    tf_broadcaster_->sendTransform(v_ts_global);

    atr_state_publisher->publish(message_global);
    //RCLCPP_INFO(this->get_logger(), "MEssage send %i",atr_id);

  }

  int atr_id;
  int atr_pose_x;
  int atr_pose_y;
  int atr_pose_z;


  atr_interfaces::msg::ATRStateStamped message_global;
  std::vector<geometry_msgs::msg::TransformStamped> v_ts_global;
  
  std::string param_state_topic_name_;
  bool connected;
  bool startup;
  int action_id;

  std::int32_t param_atr_pos_x_;
  std::int32_t param_atr_pos_y_;
  std::int32_t param_atr_pos_z_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<atr_interfaces::srv::WakeupSrv>::SharedPtr wakeup_service;
  rclcpp::Client<atr_interfaces::srv::ConnectionSrv>::SharedPtr connection_client;
  rclcpp::Publisher<atr_interfaces::msg::ATRStateStamped>::SharedPtr atr_state_publisher;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::int32_t state_pub_period;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //rclcpp::executors::MultiThreadedExecutor executor;
  //executor.add_node(std::make_shared<ATRPublisher>());
  //executor.spin();
  rclcpp::spin(std::make_shared<ATRPublisher>());
  rclcpp::shutdown();
  return 0;
}