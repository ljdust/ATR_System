/*! \file
 *
 * \author Emmanuel Dean
 *
 * \version 0.1
 * \date 15.03.2021
 *
 * \copyright Copyright 2021 Chalmers
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received financial  support  from  Chalmers  AI  Re-search Centre
 *  (CHAIR) and AB Volvo (Project ViMCoR).
 */

#include <atr_visualisation/ATRStateListSubscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace atr_visualisation
{
ATRStateListSubscriber::ATRStateListSubscriber() : Node("atr_list_subscriber"), first_message_(true)
{
  // Init parameters
  init();

  // Subscription to ATRState list topic
  subscription_ = create_subscription<atr_interfaces::msg::ATRStateListStamped>(
      subs_topic_name_, 10, std::bind(&ATRStateListSubscriber::topic_callback, this, _1));

  // Marker publisher
  v_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_name_, 10);

  // Instantiate tf broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

ATRStateListSubscriber::~ATRStateListSubscriber()
{
}

void ATRStateListSubscriber::init()
{
  std::vector<std::string> param_names = { "subs_topic_name", "marker_topic_name", "frame_id", "marker_namespace" };
  for (auto&& i : param_names)
    this->declare_parameter<std::string>(i);
  std::vector<rclcpp::Parameter> params = this->get_parameters(param_names);

  subs_topic_name_ = params.at(0).as_string();
  marker_topic_name_ = params.at(1).as_string();
  frame_id_ = params.at(2).as_string();
  marker_namespace_ = params.at(3).as_string();

  // Robot Marker initialization
  // clang-format off
  robot_marker_ = createMarkerMesh(frame_id_,
                                   marker_namespace_,
                                   0, /*ID to be changed (TBC)*/
                                   visualization_msgs::msg::Marker::MESH_RESOURCE,
                                   0.0, 0.0, 0.0, /* position TBC*/
                                   0.0, 0.0, 0.0, 1.0,
                                   0.4, 0.4, 0.4,
                                   168 / 255.0, 165 / 255.0, 155 / 255.0,
                                   0.9,
                                   "package://atr_visualisation/urdf/meshes/hrp/hrp_min_single_a.stl");
  //  clang-format on
  
}

void ATRStateListSubscriber::topic_callback(const atr_interfaces::msg::ATRStateListStamped::SharedPtr msg)
{
  // At this point, both the c_publishers and the objects must have the same size
  // for (size_t i = 0; i < msg->objects.size(); i++)


  visualization_msgs::msg::MarkerArray m_marker_msg;
  visualization_msgs::msg::Marker aux_robot_marker = robot_marker_;
  std::vector<geometry_msgs::msg::TransformStamped> v_ts;


  rclcpp::Time aux_time = now();

  for (auto&& atrs : msg->list.atr_states)
  {
    geometry_msgs::msg::TransformStamped ts;
    tf2::Transform tf;

    // Get the object id
    int16_t atr_id = atrs.atr_id;
    
    // Populate the Marker Array to visualize the ATR as a mesh
    std::string atr_name ="atr_" + std::to_string(atr_id)+ "_current";
    aux_robot_marker.header.stamp = aux_time;
    //Frame generated by the ATRState publisher ()
    aux_robot_marker.header.frame_id = atr_name;
    aux_robot_marker.ns = "ns_atr_" + std::to_string(atr_id);

    // Use the same object ID for the marker ID
    aux_robot_marker.id = atr_id;

    //Get pose from msg
    // Publish TFs for ATRs
    // if the ATRStateList publisher is full pose use fused_odom, otherwise use optom
    atrs.full_state ? tf2::fromMsg(atrs.pose.fused_odom, tf) : tf2::fromMsg(atrs.pose.optom, tf);

    
    ts.transform = tf2::toMsg(tf);
    ts.header.frame_id = frame_id_;
    ts.header.stamp = aux_time;
    ts.child_frame_id = atr_name;

    // Create ts msg
    v_ts.push_back(ts);

    // Create MarkerArray msg
    m_marker_msg.markers.push_back(aux_robot_marker);


  }

  // Publish transformations
  tf_broadcaster_->sendTransform(v_ts);

  // Publish Polygons as Line Markers
  v_marker_publisher_->publish(m_marker_msg);
}
}  // namespace atr_examples
