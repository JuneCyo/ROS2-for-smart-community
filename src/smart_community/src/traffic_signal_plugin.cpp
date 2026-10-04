#include <array>
#include <cmath>
#include <functional>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace gazebo
{
class TrafficSignalPlugin : public WorldPlugin
{
public:
  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    world_ = world;
    ros_node_ = gazebo_ros::Node::Get(sdf, "traffic_signal_plugin");
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    states_[0] = ros_node_->create_publisher<std_msgs::msg::String>("/traffic_light/light1/state", qos);
    states_[1] = ros_node_->create_publisher<std_msgs::msg::String>("/traffic_light/light2/state", qos);
    gz_node_.reset(new transport::Node());
    gz_node_->Init(world_->Name());
    visual_pub_ = gz_node_->Advertise<msgs::Visual>("~/visual");
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&TrafficSignalPlugin::OnUpdate, this));
    RCLCPP_INFO(ros_node_->get_logger(), "Gazebo Classic 红绿灯已启动：红 10 秒、黄 5 秒、绿 10 秒");
  }

private:
  void OnUpdate()
  {
    const double now = world_->SimTime().Double();
    if (now - last_update_ < 0.5) return;
    last_update_ = now;
    const double phase = std::fmod(now, 25.0);
    const int state = phase < 10.0 ? 0 : (phase < 15.0 ? 1 : 2);
    static const std::array<std::string, 3> names{{"red", "yellow", "green"}};
    for (int i = 0; i < 2; ++i) {
      auto model = world_->ModelByName("traffic_light" + std::to_string(i + 1));
      if (!model) continue;
      std_msgs::msg::String msg;
      msg.data = names[state];
      states_[i]->publish(msg);
      for (int color = 0; color < 3; ++color) {
        auto link = model->GetLink("light_" + names[color]);
        if (!link || link->Visuals().empty()) continue;
        msgs::Visual visual = link->Visuals().begin()->second;
        const auto off = ignition::math::Color(0.025, 0.025, 0.025, 1.0);
        static const std::array<ignition::math::Color, 3> on{{
          {1.0, 0.02, 0.02, 1.0}, {1.0, 0.7, 0.0, 1.0}, {0.02, 1.0, 0.02, 1.0}}};
        const auto & c = color == state ? on[color] : off;
        msgs::Set(visual.mutable_material()->mutable_ambient(), c);
        msgs::Set(visual.mutable_material()->mutable_diffuse(), c);
        msgs::Set(visual.mutable_material()->mutable_emissive(), c);
        visual_pub_->Publish(visual);
      }
    }
    if (state != last_state_) {
      RCLCPP_INFO(ros_node_->get_logger(), "红绿灯切换：%s", names[state].c_str());
      last_state_ = state;
    }
  }

  physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr ros_node_;
  transport::NodePtr gz_node_;
  transport::PublisherPtr visual_pub_;
  event::ConnectionPtr update_connection_;
  std::array<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr, 2> states_;
  double last_update_{-1.0};
  int last_state_{-1};
};

GZ_REGISTER_WORLD_PLUGIN(TrafficSignalPlugin)
}  // namespace gazebo
