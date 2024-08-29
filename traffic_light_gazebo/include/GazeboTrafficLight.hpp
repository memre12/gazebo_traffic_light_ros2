#ifndef GAZEBOTRAFFICLIGHT_CPP
#define GAZEBOTRAFFICLIGHT_CPP

#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <map>
#include <chrono>

namespace gazebo
{
  class TrafficLightPlugin : public ModelPlugin
  {
    public:
      TrafficLightPlugin();
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      virtual void Update();

    private:
      void SetLightState(const std::string& light);
      void OnTimer();

      physics::ModelPtr model;
      rclcpp::Node::SharedPtr nh;
      std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
      event::ConnectionPtr update_connection;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr light_pub;
      rclcpp::TimerBase::SharedPtr timer;
      std::string current_light;
      std::map<std::string, double> light_durations;
      std::chrono::time_point<std::chrono::steady_clock> last_switch_time;

      bool is_red_on;
      bool is_green_on;
      
      physics::JointPtr green_on_link;
      physics::JointPtr yellow_on_link;
      physics::JointPtr red_on_link;
      physics::JointPtr green_on_link_2;
      physics::JointPtr yellow_on_link_2;
      physics::JointPtr red_on_link_2;
  };
}

#endif // GAZEBOTRAFFICLIGHT_CPP
