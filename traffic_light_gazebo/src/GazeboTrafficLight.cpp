#include "GazeboTrafficLight.hpp"
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <chrono>

namespace gazebo
{
  // Arrange duration of each light just in case you want to change it
  constexpr double GREEN_DURATION = 10.0;
  constexpr double YELLOW_DURATION = 4.0;
  constexpr double RED_DURATION = 10.0;
  constexpr double LIGHT_ON_POSITION = 0.76;
  constexpr double LIGHT_OFF_POSITION = 0.0;

  TrafficLightPlugin::TrafficLightPlugin() 
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char **argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  void TrafficLightPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    model = _model;
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    try {
      std::string node_name = "traffic_light" + std::to_string(model->GetId());
      nh = std::make_shared<rclcpp::Node>(node_name);

      light_pub = nh->create_publisher<std_msgs::msg::String>("traffic_light_state", 10);
      timer = nh->create_wall_timer(std::chrono::seconds(1), std::bind(&TrafficLightPlugin::OnTimer, this));
      green_on_link = model->GetJoint("green_base_1");
      yellow_on_link = model->GetJoint("yellow_base_1");
      red_on_link = model->GetJoint("red_base_1");
      green_on_link_2 = model->GetJoint("green_base_2");
      yellow_on_link_2 = model->GetJoint("yellow_base_2");
      red_on_link_2 = model->GetJoint("red_base_2");

      if (!green_on_link || !yellow_on_link || !red_on_link) {
        RCLCPP_ERROR(nh->get_logger(), "Failed to find one or more joints");
        return;
      }

      current_light = "green";
      last_switch_time = std::chrono::steady_clock::now();
      SetLightState(current_light);

      RCLCPP_INFO(nh->get_logger(), "TrafficLightPlugin Load Complete");
    } catch (const std::exception &e) {
      if (nh) {
        RCLCPP_ERROR(nh->get_logger(), "Exception during Load: %s", e.what());
      } else {
        std::cerr << "Exception during Load: " << e.what() << std::endl;
      }
    } catch (...) {
      if (nh) {
        RCLCPP_ERROR(nh->get_logger(), "Unknown exception during Load");
      } else {
        std::cerr << "Unknown exception during Load" << std::endl;
      }
    }
    executor->add_node(nh);
    update_connection = event::Events::ConnectWorldUpdateBegin(std::bind(&TrafficLightPlugin::Update, this));
  }

  void TrafficLightPlugin::Update()
  {
    executor->spin_some();
  }
  void TrafficLightPlugin::SetLightState(const std::string& light)
  {
      // RCLCPP_INFO(nh->get_logger(), "Setting light state: %s", light.c_str());

      try {
          if (light == "green") {
              // RCLCPP_INFO(nh->get_logger(), "Switching to green light");
              green_on_link->SetPosition(0, LIGHT_ON_POSITION);
              yellow_on_link->SetPosition(0, LIGHT_OFF_POSITION);
              red_on_link->SetPosition(0, LIGHT_OFF_POSITION);
              green_on_link_2->SetPosition(0, LIGHT_ON_POSITION);
              yellow_on_link_2->SetPosition(0, LIGHT_OFF_POSITION);
              red_on_link_2->SetPosition(0, LIGHT_OFF_POSITION);
          } else if (light == "yellow") {
              // RCLCPP_INFO(nh->get_logger(), "Switching to yellow light");
              green_on_link->SetPosition(0, LIGHT_OFF_POSITION);
              yellow_on_link->SetPosition(0, LIGHT_ON_POSITION);
              red_on_link->SetPosition(0, LIGHT_OFF_POSITION);
              green_on_link_2->SetPosition(0, LIGHT_OFF_POSITION);
              yellow_on_link_2->SetPosition(0, LIGHT_ON_POSITION);
              red_on_link_2->SetPosition(0, LIGHT_OFF_POSITION);
          } else if (light == "red") {
              // RCLCPP_INFO(nh->get_logger(), "Switching to red light");
              green_on_link->SetPosition(0, LIGHT_OFF_POSITION);
              yellow_on_link->SetPosition(0, LIGHT_OFF_POSITION);
              red_on_link->SetPosition(0, LIGHT_ON_POSITION);
              green_on_link_2->SetPosition(0, LIGHT_OFF_POSITION);
              yellow_on_link_2->SetPosition(0, LIGHT_OFF_POSITION);
              red_on_link_2->SetPosition(0, LIGHT_ON_POSITION);
          } else {
              RCLCPP_ERROR(nh->get_logger(), "Unknown light state: %s", light.c_str());
          }

          // RCLCPP_INFO(nh->get_logger(), "Publishing light state: %s", light.c_str());
          std_msgs::msg::String msg;
          msg.data = light;
          light_pub->publish(msg);
      } catch (const std::exception& e) {
          RCLCPP_ERROR(nh->get_logger(), "Exception while setting light state: %s", e.what());
      } catch (...) {
          RCLCPP_ERROR(nh->get_logger(), "Unknown exception while setting light state");
      }
  }


  void TrafficLightPlugin::OnTimer()
  {
    auto now = std::chrono::steady_clock::now();
    double elapsed_time = std::chrono::duration<double>(now - last_switch_time).count();

    if (current_light == "green" && elapsed_time >= GREEN_DURATION) {
      current_light = "yellow";
      last_switch_time = now;
    } else if (current_light == "yellow" && elapsed_time >= YELLOW_DURATION) {
      if(is_red_on){
        current_light = "green";
        is_red_on = false;
      }else{
        current_light = "red";
        is_red_on = true;
      }
      last_switch_time = now;
    } else if (current_light == "red" && elapsed_time >= RED_DURATION) {
      current_light = "yellow";
      last_switch_time = now;
    }

    SetLightState(current_light);
  }

  GZ_REGISTER_MODEL_PLUGIN(TrafficLightPlugin)
}
