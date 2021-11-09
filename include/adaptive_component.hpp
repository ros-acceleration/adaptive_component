/*
      ____  ____
     /   /\/   /
    /___/  \  /   Copyright (c) 2021, Xilinx®.
    \   \   \/    Author: Víctor Mayoral Vilches <victorma@xilinx.com>
     \   \
     /   /
    /___/   /\
    \   \  /  \
     \___\/\___\

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#ifndef ADAPTIVE_COMPONENT_HPP_
#define ADAPTIVE_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"

namespace composition
{

class AdaptiveComponent : public rclcpp::Node
{
public:
  // Define within the class to avoid polluting the global namespace
  enum Hardware {
    CPU  = 0,
    FPGA = 1,
    GPU  = 2,
  };

  explicit AdaptiveComponent(const rclcpp::NodeOptions & options): Node("adaptive_component", options) {};

  explicit AdaptiveComponent(const rclcpp::NodeOptions & options,
                              std::shared_ptr<rclcpp::Node> cpu_node,
                              std::shared_ptr<rclcpp::Node> fpga_node,
                              std::shared_ptr<rclcpp::Node> gpu_node);

  explicit AdaptiveComponent(const std::string &node_name,
                              const rclcpp::NodeOptions & options,
                              std::shared_ptr<rclcpp::Node> cpu_node,
                              std::shared_ptr<rclcpp::Node> fpga_node,
                              std::shared_ptr<rclcpp::Node> gpu_node);

  void add_cpu(std::shared_ptr<rclcpp::Node> node_ptr) {cpu_node_ = node_ptr;};
  void add_fpga(std::shared_ptr<rclcpp::Node> node_ptr) {fpga_node_ = node_ptr;};
  void add_gpu(std::shared_ptr<rclcpp::Node> node_ptr) {gpu_node_ = node_ptr;};

protected:
  void on_timer();
  void spin(void);

private:
  void initialize();

  rclcpp::executors::SingleThreadedExecutor exec_;

  // Should be nullptr by default, C++11 spec.
  std::shared_ptr<rclcpp::Node> cpu_node_;
  std::shared_ptr<rclcpp::Node> fpga_node_;
  std::shared_ptr<rclcpp::Node> gpu_node_;

  rclcpp::TimerBase::SharedPtr timer_;
  int adaptive_value_;
};

}  // namespace composition

#endif  // ADAPTIVE_COMPONENT_HPP_
