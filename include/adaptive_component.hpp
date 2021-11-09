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

  // Constructors
  AdaptiveComponent(const rclcpp::NodeOptions & options)
      : Node("adaptive_component", options) {};

  AdaptiveComponent(
    const rclcpp::NodeOptions & options,
    std::shared_ptr<rclcpp::Node> cpu_node,
    std::shared_ptr<rclcpp::Node> fpga_node,
    std::shared_ptr<rclcpp::Node> gpu_node,
    const int adaptive_value
  )
      : Node("adaptive_component", options), cpu_node_(cpu_node),
        fpga_node_(fpga_node), gpu_node_(gpu_node),
        adaptive_value_(adaptive_value)
  {
    initialize();
  }

  AdaptiveComponent(
    const std::string &node_name,
    const rclcpp::NodeOptions & options,
    std::shared_ptr<rclcpp::Node> cpu_node = nullptr,
    std::shared_ptr<rclcpp::Node> fpga_node = nullptr,
    std::shared_ptr<rclcpp::Node> gpu_node = nullptr,
    const int adaptive_value = 0
  )
      : Node(node_name, options), cpu_node_(cpu_node),
        fpga_node_(fpga_node), gpu_node_(gpu_node),
        adaptive_value_(adaptive_value),
        compute_resources_{cpu_node_, fpga_node_, gpu_node_}
  {
    initialize();
  }

  // Manually add compute resources
  void add_cpu(std::shared_ptr<rclcpp::Node> node_ptr) {cpu_node_ = node_ptr;};
  void add_fpga(std::shared_ptr<rclcpp::Node> node_ptr) {fpga_node_ = node_ptr;};
  void add_gpu(std::shared_ptr<rclcpp::Node> node_ptr) {gpu_node_ = node_ptr;};

protected:
  void on_timer();
  void spin(void);

private:
  void initialize();

  // Internal container's executor
  rclcpp::executors::SingleThreadedExecutor exec_;

  // Computational Nodes across substrates
  // Should be nullptr by default, C++11 spec.
  std::shared_ptr<rclcpp::Node> cpu_node_;
  std::shared_ptr<rclcpp::Node> fpga_node_;
  std::shared_ptr<rclcpp::Node> gpu_node_;

  rclcpp::TimerBase::SharedPtr timer_;
  int adaptive_value_;

  // data structures to simplify interactions
  std::shared_ptr<rclcpp::Node> compute_resources_[3];

  // TODO: reconsider if a more synthetic implementation is used (see cpp)
  // std::string compute_resources_names_[3] = {"CPU", "FPGA", "GPU"};

};

}  // namespace composition

#endif  // ADAPTIVE_COMPONENT_HPP_
