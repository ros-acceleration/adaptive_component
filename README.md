# `AdaptiveComponent`

Nodes using hardware acceleration are able to perform computations faster relying on FPGAs or GPUs, improving performance. <ins>Adaptive ROS 2 Nodes</ins> leverage hardware acceleration at run-time, allowing robotics engineers to <ins>select which computational resource the Node uses *on-the-go*</ins>, giving roboticists a finer-grained control over the resources their computional graphs use in the underlying hardware.

This ROS 2 package provides a composable container for Adaptive ROS 2 Node computations: [`AdaptiveComponent`](https://github.com/ros-acceleration/adaptive_component/blob/main/src/adaptive_component.cpp). It allows building Nodes that can select between FPGA, CPU or GPU, at run-time.

Technically, it's a ROS 2 `Node`[^1] subclass programmed as a `Component`[^2] and including its own [single threaded executor](https://github.com/ros-acceleration/adaptive_component/blob/main/include/adaptive_component.hpp#L68)
to build adaptive computations. Adaptive ROS 2 `Nodes` can then be built easily and are able to perform computations in the CPU, the FPGA or the GPU. Adaptive behavior is controlled through the `adaptive` ROS 2 parameter, with the following values allowed:

- `0`: Hardware::CPU
- `1`: Hardware::FPGA
- `2`: Hardware::GPU

[^1]: A `Node` is a process that performs computations. ROS 2 is designed to be modular at a fine-grained scale; a robot control system usually comprises many nodes. Nodes execute arbitrary logic that contribute to the overall robotics behavior.

[^2]: A ROS 2 `Component` is a `Node` compiled into a shared library which is then loaded at runtime by a container process. This offers roboticists additional flexibility while building their computational graphs, making the layout process a deploy-time decision. A `Component` is commonly a subclass of `rclcpp::Node`. To maintain flexibility and modularity, `Components` shouldn’t perform any long running or blocking tasks in their constructors. Instead, they can use timers to get periodic notifications and use callbacks for publishers, subscribers, servers, or clients.

## How does it work?

```cpp
// Create an executor
rclcpp::executors::MultiThreadedExecutor exec;

// Create an adaptive ROS 2 Node using "components", the resulting
// Node is also programed as a "component", retaining composability
auto adaptive_node = std::make_shared<composition::AdaptiveComponent>(
      "doublevadd_publisher_adaptive",        // name of the adaptive Node
      options,                                // Node options'
                                              // CPU, FPGA or GPU 'components':
      std::make_shared<composition::DoubleVaddComponent>("_doublevadd_publisher_adaptive_cpu", options),
      std::make_shared<composition::DoubleVaddComponentFPGA>("_doublevadd_publisher_adaptive_fpga", options));

// Add the adaptive Node to an executor
exec.add_node(adaptive_node);
exec.spin();  // spin the executor
```

Then, dynamically, one could switch from CPU to FPGA by setting the `adaptive` parameter in the `/doublevadd_publisher_adaptive` Node:
- To run in the CPU: `ros2 param set /doublevadd_publisher_adaptive adaptive 0`
- To run in the FPGA: `ros2 param set /doublevadd_publisher_adaptive adaptive 1`

## Conventions

### Component-oriented

### File names

### Node names
