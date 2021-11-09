# `composition::AdaptiveComponent`

**A composable container for Adaptive ROS 2 Node computations. Select between FPGA, CPU or GPU at run-time**.

Nodes using hardware acceleration are able to perform computations faster relying on FPGAs or GPUs, improving performance. <ins>Adaptive ROS 2 Nodes</ins> leverage hardware acceleration at run-time, allowing robotics engineers to <ins>select which computational resource the Node uses *on-the-go*</ins>, giving roboticists a finer-grained control over the resources their computional graphs use in the underlying hardware.

This ROS 2 package provides a composable container for Adaptive ROS 2 Node computations: [`composition::AdaptiveComponent`](https://github.com/ros-acceleration/adaptive_component/blob/main/src/adaptive_component.cpp). It allows building Nodes that can select between FPGA, CPU or GPU, at run-time.

Technically, it's a ROS 2 `Node`[^1] subclass programmed as a `Component`[^2] and including its own [single threaded executor](https://github.com/ros-acceleration/adaptive_component/blob/main/include/adaptive_component.hpp#L68)
to build adaptive computations. Adaptive ROS 2 `Nodes` can then be built easily and are able to perform computations in the CPU, the FPGA or the GPU. Adaptive behavior is controlled through the `adaptive` ROS 2 parameter, with the following values allowed:

- `0`: Hardware::CPU
- `1`: Hardware::FPGA
- `2`: Hardware::GPU


[^1]: A `Node` is a process that performs computations. ROS 2 is designed to be modular at a fine-grained scale; a robot control system usually comprises many nodes. Nodes execute arbitrary logic that contribute to the overall robotics behavior.

[^2]: A ROS 2 `Component` is a `Node` compiled into a shared library which is then loaded at runtime by a container process. This offers roboticists additional flexibility while building their computational graphs, making the layout process a deploy-time decision. A `Component` is commonly a subclass of `rclcpp::Node`. To maintain flexibility and modularity, `Components` shouldn’t perform any long running or blocking tasks in their constructors. Instead, they can use timers to get periodic notifications and use callbacks for publishers, subscribers, servers, or clients.

## Why should I care as a ROS package maintainer?

The integration of hardware acceleration into ROS often requires rewriting parts Node computations to further exploit parallelism. These changes often conflict with CPU-centric architectures.

To <ins>**avoid unnecessary forks** and discourage package fragmentation</ins>, `composition::AdaptiveComponent` allows to extend ROS 2 CPU-centric Nodes[^3] with their computational counterparts. From a package-maintenance perspective, each Node (across computation options) is written in a separated file. This facilitates maintaining implementations across different substrates, avoiding versioning issues and functionality issues due to fragmentation.

From an execution perspective, developers can easily create Adaptive ROS 2 Nodes and compose them together as desired at launch-time, with capabilities to adaptively switch between compute alternatives at run-time.

[^3]: Assumes Nodes are written as components, but it they are not, it's a great chance to do so ;).

## How does it work?

```cpp
using NodeCPU = composition::DoubleVaddComponent;
using NodeFPGA = composition::DoubleVaddComponentFPGA;

rclcpp::NodeOptions options;

// Create an executor
rclcpp::executors::MultiThreadedExecutor exec;

// Create an adaptive ROS 2 Node using "components", the resulting
// Node is also programed as a "component", retaining composability
auto adaptive_node = std::make_shared<composition::AdaptiveComponent>(
      "doublevadd_publisher_adaptive",        // name of the adaptive Node
      options,                                // Node options
                                              // CPU
      std::make_shared<NodeCPU>("_doublevadd_publisher_adaptive_cpu", options),
                                              // FPGA
      std::make_shared<NodeFPGA>("_doublevadd_publisher_adaptive_fpga", options),
                                              // GPU
      nullptr);

exec.add_node(adaptive_node);  // fill up the executor
exec.spin();  // spin the executor
```

Then, dynamically, one could switch from CPU to FPGA by setting the `adaptive` parameter in the `/doublevadd_publisher_adaptive` Node:
- To run in the CPU: `ros2 param set /doublevadd_publisher_adaptive adaptive 0`
- To run in the FPGA: `ros2 param set /doublevadd_publisher_adaptive adaptive 1`

[![asciicast](https://asciinema.org/a/448016.svg)](https://asciinema.org/a/448016)

The source code of the examples showcased:
- [DoubleVaddComponent](https://github.com/ros-acceleration/acceleration_examples/blob/main/doublevadd_publisher/src/doublevadd_component.cpp) (CPU-based Node)
- [DoubleVaddComponentFPGA](https://github.com/ros-acceleration/acceleration_examples/blob/main/doublevadd_publisher/src/doublevadd_component_fpga.cpp) (FPGA-based Node)


## Conventions and recommendations

The following conventions and recommendations are meant to facilitate the integration of hardware acceleration in existing ROS packages

1. **Component-oriented**: [`AdaptiveComponent`](https://github.com/ros-acceleration/adaptive_component/blob/main/src/adaptive_component.cpp) is built as a component and should be used as such to maintain composability of Nodes.
2. **Naming**: The Adaptive Node should be suffix with `_adaptive` to identify in the computational graph which Nodes have adaptive capabilities and which do not.
3. **Hidden sub-Nodes**: Adaptive Node components (compute-specific ones e.g. CPU's or FPGA's) should be named with a hyphen (`_`) as a prefix, which will make them *hidden* Nodes by default.
4. **File names**: When possible, source code file names should adhere to the following guidelines:
   - CPU-based computational Nodes can <ins>optionally</ins> add the `_cpu` suffix
   - FPGA-based computational Nodes shall add the `_fpga` suffix
   - GPU-based computational Nodes shall add the `_gpu` suffix

## Quality Declaration

This package claims to be in the **Quality Level 4** category, see the [Quality Declaration](./QUALITY_DECLARATION.md) for more details.
