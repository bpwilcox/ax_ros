// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "ax_ros_interfaces/srv/run_ax_trial.hpp"
#include "std_msgs/msg/float64.hpp"

using ax_ros_interfaces::srv::RunAxTrial;
using std_msgs::msg::Float64;
using namespace std::chrono_literals;
using namespace std::placeholders;

class SimpleAxServer : public rclcpp::Node
{
public:
  SimpleAxServer()
  : Node("simple_ax_server_cpp")
  {
    ax_trial_srv = create_service<RunAxTrial>("simple_ax_server_cpp/trial_request",
        std::bind(&SimpleAxServer::ax_metric_callback, this, _1, _2, _3));

    // subA = create_subscription<Float64>("x",
    //   rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    //   [&](Float64::SharedPtr msg) {x_ = msg->data;});
    // subB = create_subscription<Float64>("y",
    //   rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    //   [&](Float64::SharedPtr msg) {y_ = msg->data;});
  
    client_node_ = std::make_shared<rclcpp::Node>("server_client_node");
    // thread_ = std::make_unique<std::thread>(
    //   [&]()
    //   {
    //     executor_.add_node(client_node_);
    //     executor_.spin();
    //     executor_.remove_node(client_node_);
    //   });
    param_client_A_ = std::make_shared<rclcpp::SyncParametersClient>(client_node_, "nodeA");
    param_client_B_ = std::make_shared<rclcpp::SyncParametersClient>(client_node_, "nodeB");

  }

  ~SimpleAxServer()
  {
    executor_.cancel();
    thread_->join();
  }

  void ax_metric_callback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<RunAxTrial::Request> /*request*/,
    std::shared_ptr<RunAxTrial::Response> response)
  {
    // std::lock_guard<std::mutex> lock(mutex_);
    x_ = param_client_A_->get_parameter<double>("x");
    y_ = param_client_B_->get_parameter<double>("y");
  
    RCLCPP_INFO(get_logger(), "Incoming Ax trial with test parameters: x: %f, y: %f", x_, y_);
    response->result.push_back(x_ * x_ + y_ * y_) ;
    RCLCPP_INFO(get_logger(), "Sending Ax trial result: %f", response->result[0]);
  }

protected:
  rclcpp::Service<RunAxTrial>::SharedPtr ax_trial_srv;
  // rclcpp::Subscription<Float64>::SharedPtr subA;
  // rclcpp::Subscription<Float64>::SharedPtr subB;
  double x_{0.0};
  double y_{0.0};
  // std::mutex mutex_;

  rclcpp::Node::SharedPtr client_node_;
  std::unique_ptr<std::thread> thread_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<rclcpp::SyncParametersClient> param_client_A_;
  std::shared_ptr<rclcpp::SyncParametersClient> param_client_B_;
};

class SimpleNode : public rclcpp::Node
{
public:
  SimpleNode(const std::string & node_name, const std::string & parameter_name)
  : Node(node_name),
    param_name_(parameter_name)
  {
    declare_parameter(parameter_name, 0.0);
    // publisher_ = create_publisher<Float64>(parameter_name,
    //   rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    // timer_ = create_wall_timer(200ms, std::bind(&SimpleNode::timer_callback, this));
  }

protected:
  void timer_callback()
  {
    auto msg = Float64();
    auto parameter = get_parameter(param_name_);
    msg.data = parameter.get_value<double>();
    publisher_->publish(msg);
  }

  std::string param_name_;
  rclcpp::Publisher<Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto ax_server_node = std::make_shared<SimpleAxServer>();
  auto nodeA = std::make_shared<SimpleNode>("nodeA", "x");
  auto nodeB = std::make_shared<SimpleNode>("nodeB", "y");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(ax_server_node->get_node_base_interface());
  executor.add_node(nodeA->get_node_base_interface());
  executor.add_node(nodeB->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}