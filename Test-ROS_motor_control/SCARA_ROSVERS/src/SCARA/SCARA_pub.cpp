#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "SCARA/SCARA_pub.h"

using namespace std::chrono_literals;
#define MOTOR_ID 3

/**
 * @brief Construct a new Read Write Node 2:: Read Write Node 2 object
 * @note Initial timer for publisher and client, regularly activate the function
 */
ReadWriteNode2::ReadWriteNode2()
: Node("read_write_node"), count_(0)
{
  RCLCPP_INFO(this->get_logger(), "Run read write node");

  // Create publisher
  set_position_publisher_ =
    this->create_publisher<SetPosition>("set_position", 10);
  
  // Publish Timer, 1s freq
  publish_timer = this->create_wall_timer(
      1000ms, std::bind(&ReadWriteNode2::publish_callback, this));

  // Create client
  get_position_client_ = create_client<GetPosition>("get_position");

  // Wait until server is initiated
  while (!get_position_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      exit(0);
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  // Client Timer, 0.5s freq
  client_timer = this->create_wall_timer(
      5000ms, std::bind(&ReadWriteNode2::client_callback, this));
}


/**
 * @brief Publisher call back
 * @note Used to publish motor command to topic
 */
void ReadWriteNode2::publish_callback()
{
  SetPosition msg;
  msg.id = MOTOR_ID;
  count_++;
  msg.position = 100*(count_%2);
  RCLCPP_INFO(this->get_logger(), "Publishing: Motor %d to position %d", msg.id, msg.position);
  set_position_publisher_->publish(msg);
}

/**
 * @brief Client callback
 * @note Sent request to get motor position, and display using response
 */
void ReadWriteNode2::client_callback()
{
  auto request = std::make_shared<GetPosition::Request>();
  
  request->id = MOTOR_ID;

  // We give the async_send_request() method a callback that will get executed once the response
  // is received.
  // This way we can return immediately from this method and allow other work to be done by the
  // executor in `spin` while waiting for the response.
  using ServiceResponseFuture =rclcpp::Client<GetPosition>::SharedFuture;

  auto response_received_callback = [this](ServiceResponseFuture future) {
    auto result = future.get();
    RCLCPP_INFO(this->get_logger(), "Response: Motor to position %d", result->position);
  };

  auto response = get_position_client_->async_send_request(request, response_received_callback);
}

ReadWriteNode2::~ReadWriteNode2()
{
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReadWriteNode2>());
  rclcpp::shutdown();
  return 0;
}