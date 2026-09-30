/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik LLC
 *  All rights reserved.
 *********************************************************************/

/*      Title     : servo_keyboard_input.cpp
 *      Project   : moveit2_tutorials
 *      Refactor  : Sticky-command model
 *                  - Press a key to start motion
 *                  - SPACE publishes zero velocity command
 *                  - Timer republishes at ~30 Hz
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>

#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

// Define used keys
#define KEYCODE_RIGHT     0x43
#define KEYCODE_LEFT      0x44
#define KEYCODE_UP        0x41
#define KEYCODE_DOWN      0x42
#define KEYCODE_PERIOD    0x2E
#define KEYCODE_SEMICOLON 0x3B
#define KEYCODE_SPACE     0x20
#define KEYCODE_1         0x31
#define KEYCODE_2         0x32
#define KEYCODE_3         0x33
#define KEYCODE_4         0x34
#define KEYCODE_5         0x35
#define KEYCODE_6         0x36
#define KEYCODE_Q         0x71
#define KEYCODE_W         0x77
#define KEYCODE_E         0x65
#define KEYCODE_R         0x72

// Topic and frame constants
const std::string TWIST_TOPIC    = "/delta_twist_cmds";
const std::string JOINT_TOPIC    = "/delta_joint_cmds";
const size_t ROS_QUEUE_SIZE      = 10;
const std::string EEF_FRAME_ID   = "tool0";
const std::string BASE_FRAME_ID  = "base_link";

// Publish rate of the sticky-command timer.
constexpr int PUBLISH_PERIOD_MS  = 33;   // ~30 Hz

class KeyboardReader
{
public:
  KeyboardReader() : kfd(0)
  {
    tcgetattr(kfd, &cooked);

    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));

    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;

    tcsetattr(kfd, TCSANOW, &raw);
  }

  void readOne(char* c)
  {
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
  }

  void shutdown()
  {
    tcsetattr(kfd, TCSANOW, &cooked);
  }

private:
  int kfd;
  struct termios cooked;
};

class KeyboardServo
{
public:
  KeyboardServo();
  int keyLoop();

private:
  enum class Mode
  {
    STOPPED,
    TWIST,
    JOINT
  };

  void spin();
  void publishTick();

  void publishTwistCommand(const geometry_msgs::msg::Twist& twist);
  void publishJointCommand(const std::string& joint_name, double velocity);
  void publishZeroCommand();

  rclcpp::Node::SharedPtr nh_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mtx_;

  Mode mode_ = Mode::STOPPED;

  // Remember last command type, so SPACE can send zero to the correct command topic.
  Mode last_motion_mode_ = Mode::TWIST;

  geometry_msgs::msg::Twist current_twist_;
  std::string current_joint_name_;

  std::string twist_frame_ = BASE_FRAME_ID;

  double joint_vel_cmd_ = 1.0;
};

KeyboardServo::KeyboardServo()
{
  nh_ = rclcpp::Node::make_shared("servo_keyboard_input");

  twist_pub_ = nh_->create_publisher<geometry_msgs::msg::TwistStamped>(
      TWIST_TOPIC, ROS_QUEUE_SIZE);

  joint_pub_ = nh_->create_publisher<control_msgs::msg::JointJog>(
      JOINT_TOPIC, ROS_QUEUE_SIZE);

  timer_ = nh_->create_wall_timer(
      std::chrono::milliseconds(PUBLISH_PERIOD_MS),
      std::bind(&KeyboardServo::publishTick, this));
}

void KeyboardServo::publishTwistCommand(const geometry_msgs::msg::Twist& twist)
{
  auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();

  msg->header.stamp = nh_->now();
  msg->header.frame_id = twist_frame_;
  msg->twist = twist;

  twist_pub_->publish(std::move(msg));
}

void KeyboardServo::publishJointCommand(const std::string& joint_name, double velocity)
{
  if (joint_name.empty())
  {
    return;
  }

  auto msg = std::make_unique<control_msgs::msg::JointJog>();

  msg->header.stamp = nh_->now();
  msg->header.frame_id = BASE_FRAME_ID;
  msg->joint_names.push_back(joint_name);
  msg->velocities.push_back(velocity);

  joint_pub_->publish(std::move(msg));
}

void KeyboardServo::publishZeroCommand()
{
  if (last_motion_mode_ == Mode::JOINT)
  {
    publishJointCommand(current_joint_name_, 0.0);
    return;
  }

  geometry_msgs::msg::Twist zero_twist;
  publishTwistCommand(zero_twist);
}

void KeyboardServo::publishTick()
{
  std::lock_guard<std::mutex> lock(mtx_);

  if (mode_ == Mode::STOPPED)
  {
    publishZeroCommand();
    return;
  }

  if (mode_ == Mode::TWIST)
  {
    publishTwistCommand(current_twist_);
    return;
  }

  if (mode_ == Mode::JOINT)
  {
    publishJointCommand(current_joint_name_, joint_vel_cmd_);
    return;
  }
}

KeyboardReader input;

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  KeyboardServo keyboard_servo;

  signal(SIGINT, quit);

  int rc = keyboard_servo.keyLoop();

  input.shutdown();
  rclcpp::shutdown();

  return rc;
}

void KeyboardServo::spin()
{
  while (rclcpp::ok())
  {
    rclcpp::spin_some(nh_);
  }
}

int KeyboardServo::keyLoop()
{
  char c;

  std::thread{ std::bind(&KeyboardServo::spin, this) }.detach();

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Cartesian TWIST:");
  puts("  Up/Down arrows = +x / -x");
  puts("  Left/Right     = -y / +y");
  puts("  .  /  ;        = -z / +z");
  puts("  W = base_link frame");
  puts("  E = tool0 frame");
  puts("");
  puts("Joint jog:");
  puts("  1..6 = select joint");
  puts("  R    = reverse joint direction");
  puts("");
  puts("SPACE = publish zero velocity");
  puts("Q     = quit");
  puts("---------------------------");

  for (;;)
  {
    try
    {
      input.readOne(&c);
    }
    catch (const std::runtime_error&)
    {
      perror("read():");
      return -1;
    }

    RCLCPP_DEBUG(nh_->get_logger(), "value: 0x%02X", c);

    std::lock_guard<std::mutex> lock(mtx_);

    geometry_msgs::msg::Twist new_twist;

    switch (c)
    {
      case KEYCODE_LEFT:
        new_twist.linear.y = -1.0;
        current_twist_ = new_twist;
        mode_ = Mode::TWIST;
        last_motion_mode_ = Mode::TWIST;
        break;

      case KEYCODE_RIGHT:
        new_twist.linear.y = 1.0;
        current_twist_ = new_twist;
        mode_ = Mode::TWIST;
        last_motion_mode_ = Mode::TWIST;
        break;

      case KEYCODE_UP:
        new_twist.linear.x = 1.0;
        current_twist_ = new_twist;
        mode_ = Mode::TWIST;
        last_motion_mode_ = Mode::TWIST;
        break;

      case KEYCODE_DOWN:
        new_twist.linear.x = -1.0;
        current_twist_ = new_twist;
        mode_ = Mode::TWIST;
        last_motion_mode_ = Mode::TWIST;
        break;

      case KEYCODE_PERIOD:
        new_twist.linear.z = -1.0;
        current_twist_ = new_twist;
        mode_ = Mode::TWIST;
        last_motion_mode_ = Mode::TWIST;
        break;

      case KEYCODE_SEMICOLON:
        new_twist.linear.z = 1.0;
        current_twist_ = new_twist;
        mode_ = Mode::TWIST;
        last_motion_mode_ = Mode::TWIST;
        break;

      case KEYCODE_E:
        twist_frame_ = EEF_FRAME_ID;
        RCLCPP_INFO(nh_->get_logger(), "Twist frame: %s", EEF_FRAME_ID.c_str());
        break;

      case KEYCODE_W:
        twist_frame_ = BASE_FRAME_ID;
        RCLCPP_INFO(nh_->get_logger(), "Twist frame: %s", BASE_FRAME_ID.c_str());
        break;

      case KEYCODE_1:
        current_joint_name_ = "shoulder_pan_joint";
        mode_ = Mode::JOINT;
        last_motion_mode_ = Mode::JOINT;
        break;

      case KEYCODE_2:
        current_joint_name_ = "shoulder_lift_joint";
        mode_ = Mode::JOINT;
        last_motion_mode_ = Mode::JOINT;
        break;

      case KEYCODE_3:
        current_joint_name_ = "elbow_joint";
        mode_ = Mode::JOINT;
        last_motion_mode_ = Mode::JOINT;
        break;

      case KEYCODE_4:
        current_joint_name_ = "wrist_1_joint";
        mode_ = Mode::JOINT;
        last_motion_mode_ = Mode::JOINT;
        break;

      case KEYCODE_5:
        current_joint_name_ = "wrist_2_joint";
        mode_ = Mode::JOINT;
        last_motion_mode_ = Mode::JOINT;
        break;

      case KEYCODE_6:
        current_joint_name_ = "wrist_3_joint";
        mode_ = Mode::JOINT;
        last_motion_mode_ = Mode::JOINT;
        break;

      case KEYCODE_R:
        joint_vel_cmd_ *= -1.0;
        RCLCPP_INFO(nh_->get_logger(), "Joint velocity command: %.3f", joint_vel_cmd_);
        break;

      case KEYCODE_SPACE:
        current_twist_ = geometry_msgs::msg::Twist();
        mode_ = Mode::STOPPED;
        RCLCPP_INFO(nh_->get_logger(), "Stop: publishing zero velocity command");
        break;

      case KEYCODE_Q:
        current_twist_ = geometry_msgs::msg::Twist();
        mode_ = Mode::STOPPED;
        publishZeroCommand();
        return 0;

      default:
        break;
    }
  }

  return 0;
}