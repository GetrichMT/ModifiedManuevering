#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class TrolleyInit : public rclcpp::Node
{
public:
  TrolleyInit()
      : Node("trolley_init"), count_(0)
  {
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 5);
    pub_ = this->create_publisher<std_msgs::msg::Int32>("pos_ang_dock", 5);
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("lidar_rear_left/scan", rclcpp::SensorDataQoS(), std::bind(&TrolleyInit::scanCallback, this, _1)); 
    //subscriber2_ = this->create_subscription<sensor_msgs::msg::LaserScan>("lidar_front_right/scan", rclcpp::SensorDataQoS(), std::bind(&TrolleyInit::scanCallback2, this, _1));
    sub_ = this->create_subscription<std_msgs::msg::Int32>("pos_ang_dock", 5, std::bind(&TrolleyInit::posisi_callback, this, _1));
  }

private:
  void posisi_callback(const std_msgs::msg::Int32::SharedPtr pos_msg)
  {
    msg = pos_msg->data;
  }
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    float smalldist = 100;
    float smalldist2 = 100;
    float left_front = 100;
    float right_back = 100;
    float right_front = 100;
    
    if(stateman == 0 )
    {
      RCLCPP_INFO(this->get_logger(), "Manuever Running...");
      stateman = 1;
    }

    RCLCPP_INFO(this->get_logger(), "waiting for topic /pos_ang_dock:%d", msg);
    if (msg == 3) // move angular
    {
      for (int i = 20; i < 90; i++)
      {
        if (scan_msg->ranges[i] < left_front)
        {
          left_front = scan_msg->ranges[i];
        }
      }

      for (int m = 590; m < 633; m++)
      {
        if (scan_msg->ranges[m] < right_front)
        {
          right_front = scan_msg->ranges[m];
        }
      }
      //RCLCPP_INFO(this->get_logger(), "trolley left leg distance(m):%lf", left_front);
      //
      (this->get_logger(), "trolley right leg distance(m):%lf", right_front);
      cmd_pub_->publish(vel_msg_);
      if (left_front > 0.93 && right_front < 0.648)
      {
        vel_msg_.angular.z = 0.2; // kiri
      }
      else if (left_front < 0.93 && right_front > 0.648)
      {
        vel_msg_.angular.z = -0.2; // kanan
      }
      else if (left_front > 0.93 && right_front > 0.648)
      {
        vel_msg_.angular.z = -0.2; // kanan
      }
      else if (left_front < 0.93 && right_front < 0.648)
      {
        vel_msg_.angular.z = 0;
        if (state == 1)
        {
          msg_.data = 1;
          pub_->publish(msg_);
          state = 2;
        }
      }
    }
    if (msg == 4) // move forward
    {
      for (int i = 270; i < 360; i++)
      {
        if (scan_msg->ranges[i] < right_back)
        {
          right_back = scan_msg->ranges[i];
        }
      }

      for (int m = 360; m < 460; m++)
      {
        if (scan_msg->ranges[m] < right_front)
        {
          right_front = scan_msg->ranges[m];
        }
      }
      //RCLCPP_INFO(this->get_logger(), "Trolley right back leftback:%lf", right_back);
      //RCLCPP_INFO(this->get_logger(), "Trolley right front distance(m):%lf", right_front);
      cmd_pub_->publish(vel_msg_);
      if (right_back > 0.73 && right_front > 0.3 || right_back < 0.73 && right_front > 0.3 || right_back > 0.73 && right_front < 0.3 || right_back < 0.2 && right_front < 0.2)
      {
        vel_msg_.linear.x = 0.05;
      }
      else if (right_back > 0.660 && right_back < 0.68 && right_front > 0.1 && right_front < 0.3)
      {
        vel_msg_.linear.x = 0.0;
        if (state == 1)
        {
          msg_.data = 5;
          pub_->publish(msg_);
          state = 2;
        }
        if (msg == 2)
        {
          rclcpp::shutdown();
        }
      }
    }
    if (msg == 6) // move back
    {
      for (int i = 0; i < 860; i++)
      {
        if (scan_msg->ranges[i] < smalldist)
        {
          smalldist = scan_msg->ranges[i];
        }
      }

      //RCLCPP_INFO(this->get_logger(), "pillar x distance(m):%lf", smalldist);
      //RCLCPP_INFO(this->get_logger(), "pillar x distance(m):%lf", smalldist2);
      cmd_pub_->publish(vel_msg_);
      if (smalldist < 0.9)
      {
        vel_msg_.linear.x = -0.1;
      }
      else if (smalldist > 1.0)
      {
        vel_msg_.linear.x = 0;
        if (state == 1)
        {
          msg_.data = 7;
          pub_->publish(msg_);
          state = 2;
        }
        if (msg == 2)
        {
          rclcpp::shutdown();
        }
      }
    }
    if (msg == 2)
    {
      rclcpp::shutdown();
    }
  }
  int state = 1;
  int stateman = 0;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_; // ini buat publisher gerak
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber2_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  sensor_msgs::msg::LaserScan::SharedPtr laser1_;
  int msg;
  std_msgs::msg::Int32 msg_;
  geometry_msgs::msg::Twist vel_msg_; // assign variable buat vel_msg_
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrolleyInit>());
  rclcpp::shutdown();
  return 0;
}
