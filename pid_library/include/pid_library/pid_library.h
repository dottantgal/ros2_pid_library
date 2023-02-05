 /**
 *  @file     pid_library.h
 *  @brief    header file
 *
 *  @author   Antonio Mauro Galiano
 *  @details  https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */

#ifndef PID_LIBRARY_H
#define PID_LIBRARY_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <chrono>


using namespace std::chrono_literals;


class PidLibrary : public rclcpp::Node
{
public:
  PidLibrary();
  ~PidLibrary();
  void UsePidLibrary();

protected:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr controlValuePub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr actualStateSub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr setPointSub_;

  rclcpp::Time currentTime_;
  rclcpp::Time previousTime_;
  rclcpp::Duration deltaTime_ = rclcpp::Duration(0ns);
  
  float kp_;
  float ki_;
  float kd_;
  float actualStateValue_;
  float errorIntegral_;
  float setPointValue_;
  float controlValue_ = 0;
  float oldActualStateValue_;
  float oldError_;
  float oldErrorDerivativeFiltered_;
  float oldErrorIntegral_;
  int sampleTime_;
  int outMin_;
  int outMax_;
  bool useSampleTime_;
  bool derivativeOnMeasurement_;
  bool removeKiBump_;
  bool resetWindup_;
  bool pidEnabled_;
  bool isTheSistemChanged_;
  bool makeReverse_;
  std::string controlValueTopic_;
  std::string actualStateTopic_;
  std::string setPointTopic_;
  
  std_msgs::msg::Float32 controlValueData_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr paramCallbackHandle_;

  void ActualStateCallback(const std_msgs::msg::Float32::SharedPtr actualStateMsg);
  void SetPointCallback(const std_msgs::msg::Float32::SharedPtr setPointMsg);
  rcl_interfaces::msg::SetParametersResult ParametersCallback(
    const std::vector<rclcpp::Parameter> &parameters);
  
  void DeclareParams();
  void GetParams();
  void PrintParams();
  void PerformPid();
  bool ValidateParams();
  float AlphaFactorCalc(float freq);
};

#endif