 /**
 *  @file     pid_library.cpp
 *  @brief    ROS2 library to apply a PID control to a system 
 *
 *  @author   Antonio Mauro Galiano
 *  @details  https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */

#include "pid_library/pid_library.h"

PidLibrary::PidLibrary() : Node("ros2_pid_library")
{
  RCLCPP_INFO_STREAM(this->get_logger(), "PID library initialized");

  DeclareParams();
  GetParams();
  PrintParams();
  if (not ValidateParams())
      RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR: Change the parameters");

  paramCallbackHandle_ = this->add_on_set_parameters_callback(
    std::bind(&PidLibrary::ParametersCallback, this, std::placeholders::_1));

  isTheSistemChanged_ = false;

  controlValuePub_ = this->create_publisher<std_msgs::msg::Float32>(controlValueTopic_, 1);
  
  actualStateSub_ = this->create_subscription<std_msgs::msg::Float32>(
      actualStateTopic_, 10, std::bind(&PidLibrary::ActualStateCallback,
      this, std::placeholders::_1));
  setPointSub_ = this->create_subscription<std_msgs::msg::Float32>(
      setPointTopic_, 10, std::bind(&PidLibrary::SetPointCallback,
      this, std::placeholders::_1));  
}

PidLibrary::~PidLibrary()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "PID library killed");
}

void PidLibrary::DeclareParams()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Declaring PID library parameters");

  this->declare_parameter<float>("kp", 0.0);
  this->declare_parameter<float>("ki", 0.0);
  this->declare_parameter<float>("kd", 0.0);
  this->declare_parameter<bool>("use_sample_time", false);
  this->declare_parameter<int>("sample_time", 5);
  this->declare_parameter<bool>("derivative_on_measurement", false);
  this->declare_parameter<bool>("remove_ki_bump", false);
  this->declare_parameter<bool>("reset_windup", false);
  this->declare_parameter<bool>("pid_enabled", true);  
  this->declare_parameter<bool>("make_reverse", false); 
  this->declare_parameter<int>("out_min", 0.0);
  this->declare_parameter<int>("out_max", 0.0);
  this->declare_parameter<std::string>("control_value_topic", "/control_value_topic");
  this->declare_parameter<std::string>("actual_state_topic", "/actual_state_topic");
  this->declare_parameter<std::string>("set_point_topic", "/set_point_topic");

}


void PidLibrary::GetParams()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Getting PID library parameters");

  this->get_parameter("kp", kp_);
  this->get_parameter("ki", ki_);
  this->get_parameter("kd", kd_);
  this->get_parameter("use_sample_time", useSampleTime_);
  this->get_parameter("sample_time", sampleTime_);
  this->get_parameter("derivative_on_measurement", derivativeOnMeasurement_);
  this->get_parameter("remove_ki_bump", removeKiBump_);
  this->get_parameter("reset_windup", resetWindup_);
  this->get_parameter("pid_enabled", pidEnabled_);
  this->get_parameter("make_reverse", makeReverse_);
  this->get_parameter("out_min", outMin_);
  this->get_parameter("out_max", outMax_);
  this->get_parameter("control_value_topic", controlValueTopic_);
  this->get_parameter("actual_state_topic", actualStateTopic_);
  this->get_parameter("set_point_topic", setPointTopic_);
}


void PidLibrary::PrintParams()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Printing PID library parameters");

  RCLCPP_INFO_STREAM(this->get_logger(),"kp = " << kp_);
  RCLCPP_INFO_STREAM(this->get_logger(),"ki = " << ki_);
  RCLCPP_INFO_STREAM(this->get_logger(),"kd = " << kd_);
  RCLCPP_INFO_STREAM(this->get_logger(),"use_sample_time = " << useSampleTime_);
  RCLCPP_INFO_STREAM(this->get_logger(),"sample_time = " << sampleTime_);
  RCLCPP_INFO_STREAM(this->get_logger(),"derivative_on_measurement = " << derivativeOnMeasurement_);
  RCLCPP_INFO_STREAM(this->get_logger(),"reset_windup = " << resetWindup_);
  RCLCPP_INFO_STREAM(this->get_logger(),"pid_enabled = " << pidEnabled_);
  RCLCPP_INFO_STREAM(this->get_logger(),"make_reverse = " << makeReverse_);
  RCLCPP_INFO_STREAM(this->get_logger(),"out_min = " << outMin_);
  RCLCPP_INFO_STREAM(this->get_logger(),"out_max = " << outMax_);
  RCLCPP_INFO_STREAM(this->get_logger(),"control_value_topic = " << controlValueTopic_);
  RCLCPP_INFO_STREAM(this->get_logger(),"actual_state_topic = " << actualStateTopic_);
  RCLCPP_INFO_STREAM(this->get_logger(),"set_point_topic = " << setPointTopic_);
}

bool PidLibrary::ValidateParams()
{
  if (outMin_ > outMax_)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Min output value can't be greater than the max output value!");
    return false;
  }
  if (!((kp_ <= 0.0 && ki_ <= 0.0 && kd_ <= 0.0) || (kp_ >= 0.0 && ki_ >= 0.0 && kd_ >= 0.0)))
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "All three gains should have the same sign ");
    return false;
  }
  return true;
}


rcl_interfaces::msg::SetParametersResult PidLibrary::ParametersCallback(
  const std::vector<rclcpp::Parameter> &parameters)
{ 
  RCLCPP_INFO_STREAM(this->get_logger(), "!!!PARAMETER CHANGED!!!");
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto &param: parameters)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), param.get_name().c_str() <<
      "=" << param.value_to_string().c_str());
    if (param.get_name() == "kp")
    {
      kp_ = param.as_double();
    }
    else if (param.get_name() == "ki")
    {
      ki_ = param.as_double();
    }
    else if (param.get_name() == "kd")
    {
      kd_ = param.as_double();
    }
    else if (param.get_name() == "use_sample_time")
    {
      useSampleTime_ = param.as_bool();
      if (useSampleTime_)
      {
        ki_ = ki_ * sampleTime_;
        kd_ = kd_ / sampleTime_;
      }
    }
    else if (param.get_name() == "sample_time")
    {
      if (param.as_int() != sampleTime_ && param.as_int()>0)
      {
        double ratio  = (double)param.as_int() / sampleTime_;
        ki_ *= ratio;
        kd_ /= ratio;
        sampleTime_ = param.as_int() ;

      }
    }
    else if (param.get_name() == "derivative_on_measurement")
    {
      derivativeOnMeasurement_ = param.as_bool();
    }
    else if (param.get_name() == "remove_ki_bump")
    {
      removeKiBump_ = param.as_bool();
    }
    else if (param.get_name() == "reset_windup")
    {
      resetWindup_ = param.as_bool();
    }
    else if (param.get_name() == "out_min")
    {
      outMin_ = param.as_int();
      if(controlValue_ < outMin_) controlValue_ = outMin_;
      if(errorIntegral_ < outMin_) errorIntegral_ = outMin_;
    }
    else if (param.get_name() == "out_max")
    {
      outMax_ = param.as_int();
      if(controlValue_ > outMax_) controlValue_ = outMax_;
      if(errorIntegral_ > outMax_) errorIntegral_ = outMax_;
    }
    else if (param.get_name() == "pid_enabled")
    {
      pidEnabled_ = param.as_bool();
      if (pidEnabled_)
      {
        oldActualStateValue_ = actualStateValue_;
        errorIntegral_ = controlValue_;
        if(errorIntegral_ > outMax_) errorIntegral_ = outMax_;
        else if(errorIntegral_ < outMin_) errorIntegral_ = outMin_;
      }
    }
    else if (param.get_name() == "make_reverse")
    {
      makeReverse_ = param.as_bool();
      if (makeReverse_)
      {
        kp_ = (0 - kp_);
        ki_ = (0 - ki_);
        kd_ = (0 - kd_);
      }
    }
    if (not ValidateParams())
      RCLCPP_ERROR_STREAM(this->get_logger(), "ERROR: Change the parameters");
  }
  return result;
}

void PidLibrary::ActualStateCallback(const std_msgs::msg::Float32::SharedPtr actualStateMsg)
{
  actualStateValue_ = actualStateMsg->data;
  isTheSistemChanged_ = true;
}

void PidLibrary::SetPointCallback(const std_msgs::msg::Float32::SharedPtr setPointMsg)
{
  setPointValue_ = setPointMsg->data;
  isTheSistemChanged_ = true;
}

float PidLibrary::AlphaFactorCalc(float freq)
{
  if (freq <= 0)
    return 1;
  const float cosine = std::cos(2 * float(M_PI) * freq);
  return cosine - 1 + std::sqrt( (cosine * cosine) - (4 * cosine) + 3 );
}


float PidLibrary::PerformPid()
{
  if (pidEnabled_)
  {
    if (isTheSistemChanged_)
    {
      if (previousTime_.seconds() != 0.0)
      { 
        currentTime_ = this->get_clock()->now();
        deltaTime_ = currentTime_ - previousTime_;
        if (0 == deltaTime_.seconds())
        {
          RCLCPP_ERROR(this->get_logger(), "Delta Time is 0, skipping this loop.");
          return 0;
        }
      }
      else
      {
        previousTime_ = this->get_clock()->now();
        return 0;
      }

      float error = setPointValue_ - actualStateValue_;
      float errorDerivative;
      if(useSampleTime_ && deltaTime_.seconds()>=sampleTime_)
      { 
        if (removeKiBump_)
          errorIntegral_+= ki_ * error;
        else if (!removeKiBump_)
          errorIntegral_+= error;

        if (derivativeOnMeasurement_)
          errorDerivative = (actualStateValue_ - oldActualStateValue_);
        else if (!derivativeOnMeasurement_)
          errorDerivative = (error - oldError_);
      }
      else if (!useSampleTime_)
      {
        if (removeKiBump_)
          errorIntegral_+= ki_ * error;
        else if (!removeKiBump_)
          errorIntegral_+= (error * deltaTime_.seconds());
        
        if (derivativeOnMeasurement_)
          errorDerivative = (actualStateValue_ - oldActualStateValue_) / deltaTime_.seconds();
        else if (!derivativeOnMeasurement_)
          errorDerivative = (error - oldError_) / deltaTime_.seconds();
      }

      if (resetWindup_)
      {
        if(errorIntegral_ > outMax_) errorIntegral_ = outMax_;
        else if(errorIntegral_ < outMin_) errorIntegral_ = outMin_;
      }

      if (derivativeOnMeasurement_)
        controlValue_ = kp_ * error + ki_ * errorIntegral_ - kd_ * errorDerivative;
      else if (!derivativeOnMeasurement_)
        controlValue_ = kp_ * error + ki_ * errorIntegral_ + kd_ * errorDerivative;

      if (resetWindup_)
      {
        if(controlValue_ > outMax_) controlValue_ = outMax_;
        else if(controlValue_ < outMin_) controlValue_ = outMin_;
      }

      oldError_ = error;
      previousTime_ = currentTime_;
      if (derivativeOnMeasurement_)
        oldActualStateValue_ = actualStateValue_;

      RCLCPP_INFO_STREAM(this->get_logger(),"Set Point=" << setPointValue_ << 
        " | Control Value=" << controlValue_ << " | Actual State=" << actualStateValue_);

      controlValueData_.data = controlValue_;
      controlValuePub_->publish(controlValueData_);    
    }
    isTheSistemChanged_ = false; 
  }
  return 0;
}