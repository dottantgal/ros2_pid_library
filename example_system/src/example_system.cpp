 /**
 *  @file     example_system.cpp
 *  @brief    An example system where to apply the PID control library
 *            based on Andrew J Zelenak plant example
 *
 *  @author   Antonio Mauro Galiano
 *  @details  https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"


class ExampleSystem : public rclcpp::Node
{
public:
  ExampleSystem() : Node("pid_example_system")
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Echo line from pid_example_system");
      this->declare_parameter<int>("plant_order", 1);

      controlValueSub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/control_value_topic", 10, std::bind(&ExampleSystem::TopicCallback,
        this, std::placeholders::_1)); 

      actualStatePub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/actual_state_topic", 10); 
      
      actualStatePub_->publish(plantState_);
    }
  std_msgs::msg::Float32 plantState_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr actualStatePub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr controlValueSub_;

private:
  int plantOrder_ = 1;
  float temp_ = 4.7;          // initial condition for first-order plant
  float displacement_ = 3.3;  // initial condition for second-order plant

  int loopCounter_ = 0;
  float deltaT_ = 0.01;

  // Initialize 1st-order (e.g temp controller) process variables
  float tempRate_ = 0.0;      // rate of temp change

  // Initialize 2nd-order (e.g. servo-motor with load) process variables
  float speed_ = 0.0;         // meters/sec
  float acceleration_ = 0.0;  // meters/sec^2
  float mass_ = 0.1;          // in kg
  float friction_ = 1.0;      // a decelerating force factor
  float stiction_ = 1.0;      // control_effort must exceed this before stationary servo moves
  float kV_ = 1.0;            // motor constant: force (newtons) / volt
  float kBackemf_ = 0.0;      // Volts of back-emf per meter/sec of speed
  float decelForce_;          // decelerating force
  float controlEffort_;

  void TopicCallback(const std_msgs::msg::Float32::SharedPtr msg);
};


void ExampleSystem::TopicCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  controlEffort_ = msg->data;
  this->get_parameter("plant_order", plantOrder_);
  switch (plantOrder_)
  {
    case 1:  // First order plant
      tempRate_ = (0.1 * temp_) + controlEffort_;
      temp_ = temp_ + tempRate_ * deltaT_;

      plantState_.data = temp_;
      break;

    case 2:  // Second order plant
      if (fabs(speed_) < 0.001)
      {
        // if nearly stopped, stop it & require overcoming stiction to restart
        speed_ = 0;
        if (fabs(controlEffort_) < stiction_)
        {
          controlEffort_ = 0;
        }
      }

      decelForce_ = -(speed_ * friction_);  // can be +ve or -ve. Linear with speed
      acceleration_ = ((kV_ * (controlEffort_ - (kBackemf_ * speed_)) + decelForce_) / mass_);  // a = F/m
      speed_ = speed_ + (acceleration_ * deltaT_);
      displacement_ = displacement_ + speed_ * deltaT_;

      plantState_.data = displacement_;
      break;

    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid plant_order");
  }
  actualStatePub_->publish(plantState_);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleSystem>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}