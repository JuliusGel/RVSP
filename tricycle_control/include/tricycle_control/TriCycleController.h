#ifndef TRICYCLE_CONTROL_TRICYCLECONTROLLER_H
#define TRICYCLE_CONTROL_TRICYCLECONTROLLER_H

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/Twist.h>

namespace tricycle_control
{
// forward declaration of odometry class
class TriCycleOdometer;

/**
 * This class implements the tricycle controller
 */
class TriCycleController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:
  /**
   * Default constructor
   */
  TriCycleController();

  /**
   * Destructor
   */
  virtual ~TriCycleController();

  /**
   * Initialize controller
   *
   * @param _hw            Velocity joint interface for the wheels
   * @param _root_nh       Node handle at root namespace
   * @param _controller_nh Node handle inside the controller namespace
   */
  bool init(hardware_interface::VelocityJointInterface* _hw,
            ros::NodeHandle& _rootNh,
            ros::NodeHandle& _controllerNh) override;

  /**
   * Updates controller, i.e. computes the odometry and sets the new
   * velocity commands
   *
   * @param[in] _time   Current time
   * @param[in] _period Time since the last called to update
   */
  void update(const ros::Time& _time, const ros::Duration& _period) override;

  /**
   * Starts controller
   *
   * @param[in] _time Current time
   */
  void starting(const ros::Time& _time) override;

  /**
   * Stops controller
   *
   * @param[in] _time Current time
   */
  void stopping(const ros::Time& _time) override;

private:
  // buffer storing the latest velocity command
  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> m_commandBuf;
  // the subscription to the velocity command (twist) messages
  ros::Subscriber m_commandSub;

  // the odometer of this robot
  std::shared_ptr<TriCycleOdometer> m_odometer;

  /**
   * Velocity command (twist) callback
   *
   * @param[in] _command new velocity command message (twist)
   */
  void cmdVelCallback(const geometry_msgs::Twist& _command);
};

PLUGINLIB_EXPORT_CLASS(tricycle_control::TriCycleController,
                       controller_interface::ControllerBase);
}  // namespace tricycle_control

#endif  // TRICYCLE_CONTROL_TRICYCLECONTROLLER_H
