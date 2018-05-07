#include "tricycle_control/TriCycleController.h"
#include "tricycle_control/TriCycleOdometer.h"

using tricycle_control::TriCycleController;

TriCycleController::TriCycleController()
: m_commandBuf()
, m_commandSub()
, m_odometer(nullptr)
{
  ROS_DEBUG_STREAM("Controller::Controller");
}

TriCycleController::~TriCycleController()
{
  ROS_DEBUG_STREAM("Controller::~Controller");
  m_commandSub.shutdown();
}

bool TriCycleController::init(hardware_interface::VelocityJointInterface* _hw,
                              ros::NodeHandle& _rootNh,
                              ros::NodeHandle& _controllerNh)
{
  ROS_DEBUG_STREAM("TriCycleController::init");

  m_odometer = std::make_shared<tricycle_control::TriCycleOdometer>(_hw, _rootNh, _controllerNh);

  // subscribe to receive velocity commands
  m_commandSub = _controllerNh.subscribe("cmd_vel", 1, &TriCycleController::cmdVelCallback, this);

  return true;
}

void TriCycleController::update(const ros::Time& _time, const ros::Duration& _period)
{
  // compute and publish the odometry of the robot
  m_odometer->update(_time, _period);

  // now move the robot
  // Retrieve current velocity command and time step:
  geometry_msgs::Twist currCmd = *(m_commandBuf.readFromRT());

  //////////////////////////////////////////////////////////////////////////////
  // TODO here you should calculate the commands of each joint and set them to
  // the appropriate HI
  //////////////////////////////////////////////////////////////////////////////


}

void TriCycleController::starting(const ros::Time& _time)
{
  ROS_DEBUG_STREAM("Controller::starting");
  // initialize odometry
  m_odometer->init(_time);
}

void TriCycleController::stopping(const ros::Time& time)
{
  //////////////////////////////////////////////////////////////////////////////
  // TODO here you should stop all of the joints
  //////////////////////////////////////////////////////////////////////////////
}

void TriCycleController::cmdVelCallback(const geometry_msgs::Twist& _command)
{
  ROS_DEBUG_STREAM("Controller::cmdVelCallback");

  if (isRunning())
  {
    m_commandBuf.writeFromNonRT(_command);
  }
  else
  {
    ROS_WARN("Can't accept new commands. Controller is not running.");
  }
}
