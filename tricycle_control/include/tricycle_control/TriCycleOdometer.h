#ifndef TRICYCLE_CONTROL_TRICYCLEODOMETER_H
#define TRICYCLE_CONTROL_TRICYCLEODOMETER_H

#include "ros/ros.h"
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

namespace tricycle_control
{
/**
 * This class calculate the odometry of the tricycle robot
 */
class TriCycleOdometer
{
public:
  /**
   * Default constructor
   *
   * @param[in] _hw joint interface
   * @param[in] _rootNh       Node handle at root namespace
   * @param[in] _controllerNh Node handle inside the controller namespace
   */
  TriCycleOdometer(hardware_interface::VelocityJointInterface* _hw,
                   ros::NodeHandle& _rootNh,
                   ros::NodeHandle& _controllerNh);

  /**
   * Destructor
   */
  virtual ~TriCycleOdometer();

  /**
   * Initialize to odometer of this robot
   */
  void init(const ros::Time& _time);

  /**
   * Updates the odometry of this robots
   *
   * @param[in] _time current time
   * @param[in] _period Time since the last called to update
   */
  void update(const ros::Time& _time, const ros::Duration& _period);

private:
  // the rate of odometry publishing
  ros::Duration m_publishPeriod;
  // the last time when odometry was published
  ros::Time m_lastStatePublish;
  // the values of the last odometry iteration
  double m_lastOdomX, m_lastOdomY, m_lastOdomYaw;
  // used to publish odometry
  boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> m_odomPub;
  // used to publish odometry tf
  boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage>> m_odomPubTf;
};
}  // namespace tricycle_control

#endif  // TRICYCLE_CONTROL_TRICYCLEODOMETER_H
