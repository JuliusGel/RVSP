#include "tricycle_control/TriCycleOdometer.h"

#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <tf/tf.h>

using tricycle_control::TriCycleOdometer;

TriCycleOdometer::TriCycleOdometer(hardware_interface::VelocityJointInterface* _hw,
                                   ros::NodeHandle& _rootNh,
                                   ros::NodeHandle& _controllerNh)
: m_publishPeriod()
, m_lastStatePublish()
, m_odomPub(nullptr)
, m_odomPubTf(nullptr)
, m_lastOdomX(0.0)
, m_lastOdomY(0.0)
, m_lastOdomYaw(0.0)
{
  ROS_DEBUG_STREAM("TriCycleOdometer::TriCycleOdometer");

  m_publishPeriod = ros::Duration(1 / 20);

  m_odomPub.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(_controllerNh, "odom", 100));
  m_odomPub->msg_.header.frame_id = "odom";
  m_odomPub->msg_.child_frame_id = "base_link";
  m_odomPub->msg_.pose.pose.position.z = 0;
  m_odomPubTf.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(_rootNh, "/tf", 100));
  m_odomPubTf->msg_.transforms.resize(1);
  m_odomPubTf->msg_.transforms[0].transform.translation.z = 0.0;
  m_odomPubTf->msg_.transforms[0].child_frame_id = "base_link";
  m_odomPubTf->msg_.transforms[0].header.frame_id = "odom";
}

TriCycleOdometer::~TriCycleOdometer()
{
  ROS_DEBUG_STREAM("TriCycleOdometer::~TriCycleOdometer");
}

void TriCycleOdometer::init(const ros::Time& _time)
{
  ROS_DEBUG_STREAM("Odometer::init");

  // Register starting time used to keep fixed rate
  m_lastStatePublish = _time;

  m_lastOdomX = 0;
  m_lastOdomY = 0;
  m_lastOdomYaw = 0;
}

void TriCycleOdometer::update(const ros::Time& _time, const ros::Duration& _period)
{
  ROS_DEBUG_STREAM("TriCycleOdometer::update");
  //////////////////////////////////////////////////////////////////////////////
  // TODO calculate and set odomX etc. here
  //////////////////////////////////////////////////////////////////////////////
  const double odomX = 0;
  const double odomY = 0;
  const double odomYaw = 0;
  const double invDeltaT = 1 / _period.toSec();

  // Publish odometry message
  if (m_lastStatePublish + m_publishPeriod < _time)
  {
    m_lastStatePublish += m_publishPeriod;
    // Compute and publish orientation info
    const geometry_msgs::Quaternion orientation(
          tf::createQuaternionMsgFromYaw(odomYaw));

    // Populate odom message and publish
    if (m_odomPub->trylock())
    {
      m_odomPub->msg_.header.stamp = _time;
      m_odomPub->msg_.pose.pose.position.x = odomX;
      m_odomPub->msg_.pose.pose.position.y = odomY;
      m_odomPub->msg_.pose.pose.orientation = orientation;
      m_odomPub->msg_.twist.twist.linear.x = (odomX - m_lastOdomX) * invDeltaT;
      m_odomPub->msg_.twist.twist.linear.y = (odomY - m_lastOdomY) * invDeltaT;
      m_odomPub->msg_.twist.twist.angular.z = -angles::shortest_angular_distance(odomYaw, m_lastOdomYaw) * invDeltaT;
      m_odomPub->unlockAndPublish();
    }

    // Publish tf /odom frame
    if (m_odomPubTf->trylock())
    {
      geometry_msgs::TransformStamped& odom_frame = m_odomPubTf->msg_.transforms[0];
      odom_frame.header.stamp = _time;
      odom_frame.transform.translation.x = odomX;
      odom_frame.transform.translation.y = odomY;
      odom_frame.transform.rotation = orientation;
      m_odomPubTf->unlockAndPublish();
    }
  }

  m_lastOdomX = odomX;
  m_lastOdomY = odomY;
  m_lastOdomYaw = odomYaw;
}
