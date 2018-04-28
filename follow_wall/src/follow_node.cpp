#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"

double front_range = 0.0;
double left_range = 0.0;
double right_range = 0.0;

void front_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("Received front:" <<  *msg);
  front_range = msg->range;
}

void left_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("Received left:" <<  *msg);
  left_range = msg->range;
}

void right_Callback(const sensor_msgs::Range::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("Received right:" <<  *msg);
  right_range = msg->range;
}

/**
 * Here is the main function of your node
 */
int main(int argc, char **argv)
{
  // initialize ROS first
  ros::init(argc, argv, "talker");

  // now create node handle
  ros::NodeHandle n;

  // advertise that we are going to publish cmd_vel commands to conrtol robot
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  // defines how often we want to send commands to the robot
  ros::Rate loop_rate(10);

  // subscribe to all of the sensor data
  ros::Subscriber sub_front = n.subscribe("front_sonar", 1, front_Callback);
  ros::Subscriber sub_left = n.subscribe("left_sonar", 1, left_Callback);
  ros::Subscriber sub_right = n.subscribe("right_sonar", 1, right_Callback);

  while (ros::ok())
  {
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    // YOUR CODE SHOULD GO HERE!!!
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    // create new robot control command and fill it with your values
    geometry_msgs::Twist msg;
    msg.linear.x = 1.0; //TODO: fill this in wiht your value
    msg.angular.z = 0.0; //TODO: fill this in wiht your value

    ROS_DEBUG_STREAM("You created following robot command:" << msg);

    // send this command to the robot
    vel_pub.publish(msg);

    // allow ros to get new sensor data and send the command to robot
    ros::spinOnce();

    // wait to make sure the messages are not send to often
    loop_rate.sleep();
  }
  return 0;
}
