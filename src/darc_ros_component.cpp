#include <ros/ros.h>
#include <darc/component.h>
#include <darc/timer/periodic_timer.h>
#include <darc_ros/pubsub_translator.h>

#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

class DarcRosComponent : public darc::Component
{
protected:
  ros::NodeHandle nh_;

  // Darc Stuff
  darc_ros::PubsubTranslator<tf2_msgs::TFMessage> tf_translator_;
  darc_ros::PubsubTranslator<sensor_msgs::LaserScan> scan_translator_;
  darc_ros::PubsubTranslator<nav_msgs::Odometry> odom_translator_;
  darc_ros::PubsubTranslator<nav_msgs::Odometry> truth_translator_;
  darc::timer::PeriodicTimer work_timer_;

protected:
  void timerHandler()
  {
    ros::spinOnce();
  }

public:
  DarcRosComponent(const std::string& instance_name, darc::Node::Ptr node) :
    darc::Component(instance_name, node),
    tf_translator_( this, "/tf", nh_ ),
    scan_translator_( this, "/scan", nh_ ),
    odom_translator_( this, "/odom", nh_ ),
    truth_translator_( this, "/truth", nh_ ),
    work_timer_(this, boost::bind(&DarcRosComponent::timerHandler, this), boost::posix_time::milliseconds(100))
  {
  }

  template<typename T>
  static boost::shared_ptr<DarcRosComponent> instantiate( const std::string& instance_name, darc::Node::Ptr node )
  {
    int argc = 0;
    ros::init(argc, (char**)0, "darc_ros");

    return Component::instantiate<DarcRosComponent>(instance_name, node);
  }

};

DARC_REGISTER_COMPONENT(DarcRosComponent);
