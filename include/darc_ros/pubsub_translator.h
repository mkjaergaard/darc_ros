#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <darc/subcomponent.h>
#include <darc/timer/periodic_timer.h>
#include <darc/pubsub/subscriber.h>
#include <darc/pubsub/publisher.h>

namespace darc_ros
{

template<typename T>
class PubsubTranslator : public darc::Subcomponent
{
protected:
  // ROS Stuff
  ros::NodeHandle nh_;
  ros::Publisher ros_pub_;
  ros::Subscriber ros_sub_;

  // Darc Stuff
  darc::pubsub::Subscriber<T> darc_sub_;
  darc::pubsub::Publisher<T> darc_pub_;

protected:
  void darcHandler(const boost::shared_ptr<const T> msg)
  {
    ros_pub_.publish(msg);
  }

  void rosHandler(const ros::MessageEvent<T const>& event)
  {/*
    const std::string& publisher_name = event.getPublisherName();
    const ros::M_string& header = event.getConnectionHeader();
    ros::Time receipt_time = event.getReceiptTime();
   */
    darc_pub_.publish(event.getMessage());
  }

public:
  PubsubTranslator(darc::Owner * owner, const std::string& topic, ros::NodeHandle& nh) :
    darc::Subcomponent(owner),
    // Ros
    ros_pub_( nh_.advertise<T>(topic, 10 ) ),
    // Darc
    darc_sub_(this, topic, boost::bind(&PubsubTranslator::darcHandler, this, _1)),
    darc_pub_(this, topic)
  {
  }

};

}
