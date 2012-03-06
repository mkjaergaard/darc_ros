#include <ros/ros.h>
#include <ros/master.h>
#include <darc/component.h>
#include <darc/pubsub/event_listener.h>
#include <darc/timer/periodic_timer.h>
#include <darc_ros/pubsub_translator.h>

// Message types we support
#include <std_msgs/String.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

class TranslatorFactoryEntry
{
public:
  virtual boost::shared_ptr<darc_ros::PubsubTranslatorAbstract> create(darc::Owner * owner, const std::string& topic, ros::NodeHandle &nh) = 0;
};

template<typename T>
class TranslatorFactoryEntryTyped : public TranslatorFactoryEntry
{
public:
  boost::shared_ptr<darc_ros::PubsubTranslatorAbstract> create(darc::Owner * owner, const std::string& topic, ros::NodeHandle &nh)
  {
    return boost::shared_ptr<darc_ros::PubsubTranslator<T> >( new darc_ros::PubsubTranslator<T>(owner, topic, nh) );
  }

};

class TranslatorFactory
{
protected:
  std::map<std::string, TranslatorFactoryEntry*> items_;
  darc::Owner * owner_;
  ros::NodeHandle nh_;

public:
  TranslatorFactory(darc::Owner * owner, ros::NodeHandle &nh) :
    owner_(owner),
    nh_(nh)
  {
  }

  template<typename T>
  void addType()
  {
    items_[ros::message_traits::DataType<T>::value()] = new TranslatorFactoryEntryTyped<T>();
  }

  darc_ros::PubsubTranslatorAbstractPtr createTranslator(const std::string& topic, const std::string& type_name)
  {
    if(items_.count(type_name) > 0)
    {
      return items_[type_name]->create(owner_, topic, nh_);
    }
    else
    {
      DARC_WARNING("No DARC/ROS translator for (%s)", type_name.c_str());
      return darc_ros::PubsubTranslatorAbstractPtr();
    }
  }

};



class DarcRosComponent : public darc::Component
{
protected:
  ros::NodeHandle nh_;

  TranslatorFactory factory_;

  std::map<std::string, darc_ros::PubsubTranslatorAbstractPtr> translators_;

  // Darc Stuff

  darc::timer::PeriodicTimer work_timer_;
  darc::timer::PeriodicTimer topic_check_timer_;
  darc::pubsub::EventListener listener_;

protected:
  void workTimerHandler()
  {
    if( ros::ok() )
    {
      ros::spinOnce();
    }
    else
    {
      exit(0);
    }
  }

  bool add(const std::string& topic, const std::string& type_name)
  {
    if(translators_.count(topic) == 0)
    {
      darc_ros::PubsubTranslatorAbstractPtr translator = factory_.createTranslator(topic, type_name);
      translators_[topic] = translator;

      if(translator.get() != 0)
      {
	return true;
      }
    }
    return false;
  }

  void topicCheckTimerHandler()
  {
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);

    for(ros::master::V_TopicInfo::iterator it = topics.begin(); it != topics.end(); it++)
    {
      if(add(it->name, it->datatype))
      {
	DARC_INFO("::: Added bridge for topic used by ROS %s (%s)", it->name.c_str(), it->datatype.c_str());
      }
    }
  }

  void darcTopicHandler(const std::string& topic, const std::string& type_name, size_t num)
  {
    if(num > 0)
    {
      if(add(topic, type_name))
      {
	DARC_INFO("::: Added bridge for topic used by DARC %s (%s)", topic.c_str(), type_name.c_str());
      }
    }
  }

public:
  DarcRosComponent(const std::string& instance_name, darc::Node::Ptr node) :
    darc::Component(instance_name, node),
    factory_(this, nh_),
    work_timer_(this, boost::bind(&DarcRosComponent::workTimerHandler, this), boost::posix_time::milliseconds(100)),
    topic_check_timer_(this, boost::bind(&DarcRosComponent::topicCheckTimerHandler, this), boost::posix_time::milliseconds(1000)),
    listener_(this)
  {
    factory_.addType<std_msgs::String>();
    factory_.addType<tf2_msgs::TFMessage>();
    factory_.addType<sensor_msgs::LaserScan>();
    factory_.addType<nav_msgs::Odometry>();

    //todo spin up a thread to handle ros::spin instead of the timer callback
    listener_.remoteSubscriberChangesListen(boost::bind(&DarcRosComponent::darcTopicHandler,
							this, _1, _2, _3));
    listener_.remotePublisherChangesListen(boost::bind(&DarcRosComponent::darcTopicHandler,
						       this, _1, _2, _3));

  }

  // Override instantiate method so we can init ros before our primitives are constructed
  template<typename T>
  static boost::shared_ptr<DarcRosComponent> instantiate( const std::string& instance_name, darc::Node::Ptr node )
  {
    int argc = 0;
    ros::init(argc, (char**)0, "darc_ros");

    return Component::instantiate<DarcRosComponent>(instance_name, node);
  }

};

DARC_REGISTER_COMPONENT(DarcRosComponent);
