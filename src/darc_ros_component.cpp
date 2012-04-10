/*
 * Copyright (c) 2012, Prevas A/S
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Prevas A/S nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * DARC DARC/ROS bridge component
 *
 * \author Morten Kjaergaard
 */

#include <ros/ros.h>
#include <darc/darc.h>
#include <darc_ros/translator_factory.h>
#include <darc_ros/pubsub_translator.h>

// Message types we support
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>

class DarcRosComponent : public darc::Component
{
protected:
  ros::NodeHandle nh_;

  darc_ros::TranslatorFactory factory_;

  std::map<std::string, darc_ros::PubsubTranslatorAbstractPtr> translators_;

  // Darc Stuff
  darc::timer::PeriodicTimer work_timer_;
  darc::timer::PeriodicTimer topic_check_timer_;
  darc::pubsub::StateInterface listener_;

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

  bool createTranslator(const std::string& topic, const std::string& type_name)
  {
    if(translators_.count(topic) == 0)
    {
      darc_ros::PubsubTranslatorAbstractPtr translator = factory_.createTranslator(topic, type_name);
      translators_[topic] = translator;

      if(translator.get() != 0)
      {
	this->startPrimitives();
	return true;
      }
      else
      {
	DARC_INFO("DARC/ROS: Type %s for topic %s not available in factory", type_name.c_str(), topic.c_str());
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
      if(createTranslator(it->name, it->datatype))
      {
	DARC_INFO("DARC/ROS: Created bridge for topic used by ROS %s (%s)", it->name.c_str(), it->datatype.c_str());
      }
    }
  }

  void darcTopicHandler(const std::string& topic, const std::string& type_name, size_t num_sub, size_t num_pub)
  {
    if(num_pub > 0 || num_sub > 0)
    {
      if(createTranslator(topic, type_name))
      {
	DARC_INFO("DARC/ROS: Created bridge for topic used by DARC %s (%s)", topic.c_str(), type_name.c_str());
      }
    }
  }

public:
  DarcRosComponent() :
    factory_(this, nh_),
    work_timer_(this, &DarcRosComponent::workTimerHandler, boost::posix_time::milliseconds(50)),
    topic_check_timer_(this, &DarcRosComponent::topicCheckTimerHandler, boost::posix_time::milliseconds(1000)),
    listener_(this)
  {
    factory_.addType<std_msgs::String>();
    factory_.addType<tf2_msgs::TFMessage>();
    factory_.addType<sensor_msgs::LaserScan>();
    factory_.addType<nav_msgs::Odometry>();
    factory_.addType<sensor_msgs::Image>();
    factory_.addType<geometry_msgs::Twist>();
    factory_.addType<geometry_msgs::PoseWithCovarianceStamped>();
    factory_.addType<geometry_msgs::PoseStamped>();

  }

  void onStart()
  {
    //todo: spin up a thread to handle ros::spin() instead of the timer callback
    listener_.remotePubsubChangesListen(boost::bind(&DarcRosComponent::darcTopicHandler,
						    this, _1, _2, _3, _4));
    listener_.localPubsubChangesListen(boost::bind(&DarcRosComponent::darcTopicHandler,
						   this, _1, _2, _3, _4));
  }

  // Override instantiate method so we can init ros before our primitives are constructed
  template<typename T>
  static boost::shared_ptr<DarcRosComponent> instantiate(const std::string& instance_name, darc::NodePtr node )
  {
    int argc = 0;
    ros::init(argc, (char**)0, "darc_ros");

    return Component::instantiate<DarcRosComponent>(instance_name, node);
  }

};

DARC_REGISTER_COMPONENT(DarcRosComponent);
