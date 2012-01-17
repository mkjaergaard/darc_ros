#include <iostream>
#include <boost/thread.hpp>
#include <darc/node.h>
#include <darc/component.h>

int main(int argc, const char* argv[])
{
  // Create Node
  darc::Node::Ptr node = darc::Node::create();

  // Create and run TFBroadcasterComponent
  darc::Component::Ptr c1 = node->instantiateComponent( "DarcRosComponent" );
  boost::shared_ptr<boost::thread> c1_thread(new boost::thread( boost::bind(&darc::Component::run, c1)));

  // You can also manually construct a component and call the run() method if you want.
  // But using the register allows for other cool stuff. E.g. starting remotely.
  node->setNodeID(300);
  node->accept("udp://127.0.0.1:5300");
  node->connect(200, "udp://127.0.0.1:5200");

  // Run Node in main thread
  node->run();
  return 0;
}

