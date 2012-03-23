#include <iostream>
#include <boost/thread.hpp>
#include <darc/node.h>
#include <darc/component.h>

int main(int argc, const char* argv[])
{
  // Create Node
  darc::NodePtr node = darc::Node::create();

  // Create and run TFBroadcasterComponent
  darc::ComponentPtr c1 = node->instantiateComponent( "DarcRosComponent" );
  c1->run();

  // You can also manually construct a component and call the run() method if you want.
  // But using the register allows for other cool stuff. E.g. starting remotely.
  node->accept("udp://127.0.0.1:5120-5130");
  node->connect("udp://127.0.0.1:5120-5130");

  // Run Node in main thread
  node->run();
  return 0;
}

