#include <signal.h>
#include <vive_ros/vive_ros.h>

using namespace std;

void shutdownSignalHandler(int sig) { ros::shutdown(); }

int main(int argc, char** argv) {
  signal(SIGINT, shutdownSignalHandler);
  ros::init(argc, argv, "vive_node");

  VIVEnode vive_node;

  if (!vive_node.Init()) {
    ROS_WARN("[VIVE] Failed to initialize the application");
    vive_node.Shutdown();
    return 1;
  }

  vive_node.Run();
  vive_node.Shutdown();

  return 0;
};
