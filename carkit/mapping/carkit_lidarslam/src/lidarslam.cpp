#include <carkit_scanmatcher/carkit_scanmatcher_component.h>
#include <carkit_graph_based_slam/carkit_graph_based_slam_component.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  rclcpp::executors::MultiThreadedExecutor exec;

  auto carkit_scanmatcher = std::make_shared<graphslam::ScanMatcherComponent>(options);
  exec.add_node(carkit_scanmatcher);
  auto graphbasedslam = std::make_shared<graphslam::GraphBasedSlamComponent>(options);
  exec.add_node(graphbasedslam);

  exec.spin();
  rclcpp::shutdown();

  return 0;
}
