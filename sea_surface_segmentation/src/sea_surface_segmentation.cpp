
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"

class SeaSurfaceSegmentation: public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit SeaSurfaceSegmentation()
  : rclcpp_lifecycle::LifecycleNode("sea_surface_segmentation")
  {

  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    declare_parameter("neural_network", std::string());


    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state)
  {
    LifecycleNode::on_activate(state);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  dai::Pipeline pipeline_;
  dai::Device device_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto sss = std::make_shared<SeaSurfaceSegmentation>();
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(sss->get_node_base_interface());
  exe.spin();
  
  rclcpp::shutdown();

  return 0;
}
