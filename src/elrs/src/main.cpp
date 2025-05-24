#include "rclcpp/rclcpp.hpp"

#include "elrs/elrs_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ELRSNode>());
    rclcpp::shutdown();
    return 0;
}