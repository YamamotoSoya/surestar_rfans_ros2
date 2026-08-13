/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

// claude: ROS2 port of the driver_node main (was ros::init + NodeHandle pair).
//         Loop shape preserved: pump the device, then service callbacks.
#include <rclcpp/rclcpp.hpp>
#include "rfans_driver.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rfans_driver");
    auto driver = std::make_shared<rfans_driver::Rfans_Driver>(node);

    while( rclcpp::ok() && (driver->spinOnce()))
    {
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
