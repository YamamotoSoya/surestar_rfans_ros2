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

    // claude: the vendor loop exited whenever spinOnce() returned 0, which in
    //         realtime mode happens after ~1 s of device silence (power cycle,
    //         cable bump) — a robot driver must survive that. Only pcap replay
    //         (EOF) may end the loop.
    const bool realtime = driver->isRealtime();
    while( rclcpp::ok() )
    {
        int got = driver->spinOnce();
        if (!got && !realtime) break;   // pcap end-of-file
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
