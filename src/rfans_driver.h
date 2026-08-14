/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#ifndef _RFANS_DRIVER_H_
#define _RFANS_DRIVER_H_
#include <rclcpp/rclcpp.hpp>
#include "ioapi.h"
#include <stdint.h>


namespace rfans_driver
{
class Rfans_Driver
{
public:
    // claude: ROS2 port — one rclcpp node replaces the ROS1 global/private NodeHandle pair
    explicit Rfans_Driver(rclcpp::Node::SharedPtr node);
    ~Rfans_Driver();

    int spinOnce();
    bool isRealtime() { return worRealtime(); }   // claude: lets main keep looping on device silence
    int prog_Set(DEB_PROGRM_S &program);
    int datalevel_Set(DEB_PROGRM_S &program);

private:
    double calcReplayPacketRate();
    void configDeviceParams();
    void setupNodeParams();   // claude: reads params from node_ (ROS2 port)
    void commandHandle(const std::shared_ptr<rfans_driver::RfansCommand::Request> req,
                       std::shared_ptr<rfans_driver::RfansCommand::Response> res);

    bool worRealtime();
    int spinOnceRealtime();
    bool spinOnceSimu();

private:
    struct
    {
        std::string command_path;
        std::string advertise_path;
        std::string device_ip;
        std::string device_name;
        std::string simu_filepath;
        int dataport;
        int scnSpeed;
        int data_level;
        bool dual_echo;
        bool read_once;    // claude: pcap replay options (were InputPCAP-private params)
        bool read_fast;
        double repeat_delay;
    } config_;
    rfans_driver::IOAPI *m_devapi;
    rclcpp::Node::SharedPtr node_;   // claude: ROS2 port
    rclcpp::Publisher<rfans_driver::RfansPacket>::SharedPtr m_output;
    rclcpp::Service<rfans_driver::RfansCommand>::SharedPtr server_;
    rfans_driver::RfansPacket tmpPacket;
    rfans_driver::InputPCAP *input_;
};

}

#endif //_RFANS_DRIVER_H_
