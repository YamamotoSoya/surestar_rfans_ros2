 /* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

// claude: ROS2 port of calculation_node.
//   - dynamic_reconfigure (FilterParamsConfig) -> node parameters +
//     add_on_set_parameters_callback (same six filters, same defaults).
//   - ROS1 cross-node param reads (/rfans_driver/rps etc.) -> declared here
//     as this node's own parameters; the launch file feeds both nodes.
//   - heartbeat pthread (raw UDP :2030) kept verbatim.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <stdio.h>
#include <time.h>
#include <iostream>
#include <string>
#include "bufferDecode.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>

static const int RFANS_POINT_CLOUD_NUM = 1024 ;

static std::vector<SCDRFANS_BLOCK_S> outBlocks ;
static std::vector<RFANS_XYZ_S> outXyzBlocks ;
static sensor_msgs::msg::PointCloud2 outCloud ;

static rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr s_output;
static rclcpp::Subscription<rfans_driver::RfansPacket>::SharedPtr s_sub ;
static int scanSpeed;
extern int gs_pointsPerRound;
extern bool use_gps;
extern int year;
extern int month;
extern int day;
extern int hour;
extern double min_range;
extern double max_range;
extern double min_angle;
extern double max_angle;
extern int ringID;
extern bool use_laserSelection_;
typedef int ( *PFUNC_THREAD)(void *);
static DEVICE_TYPE_E s_deviceType = DEVICE_TYPE_NONE;
typedef struct{
    unsigned int pkgflag;
    unsigned int pkgnumber;
    unsigned int date;
    unsigned short time;
    unsigned int maca;
    unsigned short macb;
    unsigned short dataport;
    unsigned short msgport;
    unsigned char motorspd;
    unsigned int deviceType;
    unsigned short phaseAngle;
    unsigned char padding[225];
}RFANS_HEARTBEAT_S;
static tm gs_lidar_time;

static  pthread_t ssCreateThread(int pri, void * obj, PFUNC_THREAD fnth) {
    pthread_t thrd_ ;
    pthread_create(&thrd_, NULL, (void *(*)(void*))fnth, (void *)obj);
    return thrd_ ;
}
static int heartbeat_thread_run(void *para) {
    (void)para;
    printf("heart beat thread start \n");
    // create socket, then read broadcast port2030 package
    int client_fd;
    int rtn;
    struct sockaddr_in ser_addr;
    const int opt = -1;
    client_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (client_fd < 0) {
        printf("create socket failed!\n");
        return -1;
    }

    bzero(&ser_addr,sizeof(struct sockaddr_in));
    ser_addr.sin_family = AF_INET;
    ser_addr.sin_port = htons(2030);
    ser_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    rtn =setsockopt(client_fd,SOL_SOCKET,SO_REUSEPORT,(char*)&opt,sizeof(opt));
    if(rtn<0)
    {
        printf("setsockopt failed! \n");
    }
    if (bind(client_fd,(struct sockaddr*)&ser_addr, sizeof(sockaddr_in)) < 0) {
        printf("bind server failed!\n");
        return -1;
    }
    printf("after heartbeat init \n");
    while(1) {
        unsigned char buff[512] = {'\0'};
        int rcv = recvfrom(client_fd, buff, sizeof(buff), 0, NULL, NULL);
        if (rcv > 0) {
            RFANS_HEARTBEAT_S hb;
            memset(&hb, '\0', sizeof(RFANS_HEARTBEAT_S));
            memcpy(&hb, buff, sizeof(RFANS_HEARTBEAT_S));
            swapchar((unsigned char*)&hb.pkgflag, sizeof(hb.pkgflag));
            swapchar((unsigned char*)&hb.pkgnumber, sizeof(hb.pkgnumber));
            swapchar((unsigned char*)&hb.date, sizeof(hb.date));
            swapchar((unsigned char*)&hb.time, sizeof(hb.time));

            if (hb.pkgflag == 0xe1e2e3e4) {//heartbeat flag
                gs_lidar_time.tm_year = ((hb.date& 0xFF000000)>>24)+2000;;
                gs_lidar_time.tm_mon =((hb.date & 0xFF0000) >> 16);
                gs_lidar_time.tm_mday = ((hb.date & 0xFF00) >> 8);;
                gs_lidar_time.tm_hour = (hb.date & 0xFF);
                year = gs_lidar_time.tm_year;
                month = gs_lidar_time.tm_mon;
                day = gs_lidar_time.tm_mday;
                hour = gs_lidar_time.tm_hour;
            }
        }
        usleep(500000);//500ms;
    }

    close(client_fd);

    return 0;
}

static void RFansPacketReceived(const rfans_driver::RfansPacket::SharedPtr pkt) {
  SSBufferDec::Depacket(*pkt, outCloud, s_output, s_deviceType) ;
  return ;
}

int main ( int argc , char ** argv )
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("calculation_node");

    // claude: filter parameters (was cfg/FilterParams.cfg via dynamic_reconfigure)
    min_range = node->declare_parameter<double>("min_range", 0.0);
    max_range = node->declare_parameter<double>("max_range", 180.0);
    min_angle = node->declare_parameter<double>("min_angle", 0.0);
    max_angle = node->declare_parameter<double>("max_angle", 360.0);
    use_laserSelection_ = node->declare_parameter<bool>("use_laserSelection", false);
    ringID = node->declare_parameter<int>("laserID", 0);
    auto param_cb = node->add_on_set_parameters_callback(
        [](const std::vector<rclcpp::Parameter> &params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto &p : params) {
                if (p.get_name() == "min_range") min_range = p.as_double();
                else if (p.get_name() == "max_range") max_range = p.as_double();
                else if (p.get_name() == "min_angle") min_angle = p.as_double();
                else if (p.get_name() == "max_angle") max_angle = p.as_double();
                else if (p.get_name() == "use_laserSelection") use_laserSelection_ = p.as_bool();
                else if (p.get_name() == "laserID") ringID = p.as_int();
            }
            return result;
        });

    use_gps = node->declare_parameter<bool>("use_gps", false);

    //advertise name
    std::string advertise_name = node->declare_parameter<std::string>("advertise_name", "rfans_points");
    std::string advertise_path = "rfans_driver/" + advertise_name;

    //subscribe name
    std::string subscribe_name = node->declare_parameter<std::string>("subscribe_name", "rfans_packets");
    std::string subscribe_path = "rfans_driver/" + subscribe_name;
    RCLCPP_INFO(node->get_logger(), "subscribe name %s : %s", subscribe_name.c_str(), subscribe_path.c_str());
    pthread_t s_heartbeat_worker_id = ssCreateThread(1, NULL, heartbeat_thread_run) ;
    (void)s_heartbeat_worker_id;
    //angle durantion
    double angle_duration = node->declare_parameter<double>("angle_duration", 360.0);
    SSBufferDec::SetAngleDuration(angle_duration);
    RCLCPP_INFO(node->get_logger(), "angle_duration : %f", angle_duration);

    // claude: were global-scope reads of the driver node's params in ROS1;
    //         declared here too, launch passes the same values to both nodes.
    scanSpeed = node->declare_parameter<int>("rps", 10);
    bool use_double_echo_ = node->declare_parameter<bool>("use_double_echo", false);

    if(scanSpeed ==5)
    {
        if(use_double_echo_) {
            gs_pointsPerRound = 8000;
        }
        else {
            gs_pointsPerRound = 4000;
        }
    }else if(scanSpeed == 10)
    {
        if (use_double_echo_) {
            gs_pointsPerRound = 4000;
        }
        else {
            gs_pointsPerRound = 2000;
        }
    }
    else
    {
        if(use_double_echo_) {
            gs_pointsPerRound = 2000;
        }
        else {
            gs_pointsPerRound = 1000;
        }
    }

    // device type
    std::string device_model = node->declare_parameter<std::string>("model", "R-Fans-16");
    RCLCPP_INFO(node->get_logger(), "device_model_value: %s", device_model.c_str());
    if (device_model == "C-Fans-128")
    {
        s_deviceType = DEVICE_TYPE_CFANS;
        std::string revise_angle_value = node->declare_parameter<std::string>(
            "revise_angle_128",
            "0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,45,-15,45,-15,0,0,");
        initCFansPara(revise_angle_value);
    }
    else if(device_model == "C-Fans-32")
    {
        s_deviceType = DEVICE_TYPE_CFANS;
        std::string revise_angle_value = node->declare_parameter<std::string>(
            "revise_angle_32",
            "0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,45,-15,45,-15,0,0,");
        initCFans_32(revise_angle_value);
    }
    else
    {
        s_deviceType = DEVICE_TYPE_RFANS;
    }

    // save xyz
    std::string save_xyz_value = node->declare_parameter<std::string>("save_xyz", "no");
    if (0 == strcmp(save_xyz_value.c_str(), "yes")) {
        SSBufferDec::setSaveXYZ(true);
    } else {
        SSBufferDec::setSaveXYZ(false);
    }

    // claude: push config into the decoder, then build the cloud template
    //         (was ros::param::get inside InitPointcloud2)
    SSBufferDec::SetFrameId(node->declare_parameter<std::string>("frame_id", "world"));
    SSBufferDec::SetDeviceIp(node->declare_parameter<std::string>("device_ip", "192.168.0.3"));
    SSBufferDec::SetDataLevel(node->declare_parameter<int>("data_level", 3));
    SSBufferDec::SetDeviceModel(device_model);
    SSBufferDec::InitPointcloud2(outCloud) ;

    s_output = node->create_publisher<sensor_msgs::msg::PointCloud2>(advertise_path, RFANS_POINT_CLOUD_NUM);
    s_sub = node->create_subscription<rfans_driver::RfansPacket>(
        subscribe_path, 30, &RFansPacketReceived);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return  0;
}
