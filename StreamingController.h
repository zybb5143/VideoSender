#ifndef STREAMING_CONTROLLER_H
#define STREAMING_CONTROLLER_H
#include <cstdlib>
#include <iostream>
#include "rtc/rtc.hpp"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <nlohmann/json.hpp>
#include <string>
#include <memory>
#include <thread>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <boost/signals2.hpp>
#include "ConnectionController.h"
#include <boost/program_options.hpp>
extern "C"
{
#include "third_party/hikvision/include/HCNetSDK.h"
}

using namespace boost::signals2;
using ConnectionControllerPtr = std::shared_ptr<ConnectionController>;
class StreamingController
{
    ConnectionControllerPtr connectionCtrlPtr;
    uint32_t curFrameNumber = 0u;
    time_t t1, t2;
    int numberOfLastPackets = 0; // preserved for testing;
    bool packetBeginFlag{false};
    bool setConfigure{false};
    LONG videoChannelNumber = 1;
    LONG userId;
    LONG playHandler;
    std::string deviceLoginUserName;
    std::string deviceLoginPassword;
    std::string deviceConnectionIPAddress;
    std::string localIPAddress; // local ip address to connect to device;
    unsigned short intervalBPFrame;
    unsigned short videoFrameRate;
    unsigned short intervalFrameI;
    unsigned short resolution;
    unsigned short videoBitRate;

public:
    static StreamingController *staticController;
    void registerConnectionController(ConnectionControllerPtr ccp);
    StreamingController(boost::program_options::variables_map &vm);
    static void STATIC_onDataCallback(LONG lPlayHandle, DWORD dwDataType, BYTE *buffer, DWORD bufLen, void *pUser);
    void logoutAndStopStreaming();
    void loginAndSetConfiguration();
    void setDefaultConfiguration();
    void CALLBACK videoDataCallback(LONG lPlayHandle, DWORD dwDataType, BYTE *buffer, DWORD bufLen, void *pUser);
};
#endif