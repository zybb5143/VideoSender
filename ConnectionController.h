#ifndef CONNECTION_CONTROLLER_H
#define CONNECTION_CONTROLLER_H

#include <cstdlib>
#include <boost/signals2.hpp>
#include <pthread.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include "rtc/rtc.hpp"
#include "LocationController.h"

using namespace boost::signals2;
using namespace std::chrono_literals;
using LocationControllerPtr = std::shared_ptr<LocationController>;
class ConnectionController;
using ConnectionControllerPtr = std::shared_ptr<ConnectionController>;
struct RtpBuffer
{
    uint8_t buffer[1500];
    int len; // length of buffer occupied;
};
enum SendTo
{
    USER,
    CUSTOM_SERVICE
};
class ConnectionController: public std::enable_shared_from_this<ConnectionController>
{
    friend void onGatheringStateChangeCallback(ConnectionControllerPtr _cc,int sendTo,rtc::PeerConnection::GatheringState state);
    friend void webSocketOnMessageCallback(ConnectionControllerPtr _cc, std::variant<rtc::binary, rtc::string> message);
    friend class StreamingController;
    int device_id = 0;
    const static int clockrate = 90000;
    int frame_rate = 20;
    uint32_t ssrc;
    uint16_t rtpSeqNum;
    uint16_t rtpSeqNumCustomService;
    uint16_t turnPort;
    std::string turnIPAddress;
    std::string turnUserName;
    std::string turnCredential;
    std::string webSocketConnectionURL;
    std::shared_ptr<rtc::PeerConnection> userPCPtr;
    std::shared_ptr<rtc::PeerConnection> customServicePCPtr;
    std::atomic<int> connectionId;
    std::atomic<int> requestId;
    std::shared_ptr<rtc::Track> userTrack;
    std::shared_ptr<rtc::Track> customServiceTrack;
    rtc::WebSocket webSocketConnection;
    rtc::Configuration peerConnectionConfiguration;
    signal<void()> signalStartStreaming;
    signal<void()> signalStopStreaming;
    int locationThreadId;
    bool stopSignal{false};
    LocationControllerPtr locationCtrl;
    std::thread location_thread;

public:
    ConnectionController(
        std::string webSocketIPAddress,
        uint16_t webSocketPort,
        std::string webSocketPath,
        std::string turnIP,
        uint16_t turnPort,
        std::string turnUserName,
        std::string turnCredential,
        int frameRate,
        int mobileId);
    ~ConnectionController()
    {
        stopSignal = true;
        location_thread.join();
    }

    void closeUserPeerConnection();
    void closeCustomServicePeerConnection();
    void registerLocationController(LocationControllerPtr lc);
    void connectToWebSocketServer(); // connect to WebSocket Server
    void establishCustomServicePeerConnection();
    void establishPeerConnection();
    void getRealTimeLocation();
    void sendRTPPackage(std::shared_ptr<RtpBuffer> buf, unsigned int curFrameNumber, bool isLastPacketOfCurFrame, int sendTo = 0);
};
void webSocketOnMessageCallback(ConnectionControllerPtr _cc, std::variant<rtc::binary, rtc::string> message);
void onGatheringStateChangeCallback(ConnectionControllerPtr _cc, int sendTo, rtc::PeerConnection::GatheringState state);
#endif