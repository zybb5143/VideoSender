#ifndef CONNECTION_CONTROLLER_CPP
#define CONNECTION_CONTROLLER_CPP

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <nlohmann/json.hpp>
#include <string>
#include <memory>
#include <thread>
#include <chrono>
#include <thread>
#include <boost/signals2.hpp>
#include "StreamingController.h"
#include "utils/utils.h"
#include "macro.h"

#include "ConnectionController.h"
using json = nlohmann::json;
using namespace std::literals::chrono_literals;
void onGatheringStateChangeCallback(ConnectionControllerPtr _cc, int sendTo, rtc::PeerConnection::GatheringState state)
{
    std::cout << "[INFO] " << (sendTo == SendTo::CUSTOM_SERVICE ? "To customService--" : "To Receiver--");
    printf("Gathering State Change\n");
    if (state == rtc::PeerConnection::GatheringState::Complete)
    {
        std::string toWhom = (sendTo == SendTo::CUSTOM_SERVICE ? "customService" : "receiver");
        json jmsg =
            {
                {"from", "sender"},
                {"type", "offer"},
                (json){"sdp",
                       {{"type", _cc->userPCPtr->localDescription()->typeString()},
                        {"sdp", std::string(_cc->userPCPtr->localDescription().value())}}},
                (json){"to", toWhom},
                (json){"requestId", (int)_cc->requestId},
                (json){"connectionId", (int)_cc->connectionId}};
        _cc->webSocketConnection.send(jmsg.dump());
    }
}
void webSocketOnMessageCallback(ConnectionControllerPtr _cc, std::variant<rtc::binary, rtc::string> message)
{
    if (std::holds_alternative<rtc::string>(message))
    {
        json jmsg = json::parse(std::get<std::string>(message));
        std::string msgType = jmsg["type"].get<std::string>();
        printf("[on message] type:%s\n", msgType.c_str());
        if (msgType == "answer")
        {
            rtc::Description remoteSDP(jmsg["data"]["sdp"].get<std::string>(), jmsg["data"]["type"].get<std::string>());
            if (jmsg.count("from"))
            {
                std::string from = jmsg["from"];
                if (from == "receiver")
                {
                    int connectionId = jmsg["connectionId"].get<int>();
                    if (jmsg.count("connectionId") == 0)
                    {
                        return;
                    }
                    if (connectionId != _cc->connectionId)
                    {
                        return;
                    }
                    if (_cc->userPCPtr.use_count())
                    {
                        _cc->userPCPtr->setRemoteDescription(remoteSDP);
                    }
                }
                else if (from == "customService")
                {
                    if (jmsg.count("requestId") == 0)
                    {
                        return;
                    }
                    int requestId = jmsg["requestId"].get<int>();
                    if (requestId != _cc->requestId)
                    {
                        return;
                    }
                    if (_cc->customServicePCPtr.use_count())
                    {
                        _cc->customServicePCPtr->setRemoteDescription(remoteSDP);
                    }
                }
            }
            else
            {
                return;
            }
        }
        else if (msgType == MSG_TYPE_USER_CLOSE_VIDEO_CONNECTION)
        {
            int curConnectionId = -1;
            if (jmsg.count("connectionId") == 0)
            {
                return;
            }
            curConnectionId = jmsg["connectionId"].get<int>();
            if (curConnectionId != _cc->connectionId)
            {
                return;
            }
            _cc->closeUserPeerConnection();
            _cc->closeCustomServicePeerConnection();
            _cc->userPCPtr.reset();
            _cc->customServicePCPtr.reset();
            _cc->requestId = -1;
            _cc->connectionId = -1;
            if constexpr (connectorDebugFlag)
            {
                printf("[INFO] User Close Connection\n");
            }
            _cc->webSocketConnection.send(
                (json){
                    {"type", "deviceState"},
                    {"state", "free"},
                    {"from", "sender"}}
                    .dump());
        }
        else if (msgType == MSG_TYPE_USER_ESTABLISH_CONNECTION)
        {
            if constexpr (connectorDebugFlag)
            {
                printf("[INFO] User Establish Peer Connection\n");
            }
            if (jmsg.count("connectionId") == 0)
            {
                return;
            }
            int connectionId = jmsg["connectionId"].get<int>();
            _cc->connectionId = connectionId;
            _cc->establishPeerConnection();
            if constexpr (connectorDebugFlag)
            {
                printf("[INFO] Establish Receiver(user) Peer Connection\n");
            }
        }
        else if (msgType == MSG_TYPE_SERVICE_STUFF_ACCEPT_REQUEST)
        {
            if (jmsg.count("requestId") == 0)
            {
                return;
            }
            int requestId = jmsg["requestId"].get<int>();
            _cc->requestId = requestId;
            _cc->establishCustomServicePeerConnection();
        }
        else if (msgType == MSG_TYPE_SERVICE_STUFF_STOP_SERVICE)
        {
            if (jmsg.count("requestId") == 0)
            {
                return;
            }
            int requestId = jmsg["requestId"].get<int>();
            if (requestId != _cc->requestId)
            {
                return;
            }
            _cc->requestId = -1;
            _cc->closeCustomServicePeerConnection();
        }
    }
};
void ConnectionController::registerLocationController(LocationControllerPtr lc)
{
    this->locationCtrl = lc;
}
void ConnectionController::getRealTimeLocation()
{
    while (true and !stopSignal)
    {
        std::this_thread::sleep_for(10000ms);
        if (webSocketConnection.isOpen())
        {
            nlohmann::json jvalue = {{"type", "location"},
                                     {"x", locationCtrl->getRealTimeLocation().first},
                                     {"y", locationCtrl->getRealTimeLocation().second},
                                     {"from", "sender"}};
            webSocketConnection.send(jvalue.dump());
        }
    }
}
ConnectionController::ConnectionController(
    std::string websocket_ip,
    uint16_t websocket_port,
    std::string websocket_path,
    std::string turn_ip,
    uint16_t turn_port,
    std::string turn_user_name,
    std::string turn_credential,
    int frame_rate,
    int mobile_id) : device_id(mobile_id),
                     turnCredential(turn_credential),
                     turnIPAddress(turn_ip),
                     turnUserName(turn_user_name),
                     turnPort(turn_port),
                     frame_rate(frame_rate)
{
    webSocketConnectionURL = websocketURLCreator(websocket_ip, websocket_port, websocket_path, false, {{"deviceId", std::to_string(device_id)}});
    ssrc = rand() % (UINT_MAX / 10);
    location_thread = std::thread(&ConnectionController::getRealTimeLocation, this);
    requestId = -1;
    connectionId = -1;
}
void ConnectionController::closeUserPeerConnection()
{
#if (ENABLE_STREAMING)
    signalStopStreaming();
#endif
    if (userTrack.use_count() == 0)
    {
        return;
    }
    userTrack->close();
    if (customServiceTrack.use_count() == 0)
    {
        return;
    }
    customServiceTrack->close();
}
void ConnectionController::closeCustomServicePeerConnection()
{

    if (customServiceTrack.use_count() == 0)
    {
        return;
    }

    customServiceTrack->close();
}
void ConnectionController::connectToWebSocketServer() // connect to WebSocket Server
{
    auto functionOnWebSocketOpened = [&]()
    {
#ifdef CONNECTOR_DEBUG
        std::cout << "[INFO] websocket opened" << std::endl;
#endif
    };
    webSocketConnection.onOpen(functionOnWebSocketOpened);
    webSocketConnection.onMessage(std::bind(webSocketOnMessageCallback, shared_from_this(), std::placeholders::_1));
    webSocketConnection.open(webSocketConnectionURL);
}
void ConnectionController::establishCustomServicePeerConnection()
{
    if constexpr (connectorDebugFlag)
    {
        printf("[INFO] ConnectorController::establishCustomServicePeerConnection()\n");
    }
    peerConnectionConfiguration.iceServers.emplace_back(turnIPAddress, turnPort, turnUserName, turnCredential);
    rtc::Description::Video videoDescription("video", rtc::Description::Direction::SendOnly);
    videoDescription.addH264Codec(96);
    videoDescription.addSSRC(ssrc, "video-send");
    customServicePCPtr = std::make_shared<rtc::PeerConnection>(peerConnectionConfiguration);
    customServicePCPtr->onGatheringStateChange(
        std::bind(onGatheringStateChangeCallback, shared_from_this(), SendTo::CUSTOM_SERVICE, std::placeholders::_1));
    rtpSeqNumCustomService = rand() % (8192);
    customServiceTrack = customServicePCPtr->addTrack(videoDescription);
    if constexpr (connectorDebugFlag)
    {
        customServiceTrack->onOpen([&]()
                                   { std::cout << "[INFO][Service] track opened" << std::endl; });
    }
    customServicePCPtr->setLocalDescription();
}
void ConnectionController::establishPeerConnection()
{
    peerConnectionConfiguration.iceServers.emplace_back(turnIPAddress, turnPort /* , turnUserName, turnCredential */);
    rtc::Description::Video videoDescription("video", rtc::Description::Direction::SendOnly);
    videoDescription.addH264Codec(96);
    videoDescription.addSSRC(ssrc, "video-send");
    userPCPtr = std::make_shared<rtc::PeerConnection>(peerConnectionConfiguration);
    userPCPtr->onGatheringStateChange(std::bind(onGatheringStateChangeCallback, shared_from_this(), SendTo::USER, std::placeholders::_1));
    rtpSeqNum = rand() % (8192);
    userTrack = userPCPtr->addTrack(videoDescription);
    while (!webSocketConnection.isOpen())
    {
        if (webSocketConnection.isClosed())
        {
            return;
        }
        std::this_thread::sleep_for(2ms);
    }

    userTrack->onOpen([&]()
                      {
#ifdef CONNECTOR_DEBUG
                          std::cout << "[INFO] user track opened" << std::endl;
#endif
#if (ENABLE_STREAMING)
                          signalStartStreaming();
#endif
                      });

    userPCPtr->setLocalDescription();
}
void ConnectionController::sendRTPPackage(std::shared_ptr<RtpBuffer> buf, unsigned int curFrameNumber, bool isLastPacketOfCurFrame, int sendTo)
{
    if (this->userTrack->isOpen() == false)
    {
        return;
    }
    rtc::RtpHeader *pkt = reinterpret_cast<rtc::RtpHeader *>(buf->buffer);
    pkt->preparePacket();
    pkt->setSsrc(ssrc);
    pkt->setTimestamp(curFrameNumber * (clockrate / frame_rate));
    pkt->setMarker(isLastPacketOfCurFrame);
    pkt->setExtension(false);
    pkt->setPayloadType(static_cast<uint8_t>(96u));
    if (sendTo == SendTo::USER)
        pkt->setSeqNumber(this->rtpSeqNum++);
    else if (sendTo == SendTo::CUSTOM_SERVICE)
        pkt->setSeqNumber(this->rtpSeqNumCustomService++);
    if (userTrack)
    {
        userTrack->send(reinterpret_cast<std::byte *>(buf->buffer), buf->len);
    }
    if (customServiceTrack)
    {
        customServiceTrack->send(reinterpret_cast<std::byte *>(buf->buffer), buf->len);
    }
}

#endif