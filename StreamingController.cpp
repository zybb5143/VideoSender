#ifndef STREAMING_CONTROLLER_CPP
#define STREAMING_CONTROLLER_CPP

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
#include "StreamingController.h"
#include <boost/program_options.hpp>
extern "C"
{
#include "third_party/hikvision/include/HCNetSDK.h"
}


//#define TEST_AND_LEARN                           \
    do                                             \
    {                                              \
        std::cout<<std::dec<<bufLen<<std::endl;    \
        int idx = 0;                               \
        while (idx < bufLen)                       \
        {                                          \
            unsigned char ch = *(buffer + idx);    \
            idx += 1;                              \
            int val1 = ch % 16;                    \
            int val2 = ch / 16;                    \
            std::cout << std::hex << val2 << val1; \
            std::cout << " ";                      \
        }                                          \
        std::cout << std::endl;                    \
    } while (0);

#define RAW_H264_PAYLOAD_MAX_LEN 1440u
#define BYTES_FOR_RTP_HEADER 12u
#define WITH_EXTRA_PACKAGE(BUFFER_LENGTH) ((BUFFER_LENGTH) == 5120u)

using namespace boost::signals2;

void StreamingController::registerConnectionController(ConnectionControllerPtr ccp)
{
    this->connectionCtrlPtr = ccp;
    connectionCtrlPtr->connectToWebSocketServer();
    connectionCtrlPtr->signalStopStreaming.connect(boost::bind(&StreamingController::logoutAndStopStreaming, this));
    connectionCtrlPtr->signalStartStreaming.connect(boost::bind(&StreamingController::loginAndSetConfiguration, this));
}
StreamingController *StreamingController::staticController = nullptr;
StreamingController::StreamingController(boost::program_options::variables_map &vm)
    : deviceConnectionIPAddress(vm["device_address"].as<std::string>()),
      deviceLoginUserName(vm["device_user_name"].as<std::string>()),
      deviceLoginPassword(vm["device_password"].as<std::string>()),
      localIPAddress(vm["local_address"].as<std::string>()),
      intervalBPFrame(vm["hik_interval_BP_frame"].as<uint16_t>()),
      videoFrameRate(vm["hik_video_frame_rate"].as<uint16_t>()),
      resolution(vm["hik_resolution"].as<uint16_t>()),
      videoBitRate(vm["hik_video_bit_rate"].as<uint16_t>()),
      intervalFrameI(vm["interval_frame_I"].as<uint16_t>())
{

    staticController = this;
}

void StreamingController::STATIC_onDataCallback(LONG lPlayHandle, DWORD dwDataType, BYTE *buffer, DWORD bufLen, void *pUser)
{
    staticController->videoDataCallback(lPlayHandle, dwDataType, buffer, bufLen, pUser);
}
void StreamingController::logoutAndStopStreaming()
{
    NET_DVR_StopRealPlay(playHandler);
    NET_DVR_Logout(userId);
}
void StreamingController::loginAndSetConfiguration()
{
    packetBeginFlag = false;
    int ret = NET_DVR_Init();
#ifdef STREAMING_DEBUG
    std::cout << "[INFO] device: " << ret << std::endl;
#endif
    char acquiredIPAddresses[16][16] = {0};
    NET_DVR_GetLocalIP(acquiredIPAddresses, 0, 0);
    DWORD bind_index = -1;
    for (unsigned int i = 0; i < 16u; i++)
    {
        if (strcmp(static_cast<char *>(acquiredIPAddresses[i]), localIPAddress.c_str()) == 0)
        {
            bind_index = i;
        }
    }
    std::cout << bind_index << std::endl;
    NET_DVR_SetValidIP(bind_index, true);

    // user login into the device
    NET_DVR_USER_LOGIN_INFO userLoginInfo = {0};
    userLoginInfo.bUseAsynLogin = 0; // login synchronically
    strcpy(userLoginInfo.sUserName, deviceLoginUserName.c_str());
    strcpy(userLoginInfo.sPassword, deviceLoginPassword.c_str());
    strcpy(userLoginInfo.sDeviceAddress, deviceConnectionIPAddress.c_str());
    userLoginInfo.wPort = 8000;
    NET_DVR_DEVICEINFO_V40 deviceInfo = {0};
    userId = NET_DVR_Login_V40(&userLoginInfo, &deviceInfo);
    std::cout << "[INFO] user_id: " << userId << std::endl;
    if (userId < 0)
    {
        std::cerr << NET_DVR_GetLastError() << std::endl;
        return;
    }
    if (!setConfigure)
    {
        setDefaultConfiguration();
    }
    // get real time streaming data
    NET_DVR_PREVIEWINFO previewInfo = {0};
    previewInfo.lChannel = videoChannelNumber;
    previewInfo.dwStreamType = 0u; // main code stream [0]
    previewInfo.bBlocked = 0u;     // async
    time(&t1);
    playHandler = NET_DVR_RealPlay_V40(userId, &previewInfo, StreamingController::STATIC_onDataCallback, NULL);
#ifdef STREAMING_DEBUG
    std::cout << "[INFO] play_handler: " << playHandler << std::endl;
#endif
    if (playHandler < 0)
    {
        std::cout << NET_DVR_GetLastError() << std::endl;
        return;
    }
}
void StreamingController::setDefaultConfiguration()
{
    // set compression configurations for video stream
    int ret;
    NET_DVR_COMPRESSIONCFG_V30 videoCompressionConfig = {0};
    unsigned int returnBytes;
    ret = NET_DVR_GetDVRConfig(userId, NET_DVR_GET_COMPRESSCFG_V30, videoChannelNumber, &videoCompressionConfig, sizeof(videoCompressionConfig), &returnBytes);
    if (ret < 0)
    {
        std::cerr << "Unable to Get Compression Configuration" << std::endl;
        return;
    }
    videoCompressionConfig.struNormHighRecordPara.byFormatType = 9u;                   // in PS(GB[national standard]) + RTP
    videoCompressionConfig.struNormHighRecordPara.byVideoEncType = 1u;                 // standard h264 format
    videoCompressionConfig.struNormHighRecordPara.byResolution = resolution;           // 19u -> 720p
    videoCompressionConfig.struNormHighRecordPara.dwVideoFrameRate = videoFrameRate;   // 13u -> 20fps
    videoCompressionConfig.struNormHighRecordPara.byIntervalBPFrame = intervalBPFrame; // 2u -> No B Frames
    videoCompressionConfig.struNormHighRecordPara.dwVideoBitrate = videoBitRate;       // 2.5M -> 24
    videoCompressionConfig.struNormHighRecordPara.wIntervalFrameI = intervalFrameI;    // number of interval frames between two I frames
    ret = NET_DVR_SetDVRConfig(userId, NET_DVR_SET_COMPRESSCFG_V30, videoChannelNumber, &videoCompressionConfig, sizeof(videoCompressionConfig));
    if (ret < 0)
    {
        std::cerr << "Unable to Set Compression Configuration" << std::endl;
        return;
    }

    // set auto-focus mode for camera
    NET_DVR_FOCUSMODE_CFG focusModeConfig = {0};
    ret = NET_DVR_GetDVRConfig(userId, NET_DVR_GET_FOCUSMODECFG, videoChannelNumber, &focusModeConfig, sizeof(focusModeConfig), &returnBytes);
    if (ret < 0)
    {
        std::cerr << "Unable to Get Focus Configuration" << std::endl;
        return;
    }
    // these configurations for focus mode no need to be modified;
    focusModeConfig.byFocusMode = 0u;
    focusModeConfig.byFocusSpeedLevel = 1u;
    focusModeConfig.byFocusSensitivity = 2u;
    ret = NET_DVR_SetDVRConfig(userId, NET_DVR_SET_FOCUSMODECFG, videoChannelNumber, &focusModeConfig, sizeof(focusModeConfig));
    if (ret < 0)
    {
        std::cerr << "Unable to set Focus Mode Configuration" << std ::endl;
        return;
    }
    setConfigure = true;
}
void CALLBACK StreamingController::videoDataCallback(LONG lPlayHandle, DWORD dwDataType, BYTE *buffer, DWORD bufLen, void *pUser)
{
    switch (dwDataType)
    {
    case NET_DVR_SYSHEAD:
        break;
    case NET_DVR_STREAMDATA:
        uint8_t FU_TYPE = 0u;
        uint8_t FU_NRI = 0u;
        if (bufLen > 0)
        {
#ifdef TEST_AND_LEARN
            TEST_AND_LEARN
#endif
#ifdef STREAMING_DEBUG
            // used for checking whether real-time framerate is equal to framerate expected;
            time(&t2);
            std::cout
                << std::dec << "[" << t2 - t1 << " s ]: -"
                << curFrameNumber * 1L << " - " << bufLen << std::endl;
#endif

            uint16_t curRawDataIndex = 0u;
            if (buffer[0] == 0u and buffer[1] == 0u and buffer[2] == 1u and *reinterpret_cast<char *>(buffer + 3) == '\xBA')
            {
                curRawDataIndex += 12u;
                uint8_t paddingLength = buffer[curRawDataIndex];
                curRawDataIndex += paddingLength + 1u;
                curFrameNumber = ntohl(*reinterpret_cast<uint32_t *>(buffer + curRawDataIndex));
            }
            else if (buffer[0] == 0u and buffer[1] == 0u and buffer[2] == 1u and *reinterpret_cast<char *>(buffer + 3) == '\xE0')
            {
                bool isFirstPacketOfCurFrame{false};
                bool isLastPacketOfCurFrame{false};
                uint16_t payload_len = ntohs(*reinterpret_cast<uint16_t *>(buffer + 4));
                curRawDataIndex = 8u;
                unsigned char paddingLength = buffer[curRawDataIndex];
                curRawDataIndex += 1u + paddingLength;
                std::shared_ptr<RtpBuffer> rtpPacketPtr;
                rtpPacketPtr = std::make_shared<RtpBuffer>();
                if (buffer[curRawDataIndex] == 0u and buffer[curRawDataIndex + 1] == 0u and buffer[curRawDataIndex + 2] == 0u and buffer[curRawDataIndex + 3] == 1u)
                {
                    if (bufLen > 112u)
                    {
                        isFirstPacketOfCurFrame = true;
                        packetBeginFlag = true;
                    }

                    curRawDataIndex += 4u;
                    if (!packetBeginFlag)
                    {
                        memcpy(rtpPacketPtr->buffer + BYTES_FOR_RTP_HEADER, buffer + curRawDataIndex, bufLen - curRawDataIndex);
                        rtpPacketPtr->len = BYTES_FOR_RTP_HEADER + bufLen - curRawDataIndex;
                        connectionCtrlPtr->sendRTPPackage(rtpPacketPtr, curFrameNumber, isLastPacketOfCurFrame);
                        return;
                    }
                }
                // NALU should be disparted.
                if (isFirstPacketOfCurFrame)
                {
                    FU_TYPE = 0b00011111u & buffer[curRawDataIndex];
                    FU_NRI = 0b11100000u & buffer[curRawDataIndex];
                    curRawDataIndex++;
                }
                unsigned int bytesSent = 0u;
                unsigned char FU_indicator{0u};
                unsigned char FU_header{0u};
                FU_indicator |= FU_NRI;
                FU_indicator |= 0b00011100u; // FU-A
                FU_header |= FU_TYPE;

                while (bytesSent < bufLen - curRawDataIndex)
                {
                    unsigned int curBytesSent = std::min(RAW_H264_PAYLOAD_MAX_LEN, bufLen - curRawDataIndex - bytesSent);
                    rtpPacketPtr->len = BYTES_FOR_RTP_HEADER + 2u + curBytesSent;
                    memcpy(rtpPacketPtr->buffer + BYTES_FOR_RTP_HEADER + 2u, buffer + curRawDataIndex + bytesSent, curBytesSent);
                    bytesSent += curBytesSent;
                    rtpPacketPtr->buffer[BYTES_FOR_RTP_HEADER] = FU_indicator;
                    rtpPacketPtr->buffer[BYTES_FOR_RTP_HEADER + 1u] = FU_header;
                    if (bytesSent == RAW_H264_PAYLOAD_MAX_LEN and isFirstPacketOfCurFrame)
                    // the first disparted package of current frame
                    {
                        rtpPacketPtr->buffer[BYTES_FOR_RTP_HEADER + 1] |= 0b10000000u;
                    }
                    else if (bytesSent == (bufLen - curRawDataIndex) and !WITH_EXTRA_PACKAGE(bufLen))
                    // the last disparted package of current frame
                    {
                        packetBeginFlag = false;
                        isLastPacketOfCurFrame = 1;
                        rtpPacketPtr->buffer[BYTES_FOR_RTP_HEADER + 1] |= 0b01000000u;
                    }
                    connectionCtrlPtr->sendRTPPackage(rtpPacketPtr, curFrameNumber, isLastPacketOfCurFrame);
                }
            }
        }
    }
}
#endif
