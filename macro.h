#pragma once
#define MSG_TYPE_STOP_VIDEO "stopVideo"
#define MSG_TYPE_CLOSE_CONNECTION "closeConnection"
#define MSG_TYPE_LOCATION "location"
#define MSG_TYPE_USER_CLOSE_VIDEO_CONNECTION "userCloseVideoConnection"
#define MSG_TYPE_USER_STOP_CUSTOM_SERVICE "userStopCustomServiceRequest"
#define MSG_TYPE_USER_CUSTOM_SERVICE_REQUEST "customServiceRequest"
#define MSG_TYPE_USER_ESTABLISH_CONNECTION "establishConnection"
#define MSG_TYPE_SERVICE_STUFF_STOP_SERVICE "customServiceStopCustomService"
#define MSG_TYPE_SERVICE_STUFF_ACCEPT_REQUEST "acceptCustomServiceRequest"


const int connectorDebugFlag = true;
#define STREAMING_DEBUG
#define CONNECTOR_DEBUG
#define ENABLE_STREAMING 0