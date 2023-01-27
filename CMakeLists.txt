cmake_minimum_required(VERSION 3.0.0)
project(VideoSender VERSION 0.1.0)

SET(CMAKE_CXX_FLAGS "-std=c++17")
add_executable(${PROJECT_NAME} main.cpp StreamingController.cpp ConnectionController.cpp)
find_package(Boost COMPONENTS program_options REQUIRED )
list(APPEND CMAKE_PREFIX_PATH "/usr/local/share/cmake/nlohmann_json")
find_package(Threads REQUIRED)
find_package(nlohmann_json REQUIRED)

#add_library(libdatachannel SHARED IMPORTED)
#set_target_properties(libdatachannel PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/third_party/libdatachannel/libdatachannel.so )

target_include_directories(${PROJECT_NAME} PUBLIC 
                        ${Boost_INCLUDE_DIR}
                        ${CMAKE_CURRENT_SOURCE_DIR}/third_party
                        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/libdatachannel/include    
                        /usr/include/
                        )
target_link_libraries(${PROJECT_NAME}
                    Threads::Threads 
                    nlohmann_json 
                    ${Boost_LIBRARIES}
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/libdatachannel/libdatachannel.so
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/libopenal.so.1
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/libAudioRender.so
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/libhpr.so
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/libHCCore.so
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/libcrypto.so.1.1
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/libNPQos.so
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/libz.so
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/libSuperRender.so
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/libPlayCtrl.so
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/libssl.so.1.1
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/libhcnetsdk.so
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/HCNetSDKCom/libanalyzedata.so
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/HCNetSDKCom/libAudioIntercom.so
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/HCNetSDKCom/libHCAlarm.so
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/HCNetSDKCom/libHCCoreDevCfg.so
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/HCNetSDKCom/libHCDisplay.so
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/HCNetSDKCom/libHCGeneralCfgMgr.so
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/HCNetSDKCom/libHCPlayBack.so
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/HCNetSDKCom/libHCPreview.so
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/HCNetSDKCom/libHCIndustry.so
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/HCNetSDKCom/libHCVoiceTalk.so
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/HCNetSDKCom/libiconv.so.2
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/HCNetSDKCom/libStreamTransClient.so
                    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hikvision/HCNetSDKCom/libSystemTransform.so
                    )
