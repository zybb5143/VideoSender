#include "ConnectionController.h"
#include "StreamingController.h"
#include <thread>
#include <chrono>
#include <boost/program_options.hpp>
using namespace std::chrono_literals;
namespace po = boost::program_options;

int main()
{
    po::options_description description;
    description.add_options()("turn_ip", po::value<std::string>())("turn_port", po::value<uint16_t>())("turn_credential", po::value<std::string>())("turn_user_name", po::value<std::string>())("websocket_ip", po::value<std::string>())("websocket_path", po::value<std::string>())("websocket_port", po::value<uint16_t>())("hik_resolution", po::value<uint16_t>()->default_value(19u))("hik_video_frame_rate", po::value<uint16_t>()->default_value(13u))("hik_interval_BP_frame", po::value<uint16_t>()->default_value(2u))("hik_video_bit_rate", po::value<uint16_t>()->default_value(24u))("interval_frame_I", po::value<uint16_t>()->default_value(5u))("frame_rate", po::value<int>()->default_value(20))("device_user_name", po::value<std::string>()->default_value(""))("device_password", po::value<std::string>())("device_address", po::value<std::string>())("local_address", po::value<std::string>())("device_id", po::value<int>());
    po::variables_map vm;
    po::store(po::parse_config_file("../streaming.cfg", description), vm);
    po::notify(vm);
    ConnectionControllerPtr connectionCtrl = std::make_shared<ConnectionController>(
        vm["websocket_ip"].as<std::string>(),
        vm["websocket_port"].as<uint16_t>(),
        vm["websocket_path"].as<std::string>(),
        vm["turn_ip"].as<std::string>(),
        vm["turn_port"].as<uint16_t>(),
        vm["turn_user_name"].as<std::string>(),
        vm["turn_credential"].as<std::string>(),
        vm["frame_rate"].as<int>(),
        vm["device_id"].as<int>());
    LocationControllerPtr locationCtrl = std::make_shared<LocationController>();
    connectionCtrl->registerLocationController(locationCtrl);
    StreamingController sc(vm);
    sc.registerConnectionController(connectionCtrl);
    std::this_thread::sleep_for(3600000ms);
}