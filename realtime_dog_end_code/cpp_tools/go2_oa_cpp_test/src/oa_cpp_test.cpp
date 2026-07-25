#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>

using namespace unitree::robot;
using namespace unitree::robot::go2;

int main(int argc, char** argv)
{
    std::string iface;

    if (argc > 1) {
        iface = argv[1];
    }

    try {
        std::cout << "[oa_cpp_test] init ChannelFactory..." << std::endl;

        if (!iface.empty()) {
            std::cout << "[oa_cpp_test] interface: " << iface << std::endl;
            ChannelFactory::Instance()->Init(0, iface);
        } else {
            std::cout << "[oa_cpp_test] default interface" << std::endl;
            ChannelFactory::Instance()->Init(0);
        }

        std::cout << "[oa_cpp_test] create ObstaclesAvoidClient..." << std::endl;

        ObstaclesAvoidClient client;
        client.SetTimeout(3.0f);
        client.Init();

        std::cout << "[oa_cpp_test] SwitchSet(true)..." << std::endl;
        int32_t ret = client.SwitchSet(true);
        std::cout << "[oa_cpp_test] SwitchSet ret = " << ret << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::cout << "[oa_cpp_test] UseRemoteCommandFromApi(true)..." << std::endl;
        ret = client.UseRemoteCommandFromApi(true);
        std::cout << "[oa_cpp_test] UseRemoteCommandFromApi ret = " << ret << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(800));

        std::cout << "[oa_cpp_test] Move(0.5, 0.0, 0.0) for 1s..." << std::endl;
        ret = client.Move(0.5f, 0.0f, 0.0f);
        std::cout << "[oa_cpp_test] Move ret = " << ret << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "[oa_cpp_test] Stop Move..." << std::endl;
        ret = client.Move(0.0f, 0.0f, 0.0f);
        std::cout << "[oa_cpp_test] Stop ret = " << ret << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        std::cout << "[oa_cpp_test] UseRemoteCommandFromApi(false)..." << std::endl;
        ret = client.UseRemoteCommandFromApi(false);
        std::cout << "[oa_cpp_test] Release ret = " << ret << std::endl;

        std::cout << "[oa_cpp_test] DONE" << std::endl;
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "[oa_cpp_test] exception: " << e.what() << std::endl;
        return 1;
    }
    catch (...) {
        std::cerr << "[oa_cpp_test] unknown exception" << std::endl;
        return 2;
    }
}
