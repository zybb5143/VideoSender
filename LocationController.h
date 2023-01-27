#ifndef LOCATION_CONTROLLER_H
#define LOCATION_CONTROLLER_H
#include <stdio.h>
#include <iostream>
#include <thread>
#include <random>
#include <chrono>
using namespace std::chrono_literals;
class LocationController
{
public:
    double x, y;
    std::atomic_bool stopFlag{false};
    std::thread t;
    LocationController()
    {

        x = 0;
        y = 0;
        stopFlag = false;
        t = std::thread([&]()
                        {
                while(!stopFlag)
                {
                    std::this_thread::sleep_for(100ms);
                    x += rand() % 10 * 0.0005;
                    y += rand() % 10 * 0.0005;
                } });
    }
    std::pair<double, double> getRealTimeLocation()
    {
        return {x, y};
    }
    ~LocationController()
    {
        stopFlag = true;
        t.join();
    }
};
#endif