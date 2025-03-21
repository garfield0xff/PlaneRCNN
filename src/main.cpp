#include "lidar/lidar_yd.h"
#include <unistd.h>

using namespace vl::core::lidar;

int main()
{
    YDLidarController *controller = new YDLidarController();

    #if defined(__APPLE__)
    controller->connect("/dev/tty.usbserial-0001");  
    #elif defined(__linux__)
        controller->connect("/dev/ttyUSB0"); 
    #else
        #error "Unsupported OS"
    #endif


    controller->startScan();
    sleep(10);
    controller->disconnect();
    return 0;
}