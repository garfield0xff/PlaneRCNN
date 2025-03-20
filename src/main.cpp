#include "lidar/lidar_yd.h"
#include <unistd.h>

using namespace vl::core::lidar;

int main()
{
    YDLidarController *controller = new YDLidarController();

    controller->connect("/dev/tty.usbserial-0001");
    controller->startScan();
    sleep(10);
    controller->disconnect();
    return 0;
}