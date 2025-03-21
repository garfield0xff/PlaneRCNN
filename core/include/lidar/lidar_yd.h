#ifndef LIDAR_YD_
#define LIDAR_YD_

#include "lidar_base.h"
#include "serial/serial.h"

namespace vl {
namespace core {
namespace lidar {

class YDLidarController : BaseLidarController {
public:
    YDLidarController() {};
    virtual ~YDLidarController();

    result_t connect(const char *port_path, uint32_t buadrate = 8000);

    void disconnect();

    bool startScan();

    bool stopScan();

    size_t sendCommand(uint8_t cmd, const void *payload = NULL, size_t payloadsize = 0);
    
    size_t sendData(const uint8_t *data, size_t size);

private:
    serial::Serial *m_serial;


}; 
} // namespace lidar
} // namespace core
} // namespace vl

#endif
