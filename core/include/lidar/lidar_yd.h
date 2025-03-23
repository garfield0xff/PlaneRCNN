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

    void disableDataGrabbing();

    void disconnect();

    result_t startScan();

    result_t stopScan();


    result_t createThread();

    int cacheScanData();

    result_t waitScanData(node_info *nodebuffer, size_t &count, uint32_t timeout);

    size_t sendCommand(uint8_t cmd, const void *payload = NULL, size_t payloadsize = 0);
    
    size_t sendData(const uint8_t *data, size_t size);

    size_t getData(uint8_t *data, size_t size);
    
    size_t parseResponseHeader(uint8_t *packageBuffer, uint32_t timeout);

private:
    serial::Serial *m_serial;
}; 
} // namespace lidar
} // namespace core
} // namespace vl

#endif
