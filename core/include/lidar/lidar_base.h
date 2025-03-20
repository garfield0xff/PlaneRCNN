#ifndef LIDAR_CONTROLLER_H_
#define LIDAR_CONTROLLER_H_

#include "std.h"
#include "lidar_protocol.h"

namespace vl {
namespace core {
namespace lidar {

class BaseLidarController {
public:
    BaseLidarController() : m_port(""),
                        m_buadrate(8000)
    {
        m_isScanning = false;
    }

    virtual result_t connect(const char *port_path, uint32_t buadrate = 8000) = 0;
    virtual void disconnect() = 0;

    
    protected :
        bool m_isScanning;
        bool m_isConnected;
        std::string m_port;
        uint32_t m_buadrate;

};
} // namespace lidar
} // namespace core
} // namespace vl

#endif // LIDAR_CONTROLLER_H_