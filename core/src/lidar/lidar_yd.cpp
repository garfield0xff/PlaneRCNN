#include "lidar_yd.h"

namespace vl{
 using namespace core::lidar;

 result_t YDLidarController::connect(const char *port_path, uint32_t buadrate) {
    m_buadrate = buadrate;
    m_port = string(port_path);
    return RESULT_OK;
 }
} 