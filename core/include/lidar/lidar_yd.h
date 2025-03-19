#ifndef LIDAR_YD_
#define LIDAR_YD_

#include "lidar_base.h"

namespace vl {
namespace core {
namespace lidar {

class YDLidarController : BaseLidarController {
public:
    result_t connect(const char *port_path, uint32_t buadrate = 8000);
    void disconnect();


}; 
} // namespace lidar
} // namespace core
} // namespace vl

#endif
