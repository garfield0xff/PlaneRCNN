#ifndef SERIAL_H
#define SERIAL_H

#include "logger.h"

#include <string>
#include <vector>
#include <sys/ioctl.h>

namespace vl {
namespace core {
namespace serial {

typedef enum  {
    fivebits = 5,
    sixbits = 6,
    sevenbits = 7,
    eightbits = 8
} bytesize_t;

typedef enum {
    parity_none = 0,
    parity_odd = 1,
    parity_even = 2,
    parity_mark = 3,
    parity_space = 4
} parity_t;

typedef enum {
    stopbits_one = 1,
    stopbits_two = 2,
    stopbits_one_point_five
} stopbits_t;

typedef enum {
    flowcontrol_none = 0,
    flowcontrol_software,
    flowcontrol_hardware
} flowcontrol_t;

struct Timeout {
    static constexpr uint32_t max_value = std::numeric_limits<uint32_t>::max();

    static constexpr uint32_t max() {
        return max_value;
    }

    static Timeout simpleTimeout(uint32_t timeout) {
        return Timeout(max(), timeout, 0, timeout, 0);
    }

    uint32_t inter_byte_timeout;
    uint32_t read_timeout_constant;
    uint32_t read_timeout_multiplier;
    uint32_t write_timeout_constant;
    uint32_t write_timeout_multiplier;

    Timeout(uint32_t inter_byte_timeout_= 0,
            uint32_t read_timeout_constant_ = 0,
            uint32_t read_timeout_multiplier_ = 0,
            uint32_t write_timeout_constant_ = 0,
            uint32_t write_timeout_multiplier = 0)
            : inter_byte_timeout(inter_byte_timeout_),
              read_timeout_constant(read_timeout_constant_),
              read_timeout_multiplier(read_timeout_multiplier_),
              write_timeout_constant(write_timeout_constant_),
              write_timeout_multiplier(write_timeout_multiplier) {}
};

class Serial {
public:
    explicit Serial(const std::string &port = "",
                    uint32_t baudrate = 9600,
                    Timeout timeout = Timeout(),
                    bytesize_t bytesize = eightbits,
                    parity_t parity = parity_none,
                    stopbits_t stopbits = stopbits_one,
                    flowcontrol_t flowcontrol = flowcontrol_none );

    virtual ~Serial();

    virtual bool open();

    virtual bool isOpen();

    virtual void closePort();

    size_t read(uint8_t *bufer, size_t size);

    size_t read(std::string &buffer, size_t size = 1);
    
    size_t write(const std::vector<uint8_t> &data);

    size_t write_(const uint8_t *data, size_t length);
    

private:
    class SerialImpl; 
    SerialImpl *pimpl_;
    
    uint8_t* globalRecvBuffer;
};
}// namespace serial
}//namespace core
}// namespace vl


#endif