#ifndef SERIAL_H
#define SERIAL_H

#include <string>

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

class Serial {
public:
    explicit Serial(const std::string &port = "",
                    uint32_t baudrate = 9600,
                    bytesize_t bytesize = eightbits,
                    parity_t parity = parity_none,
                    stopbits_t stopbits = stopbits_one,
                    flowcontrol_t flowcontrol = flowcontrol_none );

    virtual ~Serial();

    virtual bool open();

    virtual bool isOpen();

    virtual void closePort();

    size_t read(std::string &buffer, size_t size = 1);
    
    size_t write(const std::vector<uint8_t> &data);

    size_t write_(const uint8_t *data, size_t length);
    

private:
    class SerialImpl; 
    SerialImpl *pimpl_;
};
}// namespace serial
}//namespace core
}// namespace vl


#endif