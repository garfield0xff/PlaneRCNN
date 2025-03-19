#include "serial/serial.h"
#include "serial/serial_unix.h"

namespace vl {
namespace core {
namespace serial {

Serial::Serial( const std::string &port, uint32_t baudrate,
                bytesize_t bytesize, parity_t parity, 
                stopbits_t stopbits,flowcontrol_t flowcontrol)
        : pimpl_(new SerialImpl(port, baudrate, bytesize, parity, 
                                stopbits, flowcontrol)) 
{

}

Serial::~Serial() 
{
    delete pimpl_;
}

bool Serial::open()
{
    return pimpl_->open();
}

bool Serial::isOpen()
{
    return pimpl_->isOpen();
}

void Serial::closePort() {
    return pimpl_->close();
}

size_t Serial::write(const std::vector<uint8_t> &data)
{
    
}

size_t Serial::write_(const uint8_t *data, size_t length)
{
    return pimpl_->write(data, length);
}

}// namespace serial
}// namespace common
}// namespace vl
