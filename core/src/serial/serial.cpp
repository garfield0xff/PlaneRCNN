#include "serial/serial.h"
#include "serial/serial_unix.h"

namespace vl {
namespace core {
namespace serial {

Serial::Serial( const std::string &port, uint32_t baudrate, Timeout timeout,
                bytesize_t bytesize, parity_t parity, 
                stopbits_t stopbits,flowcontrol_t flowcontrol)
        : pimpl_(new SerialImpl(port, baudrate, bytesize, parity, 
                                stopbits, flowcontrol)) 
{
    pimpl_->setTimeout(timeout);
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

size_t Serial::read(std::string &buffer, size_t size) {
    std::vector<uint8_t> buffer_(size);
    size_t bytes_read = this->pimpl_->read(buffer_.data(), size);
    buffer.append(reinterpret_cast<const char *>(buffer_.data()), bytes_read);
    return bytes_read;
}

size_t Serial::write(const std::vector<uint8_t> &data) {
    return this->write_(&data[0], data.size());
}

size_t Serial::write_(const uint8_t *data, size_t length)
{
    printf("write_\n");
    return pimpl_->write(data, length);
}

}// namespace serial
}// namespace common
}// namespace vl
