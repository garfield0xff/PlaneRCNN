

#include <unistd.h>
#include <fcntl.h>

#include "serial_unix.h"

namespace vl{
namespace core {
namespace serial {

Serial::SerialImpl::SerialImpl(const std::string &port,
                               unsigned long buadrate,
                               bytesize_t bytesize,
                               parity_t parity,
                               stopbits_t stopbits,
                               flowcontrol_t flowcontrol)
    :port_(port), fd_(-1), pid(-1), is_open_(false), parity_(parity),
     bytesize_(bytesize), stopbits_(stopbits), flowcontrol_(flowcontrol)
{

};


static inline void set_common_props(termios *tio) 
{
    tio->c_cflag |= CLOCAL | CREAD;
    tio->c_cc[VTIME] = 0;
    tio->c_cc[VMIN] = 0;
}

static inline void set_databits(termios *tio, serial::bytesize_t databits) 
{
    tio->c_cflag &= ~CSIZE;
  
    switch (databits) {
      case serial::fivebits:
        tio->c_cflag |= CS5;
        break;
  
      case serial::sixbits:
        tio->c_cflag |= CS6;
        break;
  
      case serial::sevenbits:
        tio->c_cflag |= CS7;
        break;
  
      case serial::eightbits:
        tio->c_cflag |= CS8;
        break;
  
      default:
        tio->c_cflag |= CS8;
        break;
    }
}

static inline void set_parity(termios *tio, serial::parity_t parity) 
{
    tio->c_iflag &= ~(PARMRK | INPCK);
    tio->c_iflag |= IGNPAR;
  
    switch (parity) {
  
  #ifdef CMSPAR
      // Here Installation parity only for GNU/Linux where the macro CMSPAR.
      case serial::parity_space:
        tio->c_cflag &= ~PARODD;
        tio->c_cflag |= PARENB | CMSPAR;
        break;
  
      case serial::parity_mark:
        tio->c_cflag |= PARENB | CMSPAR | PARODD;
        break;
  #endif
  
      case serial::parity_none:
        tio->c_cflag &= ~PARENB;
        break;
  
      case serial::parity_even:
        tio->c_cflag &= ~PARODD;
        tio->c_cflag |= PARENB;
        break;
  
      case serial::parity_odd:
        tio->c_cflag |= PARENB | PARODD;
        break;
  
      default:
        tio->c_cflag |= PARENB;
        tio->c_iflag |= PARMRK | INPCK;
        tio->c_iflag &= ~IGNPAR;
        break;
    }
  }

  static inline void set_stopbits(termios *tio, serial::stopbits_t stopbits) {
    switch (stopbits) {
      case serial::stopbits_one:
        tio->c_cflag &= ~CSTOPB;
        break;
  
      case serial::stopbits_two:
        tio->c_cflag |= CSTOPB;
        break;
  
      default:
        tio->c_cflag &= ~CSTOPB;
        break;
    }
  }
  
static inline void set_flowcontrol(termios *tio,
                                   serial::flowcontrol_t flowcontrol) 
{
    switch (flowcontrol) {
      case serial::flowcontrol_none:
        tio->c_cflag &= ~CRTSCTS;
        tio->c_iflag &= ~(IXON | IXOFF | IXANY);
        break;
  
      case serial::flowcontrol_hardware:
        tio->c_cflag |= CRTSCTS;
        tio->c_iflag &= ~(IXON | IXOFF | IXANY);
        break;
  
      case serial::flowcontrol_software:
        tio->c_cflag &= ~CRTSCTS;
        tio->c_iflag |= IXON | IXOFF | IXANY;
        break;
  
      default:
        tio->c_cflag &= ~CRTSCTS;
        tio->c_iflag &= ~(IXON | IXOFF | IXANY);
        break;
    }
}
  
bool Serial::SerialImpl::open() {
    if(port_.empty()) {
        return false;
    }

    if(isOpen()) {
        return true;
    }

    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_APPEND | O_NDELAY);

    termios tio;

    if(!getTermios(&tio)) {
        close();
        return false;
    }

    set_common_props(&tio);
    set_databits(&tio, bytesize_);
    set_parity(&tio, parity_);
    set_stopbits(&tio, stopbits_);
    set_flowcontrol(&tio, flowcontrol_);

    if(!setTermios(&tio)) {
        close();
        return false;
    }

    if(!setBaudrate(baudrate_)) {
        close();
        return false;
    }

    
}

bool Seiral::SerialImpl::setBuadrate(unsigned long baudrate) {
    if(fd_ == -1) {
        return false;
    }
    baudrate_ = baudrate;
}

void Serial::SerialImpl::close() {
    if(isOpen()) {
        is_open_ = false;
    } 
    
    if(fd_ != -1) {
        ::close(fd_);
    }

    fd = -1;
}

bool Serial::SerialImpl::isOpen() {
    return is_open_;
}

size_t Serial::SerialImpl::read(uint8_t *buf, size_t size = 1) {

}

size_t Serial::SerialImpl::write(const uint8_t *data, size_t length) {
    if(is_open_ = false) {
        return 0;
    }

    fd_set writefds;
    size_t bytes_written = 0;


}

bool Serial::SerialImpl::setStandardBaudrate(unsigned long baudrate) {
    if(fd_ = -1) {
        return false;
    }
    baudrate_ = baudrate;

    return true;
}

bool Serial::SerialImpl::setTermios(const termios *tio) {

    tcflush(fd_, TCIFLUSH);

    if (fcntl(fd_, F_SETFL, FNDELAY)) {
        return false;
    }

    if(::tcsetattr(fd_, TCSANOW, tio) == -1) {
        return false;
    }

    return true;
}

bool Serial::SerialImpl::getTermios(termios *tio) {
    ::memset(tio, 0, sizeof(termios));

    if(::tcgetattr(fd_, tio) == -1) {
        return false;
    }

    return true;
}




}// namespace serial
}// namespace common
}// namespace vl