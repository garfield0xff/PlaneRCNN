

#include <unistd.h>
#include <fcntl.h>

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

#include "serial/serial_unix.h"


namespace vl{
namespace core {
namespace serial {

using serial::MillisecondTimer;

MillisecondTimer::MillisecondTimer(const uint32_t millis) : expiry(
    timespec_now()) {
  int64_t tv_nsec = expiry.tv_nsec + (millis * 1e6);

  if (tv_nsec >= 1e9) {
    int64_t sec_diff = tv_nsec / static_cast<int>(1e9);
    expiry.tv_nsec = tv_nsec % static_cast<int>(1e9);
    expiry.tv_sec += sec_diff;
  } else {
    expiry.tv_nsec = tv_nsec;
  }
}

int64_t MillisecondTimer::remaining() {
  timespec now(timespec_now());
  int64_t millis = (expiry.tv_sec - now.tv_sec) * 1e3;
  millis += (expiry.tv_nsec - now.tv_nsec) / 1e6;
  return millis;
}

timespec MillisecondTimer::timespec_now() {
  timespec time;
# ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
  clock_serv_t cclock;
  mach_timespec_t mts;
  host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &cclock);
  clock_get_time(cclock, &mts);
  mach_port_deallocate(mach_task_self(), cclock);
  time.tv_sec = mts.tv_sec;
  time.tv_nsec = mts.tv_nsec;
# else
  clock_gettime(CLOCK_MONOTONIC, &time);
# endif
  return time;
}

timespec timespec_from_ms(const uint32_t millis) {
  timespec time;
  time.tv_sec = millis / 1e3;
  time.tv_nsec = (millis - (time.tv_sec * 1e3)) * 1e6;
  return time;
}

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

    is_open_ = true;

    return true;
}

bool Serial::SerialImpl::setBaudrate(unsigned long baudrate) {
    termios tio;

    if(!getTermios(&tio)) {
        return false;
    }

    if(fd_ == -1) {
        return false;
    }
    baudrate_ = baudrate;

    if(::cfsetispeed(&tio, B230400) < 0) {
        return false;
    };
    if(::cfsetospeed(&tio, B230400) < 0) {
        return false;
    };

    return setTermios(&tio);
}

void Serial::SerialImpl::close() {
    if(isOpen()) {
        is_open_ = false;
    } 
    
    if(fd_ != -1) {
        ::close(fd_);
    }

    fd_ = -1;
}

bool Serial::SerialImpl::isOpen() {
    return is_open_;
}

size_t Serial::SerialImpl::read(uint8_t *buf, size_t size) {

}

size_t Serial::SerialImpl::write(const uint8_t *data, size_t length) {
    printf("write\n");
    if(is_open_ == false) {
        return 0;
    }

    for(int i = 0; i < length; i++) {
        printf("buffer : 0x%X\n", data[i]);
    }

    fd_set write_fds;
    size_t bytes_written = 0;
    long total_timeout_ms = timeout_.write_timeout_constant;
    total_timeout_ms += timeout_.write_timeout_multiplier * static_cast<long>(length);
    MillisecondTimer total_timeout(total_timeout_ms);

    bool first_iteration = true;

    while(bytes_written < length) {
        int64_t timeout_remaining_ms = total_timeout.remaining();
        if(!first_iteration && (timeout_remaining_ms <= 0)) {
            break;
        }

        first_iteration = false;

        timespec timeout(timespec_from_ms(timeout_remaining_ms));

        FD_ZERO(&write_fds);
        FD_SET(fd_, &write_fds);

        // pselct(nfds - 1) -> pselect(fd_ + 1)
        int r = pselect(fd_ + 1, NULL, &write_fds, NULL, &timeout, NULL);

        if(r==0) {
            break;
        }

        if(r > 0) {
            if(FD_ISSET(fd_, &write_fds)) {
                ssize_t bytes_written_now = ::write(fd_, data + bytes_written, length - bytes_written);
                if(bytes_written_now < 0) {
                    continue;
                }
                bytes_written += static_cast<size_t>(bytes_written_now);
                if (bytes_written == length) {
                    printf("bytes == length\n");
                    break;
                }
                if (bytes_written < length) {
                    printf("bytes < length\n");
                    continue;
                }
                if (bytes_written > length) {
                    printf("bytes > length\n");
                    break;
                } 
            }
            break;
        }
    }    
    return bytes_written;
}

bool Serial::SerialImpl::setStandardBaudrate(unsigned long baudrate) {
    if(fd_ == -1) {
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

void Serial::SerialImpl::setTimeout(Timeout &timeout) {
    timeout_ = timeout;
}


}// namespace serial
}// namespace common
}// namespace vl