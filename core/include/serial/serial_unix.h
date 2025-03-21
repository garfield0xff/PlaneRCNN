#ifndef SERIAL_UNIX_H
#define SERIAL_UNIX_H

#include <termios.h>
#include <cstring>

#include "serial/serial.h"

namespace vl {
namespace core {
namespace serial {

class MillisecondTimer {
    public:
     explicit MillisecondTimer(const uint32_t millis);
     int64_t remaining();

    private:
     static timespec timespec_now();
     timespec expiry;
};

class Serial::SerialImpl {

public:
explicit SerialImpl(const std::string &port,
                    unsigned long buadrate,
                    bytesize_t bytesize,
                    parity_t parity,
                    stopbits_t stopbits,
                    flowcontrol_t flowcontrol);

bool open();

void close();

bool isOpen();

size_t read(uint8_t *buf, size_t size = 1);

size_t write(const uint8_t *data, size_t length);

bool setBaudrate(unsigned long buadrate);

bool setStandardBaudrate(unsigned long baudrate);

bool setTermios(const termios *tio);

bool getTermios(termios *tio);

void setTimeout(Timeout &timeout);

private:
    std::string port_;
    int fd_;
    pid_t pid;

    bool is_open_;

    unsigned long baudrate_;
    Timeout  timeout_;
    uint32_t byte_time_ns;

    parity_t parity_;           
    bytesize_t bytesize_;       
    stopbits_t stopbits_;       
    flowcontrol_t flowcontrol_; 

};
}
}
}



#endif