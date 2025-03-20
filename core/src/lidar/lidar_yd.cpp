#include "lidar/lidar_yd.h"

namespace vl{
 using namespace core::lidar;
 using namespace core::serial;

 YDLidarController::~YDLidarController() {
    if(m_serial) {
        if(m_serial->isOpen()) {
            m_serial->closePort();
        }
        if(m_serial) {
            delete m_serial;
            m_serial = NULL;
        }
    }
 }

 result_t YDLidarController::connect(const char *port_path, uint32_t buadrate) {
    printf("connect\n");
    m_buadrate = buadrate;
    m_port = string(port_path);
    m_isConnected = true;

    m_serial = new Serial(m_port, m_buadrate, Timeout::simpleTimeout(1000));

    if(!m_serial->open()) {
        return RESULT_FAIL;
    }
    

    return RESULT_OK;
 }
 
 void YDLidarController::disconnect() {
    printf("disconnect\n");
    if(!m_isConnected) {
        return;
    }

    stopScan();

    m_isConnected = false;
 }

 bool YDLidarController::startScan() {
    printf("startScan\n");
    result_t ret = sendCommand(YDLIDAR_START_SCAN);
    return true;
 }

 bool YDLidarController::stopScan() {
    printf("stopScan\n");
    result_t ret = sendCommand(YDLIDAR_STOP_SCAN);
    return true;
 }

 size_t YDLidarController::sendCommand(uint8_t cmd, const void *payload, size_t payloadsize) {
    uint8_t pkt_header[10];
    cmd_packet *header = reinterpret_cast<cmd_packet *>(pkt_header);

    if (!m_isConnected)
    {
      return RESULT_FAIL;
    }

    header->syncByte = YDLIDAR_START_BIT;
    header->cmd_flag = cmd;
    sendData(pkt_header, 2);

    return RESULT_OK;
 }

 size_t YDLidarController::sendData(const uint8_t *data, size_t size) {
    printf("sendData\n");
    m_serial->write_(data, size);
    return RESULT_OK;
 }

 
} 