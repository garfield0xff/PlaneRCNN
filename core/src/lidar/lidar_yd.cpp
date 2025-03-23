#include "lidar/lidar_yd.h"

namespace vl{
 using namespace core::lidar;
 using namespace core::serial;

 YDLidarController::~YDLidarController() {
   disableDataGrabbing();

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

    m_buadrate = buadrate;
    m_port = string(port_path);
    m_isConnected = true;

    m_serial = new Serial(m_port, m_buadrate, Timeout::simpleTimeout(1000));

    if(!m_serial->open()) {
        return RESULT_FAIL;
    }
    
    return RESULT_OK;
 }

 void YDLidarController::disableDataGrabbing() {
   m_isScanning = false;
   if(m_thread) {
      if(m_thread->joinable())
         m_thread->join();
      delete m_thread;
      m_thread = nullptr;
   }
 }
 
 void YDLidarController::disconnect() {
    if(!m_isConnected) {
        return;
    }

    stopScan();

    m_isConnected = false;
 }

 result_t YDLidarController::startScan() {
    result_t ret;
    ret = sendCommand(YDLIDAR_START_SCAN);
    ret = createThread();
    return ret;
 }

 result_t YDLidarController::stopScan() {
    delay(30);
    result_t ret;
    ret = sendCommand(YDLIDAR_STOP_SCAN);  
    return ret;
 }

 result_t YDLidarController::createThread() {
   m_thread = new std::thread(&YDLidarController::cacheScanData, this);
   if(!m_thread) {
      return RESULT_FAIL;
   }
   return RESULT_OK;
 }

 int YDLidarController::cacheScanData() {
   node_info local_buf[YD_PACKMAXNODES];
   size_t count = YD_PACKMAXNODES;
   node_info local_scan[MAX_SCAN_NODES];
   result_t ans  = RESULT_FAIL;
   ::memset(local_scan, 0, sizeof(local_scan));

   int timeout_count = 0;
   m_isScanning = true;

   while(m_isScanning) {
      ans = waitScanData(local_buf, count, 1000);
      

      for(size_t pos = 0; pos < count; ++pos) {
         
      }
   }
   return 0;
 }

 result_t YDLidarController::waitScanData(node_info *nodebuffer, size_t &count, uint32_t timeout) {
   if(!m_isConnected)  {
      count = 0;
      return RESULT_FAIL;
   }

   
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
    m_serial->write_(data, size);
    return RESULT_OK;
 }

 size_t YDLidarController::getData(uint8_t *data, size_t size) {
   if(!m_isConnected) {
      return RESULT_FAIL;
   }
   
   size_t r = 0;
   while(size) {
      r = m_serial->read(data, size);
      if ( r < 1 ) {
         return RESULT_FAIL;
      }

      size -= r;
      data += r;
   }
   return RESULT_OK;
 }

 

 size_t YDLidarController::parseResponseHeader(uint8_t *packageBuffer, uint32_t timeout)
 {
   int recvPos = 0;
   uint32_t startTs = getms();
   uint32_t waitTime = 0;

   result_t ans = RESULT_TIMEOUT;
   
   // while (waitTime = getms() - startTs <= timeout)
   // {
   //    size_t remainSize = YD_PACKHEADSIZE - recvPos;
   //    size_t recvSize = 0;
      
   // }
 }

 
} 