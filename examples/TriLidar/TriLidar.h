#pragma once

#include "Arduino.h"
#include "inc/v8stdint.h"


#define LIDAR_CMD_STOP                      0x65
#define LIDAR_CMD_SCAN                      0x60
#define LIDAR_CMD_FORCE_SCAN                0x61
#define LIDAR_CMD_RESET                     0x80
#define LIDAR_CMD_FORCE_STOP                0x00
#define LIDAR_CMD_GET_EAI                   0x55
#define LIDAR_CMD_GET_DEVICE_INFO           0x90
#define LIDAR_CMD_GET_DEVICE_HEALTH         0x91
#define LIDAR_CMD_SYNC_BYTE                 0xA5
#define LIDAR_CMDFLAG_HAS_PAYLOAD           0x8000

#define LIDAR_ANS_TYPE_DEVINFO              0x4
#define LIDAR_ANS_TYPE_DEVHEALTH            0x6
#define LIDAR_ANS_SYNC_BYTE1                0xA5
#define LIDAR_ANS_SYNC_BYTE2                0x5A
#define LIDAR_ANS_TYPE_MEASUREMENT          0x81

#define LIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define LIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1
#define LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT    8


#define LIDAR_CMD_RUN_POSITIVE             0x06
#define LIDAR_CMD_RUN_INVERSION            0x07
#define LIDAR_CMD_SET_AIMSPEED_ADDMIC      0x09
#define LIDAR_CMD_SET_AIMSPEED_DISMIC      0x0A
#define LIDAR_CMD_SET_AIMSPEED_ADD         0x0B
#define LIDAR_CMD_SET_AIMSPEED_DIS         0x0C
#define LIDAR_CMD_GET_AIMSPEED             0x0D
#define LIDAR_CMD_SET_SAMPLING_RATE        0xD0
#define LIDAR_CMD_GET_SAMPLING_RATE        0xD1

#define LIDAR_STATUS_OK                    0x0
#define LIDAR_STATUS_WARNING               0x1
#define LIDAR_STATUS_ERROR                 0x2

#define PackageSampleBytes 2
#define PackageSampleMaxLngth 0x40
#define Node_Default_Quality (10<<2)
#define Node_Sync 1
#define Node_NotSync 2
#define PackagePaidBytes 10
#define PH 0x55AA
#define PH1 0xAA
#define PH2 0x55

#define YDLIDAR_MOTOR_SCTP 3 // The PWM pin for control the speed of YDLIDAR's motor.
#define YDLIDAR_MOTOR_EN 7 // The ENABLE PIN for YDLIDAR's motor

//CT定义
typedef enum {
  CT_Normal = 0,
  CT_RingStart  = 1,
  CT_Tail,
} CT;

struct node_info {
  uint8_t sync_flag;
  uint8_t quality;
  uint16_t angle;
  uint16_t distance;
} __attribute__((packed)) ;

//单包数据头
struct node_package_head {
  uint16_t  package_Head;
  uint8_t   package_CT;
  uint8_t   nowPackageNum;
  uint16_t  packageFirstSampleAngle;
  uint16_t  packageLastSampleAngle;
  uint16_t  checkSum;
} __attribute__((packed));
//单点（不带信号强度）
struct node_point {
  uint16_t is : 2;
  uint16_t dist : 14;
} __attribute__((packed));
//单点（带信号强度）
struct node_point_i {
  uint8_t quality;
  uint16_t is : 2;
  uint16_t dist : 14;
} __attribute__((packed));

//单包点云数据（不带信号强度）
struct node_package {
  node_package_head head;
  node_point points[PackageSampleMaxLngth];
} __attribute__((packed));

//单包点云数据（带信号强度）
struct node_package_i {
  struct node_package_head head;
  node_point_i points[PackageSampleMaxLngth];
} __attribute__((packed));

struct device_info {
  uint8_t   model;
  uint16_t  firmware_version;
  uint8_t   hardware_version;
  uint8_t   serialnum[16];
} __attribute__((packed)) ;

struct device_health {
  uint8_t   status;
  uint16_t  error_code;
} __attribute__((packed))  ;

struct sampling_rate {
  uint8_t rate;
} __attribute__((packed))  ;

struct scan_frequency {
  uint32_t frequency;
} __attribute__((packed))  ;

struct scan_rotation {
  uint8_t rotation;
} __attribute__((packed))  ;

struct cmd_packet {
  uint8_t syncByte;
  uint8_t cmd_flag;
  uint8_t size;
  uint8_t data;
} __attribute__((packed)) ;

struct lidar_ans_header {
  uint8_t  syncByte1;
  uint8_t  syncByte2;
  uint32_t size: 30;
  uint32_t subType: 2;
  uint8_t  type;
} __attribute__((packed));

struct scanPoint {
  uint8_t quality;
  float 	angle;
  float 	distance;
  bool    startBit;
};


//TriLidar class
class TriLidar 
{
public:
  enum {
    SERIAL_BAUDRATE = 115200,
    DEFAULT_TIMEOUT = 500,
  };
  //construct
  TriLidar();
  //destructor
  ~TriLidar();

  // open the given serial interface and try to connect to the YDLIDAR
  bool begin(HardwareSerial &serialobj, uint32_t baudrate = SERIAL_BAUDRATE);

  // close the currently opened serial interface
  void end(void);

  // check whether the serial interface is opened
  bool isOpen(void);

  //设置强度位数信息（0表示不带信号强度，8表示8位信号强度，10表示10位信号强度）
  void setIntensity(int i);
  //设置单/双通
  void setSingleChannel(bool yes);
  bool isSingleChannel() const {return singleChannel;}
  //设置电机速度
  void setMotorSpeed(float vol);

  // ask the YDLIDAR for its health info
  result_t getHealth(device_health &health, uint32_t timeout = DEFAULT_TIMEOUT);

  // ask the YDLIDAR for its device info like the serial number
  result_t getDeviceInfo(device_info &info, uint32_t timeout = DEFAULT_TIMEOUT);

  // stop the scanPoint operation
  result_t stop(void);

  // start the scanPoint operation
  result_t startScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT * 2);

  // wait for one sample package to arrive
  result_t waitScanDot(uint32_t timeout = DEFAULT_TIMEOUT);

  // retrieve currently received sample point
  const scanPoint &getCurrentScanPoint(void) {
    return point;
  }

protected:
  // send ask commond to YDLIDAR
  result_t sendCommand(uint8_t cmd, const void *payload = NULL, size_t payloadsize = 0);
  //wait for response header to arrive
  result_t waitResponseHeader(
    lidar_ans_header *header, 
    uint8_t cmd = 0,
    uint32_t timeout = DEFAULT_TIMEOUT);

protected:
  HardwareSerial *_bined_serialdev = NULL;
  scanPoint point;
  int intensity = 0; //信号强度位数（0表示不带信号强度，8表示8位信号强度，10表示10位信号强度）
  uint8_t* packageBuffer = NULL; //接收缓存
  bool singleChannel = false; //单通标识
};
