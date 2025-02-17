#pragma once
#include "Arduino.h"
#include "inc/v8stdint.h"

//命令
#define GS_CMD_START 0x63
#define GS_CMD_STOP 0x64
#define GS_CMD_GETDEVINFO 0x90
#define GS_CMD_GETADDR 0x60
#define GS_CMD_GETPARAM 0x61
#define GS_CMD_GETVERSION 0x62 //获取设备信息
#define GS_CMD_GETVERSION2 0x6A //获取设备信息（带补丁号）
#define GS_CMD_GETVERSION3 0x6B //获取设备信息（带雷达型号码）
#define PH 0xA5
#define SDK_SNLEN 16 //序列号长度

//模组地址
#define LIDAR_MODULE_1 0x01
#define LIDAR_MODULE_2 0x02
#define LIDAR_MODULE_3 0x04
#define LIDAR_MODULE_ALL 0x00
#define LIDAR_MAXCOUNT 3 //最大模组数
#define LIDAR_MAXPOINTS 160 //单帧最大点数

#define SDK_TIMEOUT2000 2000 //默认超时时间
#define SDK_TIMEOUT1000 1000
#define SDK_TIMEOUT500 500
#define SDK_TIMEOUT300 300
#define SDK_TIMEOUT100 100
#ifndef SDK_TIMEOUT
#define SDK_TIMEOUT SDK_TIMEOUT1000 //默认超时时间
#endif

#define GS2_ANGLE 22.5 //GS2角度参数
#define GS5_ANGLE 16.0 //GS5角度参数

#define YDLIDAR_MOTOR_SCTP 3 // The PWM pin for control the speed of YDLIDAR's motor.
#define YDLIDAR_MOTOR_EN 7 // The ENABLE PIN for YDLIDAR's motor


//GS系列设备信息（对外协议）
struct gs_device_info {
    uint8_t hardware_version; //硬件版本号
    uint16_t firmware_version; //固件版本号
    uint8_t serialnum[SDK_SNLEN]; //序列号
} __attribute__((packed));
//GS系列设备信息（对内协议）
struct gs_device_info2 {
    uint8_t hardware_version; //硬件版本号
    uint16_t firmware_version; //固件版本号（大版本、小版本）
    uint8_t firmware2; //固件版本（补丁号）
    uint8_t serialnum[SDK_SNLEN]; //序列号
} __attribute__((packed));
struct gs_device_info3
{
    uint8_t hwVersion = 0; //硬件版本号
    uint16_t fwVersion = 0; //固件版本号
    uint8_t model = 0; //雷达型号
    uint8_t sn[SDK_SNLEN] = {0}; //序列号
} __attribute__((packed));
#define GSDEVINFO3SIZE sizeof(gs_device_info3)
//GS设备参数
struct gs_device_param
{
    uint16_t k0;
    uint16_t b0;
    uint16_t k1;
    uint16_t b1;
    int8_t bias;
} __attribute__((packed));
#define GSDEVPARAMSIZE sizeof(gs_device_param)

//GS包头
struct gs_pack_head {
    uint8_t flag0;
    uint8_t flag1;
    uint8_t flag2;
    uint8_t flag3;
    uint8_t addr;
    uint8_t type;
    uint16_t size;
} __attribute__((packed));
#define GSPACKHEADSIZE sizeof(gs_pack_head)
//GS点
struct gs_node
{
  uint16_t node = 0;
} __attribute__((packed));
#define GSNODESIZE sizeof(gs_node) //定义GS点大小
//GS单帧点
struct gs_nodes
{
  uint16_t evn;
  gs_node nodes[LIDAR_MAXPOINTS];
} __attribute__((packed));

//点定义
struct Point {
  float angle; //角度
  float dist; //距离
  uint8_t qual; //强度
};
//点云定义
struct Points {
  int count;
  Point points[LIDAR_MAXPOINTS];
};


//GsLidar class
class GsLidar 
{
public:
  enum Model
  {
    M_GS2 = 51, //GS2雷达
    M_GS1 = 52, //GS1-M雷达
    M_GS5 = 53, //GS5雷达
    M_GS6 = 54, //GS6雷达
    M_GS5_2 = 55, //GS5-2雷达
  };
public:
  GsLidar();
  ~GsLidar();
  // open the given serial interface and try to connect to the YDLIDAR
  bool begin(HardwareSerial &hs);
  // close the currently opened serial interface
  void end(void);
  // check whether the serial interface is opened
  bool isOpen(void);
  // start the scanPoint operation
  bool start(uint32_t timeout = SDK_TIMEOUT * 2);
  // stop the scanPoint operation
  bool stop(void);
  //配置是否解析信号强度
  void setIntensity(bool yes);
  // wait for one sample package to arrive
  bool getPoints(Points& points, uint32_t timeout = SDK_TIMEOUT);
  //enable debug
  void setEnableDebug(bool yes);
  //get version
  String getVersion() const;

protected:
  //发送命令
  bool sendCmd(
    uint8_t cmd,
    const void *data = NULL,
    uint32_t size = 0);
  //等待响应头
  bool waitRespHead(
    gs_pack_head *head, 
    uint8_t cmd = 0,
    uint32_t timeout = SDK_TIMEOUT);
  //地址转序号
  uint8_t addr2Index(uint8_t addr);
  //根据当前点序号和距离计算对应的角度和距离
  void angTrans(
    uint16_t dist,
    int n,
    double *dstTheta,
    uint16_t *dstDist); //GS2、GS5
  void angTrans2(
    uint16_t dist,
    int n,
    double *dstTheta,
    uint16_t *dstDist); //GS1、GS6
  //配置雷达地址
  bool resetDeviceAddr(uint32_t timeout = SDK_TIMEOUT);
  //获取设备信息
  bool getDeviceInfo(uint32_t timeout = SDK_TIMEOUT);
  bool getDeviceInfo2(uint32_t timeout = SDK_TIMEOUT);
  //获取设备参数
  bool getDeviceParam(uint32_t timeout = SDK_TIMEOUT);

protected:
  HardwareSerial *m_hs = NULL; //硬串口
  uint8_t moduleCount = 1; //模组数
  uint8_t moduleAddr = 0; //模块编号
  int m_models[LIDAR_MAXCOUNT] = {0}; //雷达型号
  double m_k0[LIDAR_MAXCOUNT] = {.0};
  double m_k1[LIDAR_MAXCOUNT] = {.0};
  double m_b0[LIDAR_MAXCOUNT] = {.0};
  double m_b1[LIDAR_MAXCOUNT] = {.0};
  double bias[LIDAR_MAXCOUNT] = {.0};
  bool m_intensity = false;
  int nodeCount = 0; //当前包点数

  double angle_p_x = 1.22;
  double angle_p_y = 5.315;
  double angle_p_angle = GS2_ANGLE; //GS2是22.5，GS5是16.0
  bool m_debug = false;
};
