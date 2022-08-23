/*
    YDLIDAR SYSTEM
    YDLIDAR Arduino

    Copyright 2015 - 2022 EAI TEAM
    http://www.eaibot.com
*/
#include <YDLidar.h>

void initLidar();
void printf_begin();

// You need to create an driver instance
YDLidar lidar;


void setup() {
  //设置为带信号强度
  lidar.setIntensity(true);
  //设置为双通
  lidar.setSingleChannel(false);
  //设置串口波特率
  //f4,s4:115200
  //g4:230400
  //x4:128000
  // bind the YDLIDAR driver to the arduino hardware serial
  lidar.begin(Serial, 115200);

  printf_begin();

  //如果是双通雷达，尝试获取设备信息和健康信息
  if (!lidar.isSingleChannel())
  {
    initLidar();
  }
  //启动雷达
  if (lidar.startScan() != RESULT_OK)
  {
    printf("Fail to start!");
  }
}

void loop() {
    if (lidar.waitScanDot() == RESULT_OK) {
      const scanPoint &p = lidar.getCurrentScanPoint();
      printf("Point angle: %d dist: %d\n", int(p.angle * 10), int(p.distance));
    } else {
      printf("Fail to get scan point!\n");
      delay(1);
    }
}

//获取并打印设备信息和健康信息
void initLidar() {
  //获取设备信息
  device_info di = {0};
  if (lidar.getDeviceInfo(di, 100) == RESULT_OK)
  {
    int _samp_rate = 4;
    String model;
    float freq = 7.0f;
    switch (di.model) {
      case 1:
        model = "F4";
        _samp_rate = 4;
        freq = 7.0;
        break;
      case 4:
        model = "S4";
        _samp_rate = 4;
        freq = 7.0;
        break;
      case 5:
        model = "G4";
        _samp_rate = 9;
        freq = 7.0;
        break;
      case 6:
        model = "X4";
        _samp_rate = 5;
        freq = 7.0;
        break;
      default:
        model = "Unknown";
    }

    uint16_t maxv = (uint16_t)(di.firmware_version >> 8);
    uint16_t midv = (uint16_t)(di.firmware_version & 0xff) / 10;
    uint16_t minv = (uint16_t)(di.firmware_version & 0xff) % 10;
    if (midv == 0) {
      midv = minv;
      minv = 0;
    }

    printf("Firmware version: %u.%u.%u\n", maxv, midv, minv);
    printf("Hardware version: %u\n", di.hardware_version);
    printf("Model: %s\n", model);
    printf("Serial: ");
    for (int i = 0; i < 16; i++) {
      printf("%u", di.serialnum[i] & 0xff);
    }
    printf("\n");

    Serial.print("Sampling Rate: ");
    Serial.print(_samp_rate, DEC);
    Serial.println("K");

    Serial.print("Scan Frequency: ");
    Serial.print(freq, DEC);
    Serial.println("Hz");
  } else {
    Serial.println("Fail to get device info!");
  }

  delay(100);
  device_health hi = {0};
  if (lidar.getHealth(hi, 100) == RESULT_OK) {
    Serial.print("[YDLIDAR INFO] YDLIDAR running correctly! The health status:");
    Serial.println(hi.status == 0 ? "well" : "bad");
  } else {
    Serial.println("Fail to get health status!");
  }
}

//让printf函数能输出数据到串口（printf函数无法处理float和double类型数据）
int serial_putc(char c, struct __file*) {
  Serial.write(c);
  return c;
}
void printf_begin() {
  fdevopen(&serial_putc, 0);
}