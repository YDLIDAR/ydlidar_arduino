/*
注：当前示例使用Arduino Nano ESP32进行开发，仅作为参考，其它型号请自行适配
*/
#include "GsLidar.h"

//全局实例
HardwareSerial hs(0);
// HardwareSerial hs1(1);
GsLidar lidar;
bool on = true;

void setup() {
  //初始化LED_BUILTIN数字引脚为输出模式（使用灯作为程序运行的指示器）
  pinMode(LED_BUILTIN, OUTPUT);
  //设置串口波特率并打开
  hs.begin(921600);
  // hs1.begin(230400, SERIAL_8N1, 5, 6);
  //打印版本信息
  printf("GS SDK Version %s\n", lidar.getVersion());
  //设置串口对象
  lidar.begin(hs);
  //延时等待雷达启动
  // lidar.setEnableDebug(true); //启用调试
  //启动雷达
  if (!lidar.start()) {
    printf("Failed to start!\n");
  }
}

void loop() {
  Points ps;
  if (lidar.getPoints(ps)) {
    //  printf("Get %d points\n", ps.count);
    for (int i = 0; i < ps.count; ++i) {
      const Point& p = ps.points[i];
      //注：因可能无法打印浮点型数据，角度值可放大10倍后打印
      printf("p%d a:%.02f° d:%.0f\n", i, p.angle, p.dist);
    }
  } else {
    printf("Failed to get points!\n");
  }
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);  //点亮LED
  on = !on;
}
