/*
注：当前示例使用Arduino Nano ESP32进行开发，仅作为参考，其它型号请自行适配
*/
#include "GsLidar.h"

//全局实例
HardwareSerial hs(0);
GsLidar lidar;
bool on = true;

void setup() {
  //初始化LED_BUILTIN数字引脚为输出模式（使用灯作为程序运行的指示器）
  pinMode(LED_BUILTIN, OUTPUT);

  // hs.begin(230400, SERIAL_8N1, 5, 6);
  hs.begin(230400);
  //设置串口波特率
  lidar.begin(hs);
  // lidar.setEnableDebug(true); //启用调试
  //启动雷达
  if (!lidar.start()) {
    printf("Fail to start!\n");
  }
}

void loop() {
 Points ps;
 if (lidar.getPoints(ps)) {
   printf("Get %d points\n", ps.count);
  //  for (int i = 0; i < ps.count; ++i) {
  //    const Point& p = ps.points[i];
  //    printf("P%d a:%d d:%d\n", i, int(p.angle * 10), int(p.dist)); //无法打印浮点型数据
  //  }
 } else {
   printf("Fail to get points!\n");
 }
 digitalWrite(LED_BUILTIN, on ? HIGH : LOW); //点亮LED
 on = !on;
}
