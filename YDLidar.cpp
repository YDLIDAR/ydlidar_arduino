/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR Arduino
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.eaibot.com
 *
 */
#include "YDLidar.h"

YDLidar::YDLidar()
  : _bined_serialdev(NULL) {
  point.distance = 0;
  point.angle = 0;
  point.quality = 0;
  setIntensity(false);
}

YDLidar::~YDLidar() {
  end();
  delete[] packageBuffer;
}

// open the given serial interface and try to connect to the YDLIDAR
bool YDLidar::begin(HardwareSerial &serialobj, uint32_t baudrate) {
  if (isOpen()) {
    end();
  }

  _bined_serialdev = &serialobj;
  _bined_serialdev->end();
  _bined_serialdev->begin(baudrate);
  return true;
}

// close the currently opened serial interface
void YDLidar::end(void) {
  if (isOpen()) {
    _bined_serialdev->end();
    _bined_serialdev = NULL;
  }
}

// check whether the serial interface is opened
bool YDLidar::isOpen(void) {
  return _bined_serialdev ? true : false;
}

void YDLidar::setIntensity(int i) {
  intensity = i;
  //设置强度位数时重新分配内存
  if (packageBuffer)
  {
    delete[] packageBuffer;
    packageBuffer = NULL;
  }
  packageBuffer = new uint8_t[i ? sizeof(node_package_i) : sizeof(node_package)];
}

void YDLidar::setSingleChannel(bool yes) {
  singleChannel = yes;
  if (yes)
  {
    //单通雷达需要通过dtr控制启动
    pinMode(YDLIDAR_MOTOR_SCTP, OUTPUT);
    pinMode(YDLIDAR_MOTOR_EN, OUTPUT);
  }
}

//设置电机速度
void YDLidar::setMotorSpeed(float vol) {
  uint8_t PWM = (uint8_t)(51 * vol);
  analogWrite(YDLIDAR_MOTOR_SCTP, PWM);
}

// ask the YDLIDAR for its device health
result_t YDLidar::getHealth(device_health &health, uint32_t timeout) {
  result_t  ans;
  uint8_t  recvPos = 0;
  uint32_t currentTs = millis();
  uint32_t remainingtime;
  uint8_t *infobuf = (uint8_t *)&health;
  lidar_ans_header response_header;

  if (!isOpen()) {
    return RESULT_FAIL;
  }

  {
    ans = sendCommand(LIDAR_CMD_GET_DEVICE_HEALTH, NULL, 0);

    if (ans != RESULT_OK) {
      return ans;
    }

    if ((ans = waitResponseHeader(&response_header, LIDAR_ANS_TYPE_DEVHEALTH, timeout)) != RESULT_OK) {
      return ans;
    }
    if (response_header.size < sizeof(device_health)) {
      return RESULT_FAIL;
    }

    while ((remainingtime = millis() - currentTs) <= timeout) {
      int currentbyte = _bined_serialdev->read();

      if (currentbyte < 0) {
        continue;
      }

      infobuf[recvPos++] = currentbyte;

      if (recvPos == sizeof(device_health)) {
        return RESULT_OK;
      }
    }
  }

  return RESULT_TIMEOUT;
}

// ask the YDLIDAR for its device info
result_t YDLidar::getDeviceInfo(device_info &info, uint32_t timeout) {
  result_t  ans;
  uint8_t  recvPos = 0;
  uint32_t currentTs = millis();
  uint32_t remainingtime;
  uint8_t *infobuf = (uint8_t *)&info;
  lidar_ans_header response_header;

  if (!isOpen()) {
    return RESULT_FAIL;
  }

  {
    ans = sendCommand(LIDAR_CMD_GET_DEVICE_INFO, NULL, 0);

    if (ans != RESULT_OK) {
      return ans;
    }

    if ((ans = waitResponseHeader(&response_header, LIDAR_ANS_TYPE_DEVINFO, timeout)) != RESULT_OK) {
      return ans;
    }
    if (response_header.size < sizeof(lidar_ans_header)) {
      return RESULT_FAIL;
    }

    while ((remainingtime = millis() - currentTs) <= timeout) {
      int currentbyte = _bined_serialdev->read();

      if (currentbyte < 0) {
        continue;
      }

      infobuf[recvPos++] = currentbyte;

      if (recvPos == sizeof(device_info)) {
        return RESULT_OK;
      }
    }
  }

  return RESULT_TIMEOUT;
}

// stop the scanPoint operation
result_t YDLidar::stop(void) {
  if (!isOpen()) {
    return RESULT_FAIL;
  }
  result_t ans = RESULT_OK;
  if (singleChannel)
  {
      //stop motor
    digitalWrite(YDLIDAR_MOTOR_EN, LOW);
    setMotorSpeed(0);
  }
  else
  {
    ans = sendCommand(LIDAR_CMD_FORCE_STOP, NULL, 0);
  }
  return ans;
}

// start the scanPoint operation
result_t YDLidar::startScan(bool force, uint32_t timeout) 
{
  result_t ans;

  if (!isOpen()) {
    return RESULT_FAIL;
  }

  stop(); //force the previous operation to stop

  //双通雷达通过命令控制启动扫描
  if (!singleChannel)
  {
    if ((ans = sendCommand(force ? LIDAR_CMD_FORCE_SCAN : LIDAR_CMD_SCAN, NULL, 0)) != RESULT_OK) {
      return ans;
    }
    lidar_ans_header response_header;
    if ((ans = waitResponseHeader(&response_header, LIDAR_ANS_TYPE_MEASUREMENT, timeout)) != RESULT_OK) {
      return ans;
    }
    if (response_header.size < sizeof(node_info)) {
      return RESULT_FAIL;
    }
  }
  else //单通雷达通过串口dtr控制启动
  {
    //start motor in 1.8v
    setMotorSpeed(1.8);
    digitalWrite(YDLIDAR_MOTOR_EN, HIGH);
  }
  
  return RESULT_OK;
}

// wait scan data
result_t YDLidar::waitScanDot(uint32_t timeout) 
{
  if (!isOpen())
    return RESULT_FAIL;

  int recvPos = 0;
  uint32_t startTs = millis();
  uint32_t waitTime = 0;
  node_info node;
  node_package* package = (node_package*)packageBuffer;
  node_package_i* package_i = (node_package_i*)packageBuffer;
  static uint16_t package_Sample_Index = 0;
  static float IntervalSampleAngle = 0;
  static float IntervalSampleAngle_LastPackage = 0;
  static uint16_t FirstSampleAngle = 0;
  static uint16_t LastSampleAngle = 0;
  static uint16_t CheckSum = 0;

  static uint16_t CheckSumCal = 0;
  static uint16_t SampleNumlAndCTCal = 0;
  static uint16_t LastSampleAngleCal = 0;
  static bool CheckSumResult = true;
  static uint16_t Valu8Tou16 = 0;

  static uint8_t package_Sample_Num = 0;
  int32_t AngleCorrectForDistance = 0;
  static uint8_t package_CT = 0;
  int package_recvPos = 0;

  if (package_Sample_Index == 0)
  {
    recvPos = 0;

    while ((waitTime = millis() - startTs) <= timeout) 
    {
      int currentByte = _bined_serialdev->read();

      if (currentByte < 0)
        continue;

//      printf("%02X ", uint8_t(currentByte));

      switch (recvPos)
      {
      case 0:
        if (currentByte != PH1) {
          continue;
        }
        break;

      case 1:
        CheckSumCal = PH;
        if (currentByte != PH2) {
          recvPos = 0;
          continue;
        }
        break;

      case 2:
        SampleNumlAndCTCal = currentByte;
        if (((currentByte&0x01) != CT_Normal) && ((currentByte & 0x01) != CT_RingStart)) {
          recvPos = 0;
          continue;
        }
        package_CT = currentByte;
        break;

      case 3:
        SampleNumlAndCTCal += (currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
        package_Sample_Num = currentByte;
        break;

      case 4:
        if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
          FirstSampleAngle = currentByte;
        } else {
          recvPos = 0;
          continue;
        }
        break;

      case 5:
        FirstSampleAngle += (currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
        CheckSumCal ^= FirstSampleAngle;
        FirstSampleAngle = FirstSampleAngle >> 1;
        break;

      case 6:
        if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
          LastSampleAngle = currentByte;
        } else {
          recvPos = 0;
          continue;
        }
        break;

      case 7:
        LastSampleAngle += (currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
        LastSampleAngleCal = LastSampleAngle;
        LastSampleAngle = LastSampleAngle >> 1;

        if (package_Sample_Num == 1) {
          IntervalSampleAngle = 0;
        } else {
          if (LastSampleAngle < FirstSampleAngle) {
            if ((FirstSampleAngle > 17280) && (LastSampleAngle < 5760)) {
              IntervalSampleAngle = ((float)(23040 + LastSampleAngle - FirstSampleAngle)) /
                                    (package_Sample_Num - 1);
              IntervalSampleAngle_LastPackage = IntervalSampleAngle;
            } else {
              IntervalSampleAngle = IntervalSampleAngle_LastPackage;
            }
          } else {
            IntervalSampleAngle = ((float)(LastSampleAngle - FirstSampleAngle)) / (package_Sample_Num - 1);
            IntervalSampleAngle_LastPackage = IntervalSampleAngle;
          }
        }
        break;

      case 8:
        CheckSum = currentByte;
        break;
      case 9:
        CheckSum += (currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
        break;
      }

      packageBuffer[recvPos++] = currentByte;

      if (recvPos == PackagePaidBytes) {
        package_recvPos = recvPos;
        break;
      }
    }

    if (PackagePaidBytes == recvPos) 
    {
      startTs = millis();
      recvPos = 0;
      int package_sample_sum = intensity ? package_Sample_Num * 3 : package_Sample_Num * 2;

      while ((waitTime = millis() - startTs) <= timeout) 
      {
        int currentByte = _bined_serialdev->read();
        if (currentByte < 0)
          continue;

//        printf("%02X ", uint8_t(currentByte));

        //计算点云部分的校验和
        if (intensity)
        {
          if (recvPos % 3 == 0)
            CheckSumCal ^= uint8_t(currentByte);
          else if (recvPos % 3 == 1)
            Valu8Tou16 = currentByte;
          else
          {
            Valu8Tou16 += (currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
            CheckSumCal ^= Valu8Tou16;
          }
        }
        else
        {
          if (recvPos % 2 == 0)
            Valu8Tou16 = currentByte;
          else {
            Valu8Tou16 += (currentByte << LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
            CheckSumCal ^= Valu8Tou16;
          }
        }

        packageBuffer[package_recvPos + recvPos] = currentByte;
        recvPos ++;

        if (package_sample_sum == recvPos) {
          package_recvPos += recvPos;
          break;
        }
      }

      if (package_sample_sum != recvPos) {
        return RESULT_FAIL;
      }
    } else {
      return RESULT_FAIL;
    }

    CheckSumCal ^= SampleNumlAndCTCal;
    CheckSumCal ^= LastSampleAngleCal;

    if (CheckSumCal != CheckSum) {
//      printf("CheckSum error cal: %04X cur: %04X\n", CheckSumCal, CheckSum);
      CheckSumResult = false;
    } else {
      CheckSumResult = true;
    }
  }

  if ((package_CT&0x01) == CT_Normal) {
    node.sync_flag = Node_NotSync;
  } else {
    node.sync_flag = Node_Sync;
  }

  if (CheckSumResult)
  {
    //如果带信号强度信息
    if (intensity)
    {
      node.quality = package_i->points[package_Sample_Index].quality;
      node.distance = package_i->points[package_Sample_Index].dist;
    }
    else
    {
      node.quality = 0;
      node.distance = package->points[package_Sample_Index].dist;
    }

    if (node.distance != 0) {
      AngleCorrectForDistance = (int32_t)((atan(((21.8 * (155.3 - (node.distance))) /
         155.3) / (node.distance))) * 3666.93);
    } else {
      AngleCorrectForDistance = 0;
    }

    float sampleAngle = IntervalSampleAngle * package_Sample_Index;
    if ((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) < 0) {
      node.angle = (((uint16_t)(FirstSampleAngle + sampleAngle + AngleCorrectForDistance +
          23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
    } else {
      if ((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) > 23040) {
        node.angle = ((uint16_t)((FirstSampleAngle + sampleAngle + AngleCorrectForDistance -
            23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
      } else {
        node.angle = ((uint16_t)((FirstSampleAngle + sampleAngle + AngleCorrectForDistance)) <<
            LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
      }
    }
  } else {
    node.sync_flag = Node_NotSync;
    node.quality = Node_Default_Quality;
    node.angle = LIDAR_RESP_MEASUREMENT_CHECKBIT;
    node.distance = 0;
    package_Sample_Index = 0;
    return RESULT_FAIL;
  }

  point.distance = node.distance;
  point.angle = (node.angle >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
  point.quality = (node.quality);
  point.startBit = (node.sync_flag);

  package_Sample_Index ++;
  if (package_Sample_Index >= package_Sample_Num) {
    package_Sample_Index = 0;
  }

  return RESULT_OK;
}

//send data to serial
result_t YDLidar::sendCommand(uint8_t cmd, const void *payload, size_t payloadsize) {
  cmd_packet pkt_header;
  cmd_packet *header = &pkt_header;
  uint8_t checksum = 0;

  if (payloadsize && payload) {
    cmd |= LIDAR_CMDFLAG_HAS_PAYLOAD;
  }

  header->syncByte = LIDAR_CMD_SYNC_BYTE;
  header->cmd_flag = cmd & 0xff;

  _bined_serialdev->write((uint8_t *)header, 2) ;

  if ((cmd & LIDAR_CMDFLAG_HAS_PAYLOAD)) {
    checksum ^= LIDAR_CMD_SYNC_BYTE;
    checksum ^= (cmd & 0xff);
    checksum ^= (payloadsize & 0xFF);

    for (size_t pos = 0; pos < payloadsize; ++pos) {
      checksum ^= ((uint8_t *)payload)[pos];
    }

    uint8_t sizebyte = payloadsize;
    _bined_serialdev->write(&sizebyte, 1);
    _bined_serialdev->write((const uint8_t *)payload, sizebyte);
    _bined_serialdev->write(&checksum, 1);
  }

  return RESULT_OK;
}

// wait response header
result_t YDLidar::waitResponseHeader(
  lidar_ans_header *header, 
  uint8_t cmd,
  uint32_t timeout) {
  int recvPos = 0;
  uint32_t startTs = millis();
  uint8_t *headerBuffer = (uint8_t *)(header);
  uint32_t waitTime;

  while ((waitTime = millis() - startTs) <= timeout) {
    int currentbyte = _bined_serialdev->read();

    if (currentbyte < 0) 
      continue;

    switch (recvPos) {
    case 0:
      if (currentbyte != LIDAR_ANS_SYNC_BYTE1) {
        continue;
      }
      break;
    case 1:
      if (currentbyte != LIDAR_ANS_SYNC_BYTE2) {
        recvPos = 0;
        continue;
      }
      break;
    case 6:
      if (cmd && currentbyte != cmd) {
          recvPos = 0;
          continue;
      }
      break;
    }

    headerBuffer[recvPos++] = currentbyte;

    if (recvPos == sizeof(lidar_ans_header)) {
      return RESULT_OK;
    }
  }

  return RESULT_TIMEOUT;
}
