#include "GsLidar.h"


GsLidar::GsLidar()
{
}

GsLidar::~GsLidar()
{
  end();
}

bool GsLidar::begin(HardwareSerial &hs)
{
  //如果已打开则先关闭
  if (isOpen())
    end();

  m_hs = &hs;
  // m_hs->end();
  // m_hs->begin(br);
  return true;
}

// close the currently opened serial interface
void GsLidar::end(void)
{
  if (isOpen()) {
    m_hs->end();
    m_hs = NULL;
  }
}

// check whether the serial interface is opened
bool GsLidar::isOpen(void)
{
  return m_hs ? true : false;
}

void GsLidar::setIntensity(bool yes)
{
  m_intensity = yes;
}

bool GsLidar::start(uint32_t timeout)
{
  if (!isOpen())
    return false;

  //重置设备地址
  if (!resetDeviceAddr())
  {
    printf("Fail to reset device address\n");
    return false;
  }
  //获取设备信息
  if (!getDeviceInfo())
  {
    printf("Fail to reset device info\n");
    return false;
  }
  //获取设备参数
  if (!getDeviceParam())
  {
    printf("Fail to reset device parameters\n");
    return false;
  }
  //启动扫描
  if (!sendCmd(GS_CMD_START))
    return false;
  gs_pack_head h;
  if (!waitRespHead(&h, GS_CMD_START, timeout))
    return false;
  return true;
}

bool GsLidar::stop(void)
{
  if (!isOpen())
    return false;
  if (!sendCmd(GS_CMD_STOP))
    return false;
  gs_pack_head h;
  if (!waitRespHead(&h, GS_CMD_STOP))
    return false;
  return true;
}

bool GsLidar::getPoints(Points& ps, uint32_t timeout)
{
  if (!isOpen())
    return RESULT_FAIL;

  int pos = 0;
  uint32_t startTs = millis();
  uint32_t waitTime = 0;
  uint16_t size = 0;
  uint8_t ccs = 0; //计算校验和
  uint8_t scs = 0; //原始校验和
  gs_nodes nodes;
  uint8_t* buff = (uint8_t*)&nodes;
  Point p;

  while ((waitTime = millis() - startTs) < timeout)
  {
    int c = m_hs->read();
    if (c < 0)
    {
      delay(1); //延时
      continue;
    }
    if (m_debug)
      printf("%02X", uint8_t(c));
    switch (pos)
    {
      case 0:
        if (c != PH) {
          continue;
        }
        break;
      case 1:
        if (c != PH) {
          pos = 0;
          continue;
        }
        break;
      case 2:
        if (c != PH) {
          pos = 0;
          continue;
        }
        break;
      case 3:
        if (c != PH) {
          pos = 0;
          continue;
        }
        break;
      case 4:
        if (c == PH) {
          continue;
        }
        moduleAddr = c;
        break;
      case 5:
        if (c != GS_CMD_START) {
          pos = 0;
          continue;
        }
        break;
      case 6:
        size = uint16_t(0xFF & c);
        break;
      case 7:
        size += uint16_t(0xFF & c) << 8;
        break;
    }

    if (pos >= 4)
      ccs += uint8_t(c);
    else
      ccs = 0;
    pos++;

    if (pos == GSPACKHEADSIZE)
    {
      if (m_debug)
        printf("\n");
      nodeCount = (size - 2) / GSNODESIZE; //计算1包数据中的点数
      //读取数据部分
      int s = m_hs->readBytes(buff, size);
      if (s < size)
      {
        printf("Read %d data timeout, readed %d\n", size, s);
        return false;
      }
      //计算校验和
      for (int i = 0; i < size; ++i)
      {
        ccs += buff[i];
        if (m_debug)
          printf("%02X", uint8_t(buff[i]));
      }
      if (m_debug)
      {
        printf("\n");
        fflush(stdout);
      }
      //读取校验和字节
      c = m_hs->read();
      if (c < 0)
      {
        printf("Fail to read check sum byte\n");
        return false;
      }
      scs = c;
      if (ccs != scs)
      {
        printf("Check sum error calc[0x%02X] != [0x%02X]\n", ccs, scs);
        return false;
      }
      //处理点云
      uint8_t id = addr2Index(moduleAddr);
      int model = m_models[id]; //当前雷达型号
      if (m_debug)
        printf("Module[%d] Model[%d]\n", id, model);
      //根据雷达型号设置角度参数
      if (M_GS5 == model ||
          M_GS5_2 == model)
        angle_p_angle = GS5_ANGLE;
      else
        angle_p_angle = GS2_ANGLE;

      //解析点云数据
      uint16_t dist = 0;
      double angle = 0;
      ps.count = nodeCount;
      for (int i = 0; i < nodeCount; ++i)
      {
        if (M_GS1 == model) //GS1低10位为距离，高6位为信号强度
        {
          dist = uint16_t(nodes.nodes[i].node & 0x03FF);
          //如果配置了信号强度则处理信号强度
          //if (m_intensities)
          p.qual = uint16_t(nodes.nodes[i].node >> 10);
        }
        else if (M_GS5 == model ||
                 M_GS5_2 == model ||
                 M_GS6 == model) //GS5、GS6低11位为距离，高5位为信号强度
        {
          dist = uint16_t(nodes.nodes[i].node & 0x07FF);
          //如果配置了信号强度则处理信号强度
          //if (m_intensities)
          p.qual = uint16_t(nodes.nodes[i].node >> 11);
        }
        else //GS2低9位为距离，高7位为信号强度
        {
          dist = uint16_t(nodes.nodes[i].node & 0x01FF);
          //如果配置了信号强度则处理信号强度
          //if (m_intensities)
          p.qual = uint16_t(nodes.nodes[i].node >> 9);
        }

        if (dist > 0)
        {
          //根据距离算角度，点的排序也不一样
          if (M_GS1 == model)
            angTrans2(dist, i, &angle, &dist);
          else if (M_GS6 == model)
            angTrans2(dist, nodeCount - i, &angle, &dist);
          else //GS2，GS5
            angTrans(dist, i, &angle, &dist);
        }

        p.angle = angle;
        p.dist = dist;
        ps.points[i] = p;
      }

      return true;
    }
  }

  return false;
}

void GsLidar::setEnableDebug(bool yes)
{
  m_debug = yes;
}

String GsLidar::getVersion() const
{
  //V1.0
  //初版
  return "V1.0";
}

bool GsLidar::sendCmd(
  uint8_t cmd,
  const void *data,
  uint32_t size)
{
  if (!isOpen())
    return false;

  int s = GSPACKHEADSIZE + size + 1;
  uint8_t *buff = new uint8_t[s];
  gs_pack_head *head = reinterpret_cast<gs_pack_head*>(buff);
  
  head->flag0 = PH;
  head->flag1 = PH;
  head->flag2 = PH;
  head->flag3 = PH;
  head->addr = LIDAR_MODULE_ALL;
  head->type = cmd;
  head->size = 0xFFFF & size;

  if (size && data)
    memcpy(&buff[GSPACKHEADSIZE], data, size);

  uint8_t cs = 0;
  for (int i = 4; i < s - 1; ++i)
    cs += buff[i];
  buff[s - 1] = cs;

  int ss = m_hs->write(buff, s);

  delete[] buff; //释放内存

  return ss > 0;
}

bool GsLidar::waitRespHead(
  gs_pack_head *head,
  uint8_t cmd,
  uint32_t timeout)
{
  int pos = 0;
  uint32_t startTs = millis();
  uint8_t *buff = (uint8_t*)(head);
  uint32_t waitTime;

  while ((waitTime = millis() - startTs) <= timeout)
  { 
    int c = m_hs->read();
    if (c < 0) {
      delay(100); //延时
      continue;
    }
    if (m_debug)
      printf("%02X", uint8_t(c));
    switch (pos)
    {
      case 0:
        if (c != PH) {
          continue;
        }
        break;
      case 1:
        if (c != PH) {
          pos = 0;
          continue;
        }
        break;
      case 2:
        if (c != PH) {
          pos = 0;
          continue;
        }
        break;
      case 3:
        if (c != PH) {
          pos = 0;
          continue;
        }
        break;
      case 4: //地址
        if (c == PH) {
          continue;
        }
        break;
      case 5: //命令字
        if (cmd && c != cmd) {
          pos = 0;
          continue;
        }
        break;
    }

    buff[pos++] = c;

    if (pos == GSPACKHEADSIZE)
    {
      printf("\n");
      return true;
    }
  }

  return false;
}

uint8_t GsLidar::addr2Index(uint8_t addr)
{
  uint8_t i = 1;
  while (!(addr & 0x01))
  {
    addr = addr >> 1;
    i ++;

    if (i >= 8)
      break;
  }
  return i - 1;
}

void GsLidar::angTrans(
  uint16_t dist,
  int n,
  double *dstTheta,
  uint16_t *dstDist)
{
  //根据距离和序号计算对应的角度值和距离值（无相关文档）
  double pixelU = n, Dist = 0, theta = 0, tempTheta = 0, tempDist = 0, tempX = 0, tempY = 0;
  uint8_t mdNum = 0x03 & (moduleAddr >> 1); //1,2,4
  if (n < nodeCount / 2)
  {
    pixelU = nodeCount / 2 - pixelU;
    if (m_b0[mdNum] > 1) {
      tempTheta = m_k0[mdNum] * pixelU - m_b0[mdNum];
    }
    else
    {
      tempTheta = atan(m_k0[mdNum] * pixelU - m_b0[mdNum]) * 180 / M_PI;
    }
    tempDist = (dist - angle_p_x) / cos(((angle_p_angle + bias[mdNum]) - (tempTheta)) * M_PI / 180);
    tempTheta = tempTheta * M_PI / 180;
    tempX = cos((angle_p_angle + bias[mdNum]) * M_PI / 180) * tempDist * cos(tempTheta) +
            sin((angle_p_angle + bias[mdNum]) * M_PI / 180) * (tempDist * sin(tempTheta));
    tempY = -sin((angle_p_angle + bias[mdNum]) * M_PI / 180) * tempDist * cos(tempTheta) +
            cos((angle_p_angle + bias[mdNum]) * M_PI / 180) * (tempDist * sin(tempTheta));
    tempX = tempX + angle_p_x;
    tempY = tempY - angle_p_y; //5.315
    Dist = sqrt(tempX * tempX + tempY * tempY);
    theta = atan(tempY / tempX) * 180 / M_PI;
  }
  else
  {
    pixelU = nodeCount - pixelU;
    if (m_b1[mdNum] > 1)
    {
      tempTheta = m_k1[mdNum] * pixelU - m_b1[mdNum];
    }
    else
    {
      tempTheta = atan(m_k1[mdNum] * pixelU - m_b1[mdNum]) * 180 / M_PI;
    }
    tempDist = (dist - angle_p_x) / cos(((angle_p_angle + bias[mdNum]) + (tempTheta)) * M_PI / 180);
    tempTheta = tempTheta * M_PI / 180;
    tempX = cos(-(angle_p_angle + bias[mdNum]) * M_PI / 180) * tempDist * cos(tempTheta) +
            sin(-(angle_p_angle + bias[mdNum]) * M_PI / 180) * (tempDist * sin(tempTheta));
    tempY = -sin(-(angle_p_angle + bias[mdNum]) * M_PI / 180) * tempDist * cos(tempTheta) +
            cos(-(angle_p_angle + bias[mdNum]) * M_PI / 180) * (tempDist * sin(tempTheta));

    tempX = tempX + angle_p_x;
    tempY = tempY + angle_p_y; //5.315
    Dist = sqrt(tempX * tempX + tempY * tempY);
    theta = atan(tempY / tempX) * 180 / M_PI;
  }
  if (theta < 0)
  {
    theta += 360;
  }
  *dstTheta = theta;
  *dstDist = Dist;
}

void GsLidar::angTrans2(
  uint16_t dist,
  int n,
  double *dstTheta,
  uint16_t *dstDist)
{
  double pixelU = nodeCount - n, Dist, theta, tempTheta;
  uint8_t mdNum = 0x03 & (moduleAddr >> 1); // 1,2,4

  tempTheta = atan(m_k0[mdNum] * pixelU - m_b0[mdNum]) * 180 / M_PI;
  Dist = dist / cos(tempTheta * M_PI / 180);
  theta = tempTheta;

  if (theta < 0)
  {
    theta += 360;
  }
  *dstTheta = theta;
  *dstDist = Dist;
}

bool GsLidar::resetDeviceAddr(uint32_t timeout)
{
  if (!sendCmd(GS_CMD_GETADDR))
    return false;
  gs_pack_head h;
  if (!waitRespHead(&h, GS_CMD_GETADDR, timeout))
    return false;
  moduleCount = (h.addr >> 1) + 1;
  printf("Module count [%u]\n", moduleCount);
  return true;
}

bool GsLidar::getDeviceInfo(uint32_t timeout)
{
  //先初始化为GS2
  for (int i = 0; i < moduleCount && i < LIDAR_MAXCOUNT; ++i)
    m_models[i] = M_GS2;
    
  getDeviceInfo2(timeout);

  for (int i = 0; i < moduleCount && i < LIDAR_MAXCOUNT; ++i)
    printf("Module[%d] Model[%d]\n", i, m_models[i]);
  return true;
}

bool GsLidar::getDeviceInfo2(uint32_t timeout)
{
  if (!sendCmd(GS_CMD_GETVERSION3))
    return false;
  for (int i = 0; i < moduleCount && i < LIDAR_MAXCOUNT; ++i)
  {
    gs_pack_head h;
    memset(&h, 0, GSPACKHEADSIZE);
    if (!waitRespHead(&h, GS_CMD_GETVERSION3, timeout))
      return false;
    if (h.size < GSDEVINFO3SIZE)
      return false;
    gs_device_info3 di;
    if (m_hs->readBytes(reinterpret_cast<uint8_t*>(&di), GSDEVINFO3SIZE) < GSDEVINFO3SIZE)
      return false;
    uint8_t id = addr2Index(h.addr); //雷达序号
    m_models[id] = di.model; //雷达型号
    printf("Get module [%d] model [%u]\n", id, di.model);
  }
  return true;
}

bool GsLidar::getDeviceParam(uint32_t timeout)
{
  //先初始化为0
  for (int i = 0; i < moduleCount && i < LIDAR_MAXCOUNT; ++i)
  {
    m_k0[i] = .0;
    m_k1[i] = .0;
    m_b0[i] = .0;
    m_b1[i] = .0;
    bias[i] = .0;
  }

  if (!sendCmd(GS_CMD_GETPARAM))
    return false;
  gs_pack_head h;
  for (int i = 0; i < moduleCount && i < LIDAR_MAXCOUNT; ++i)
  {
    if (!waitRespHead(&h, GS_CMD_GETPARAM, timeout))
      return false;
    if (h.size < GSDEVPARAMSIZE)
      return false;
    gs_device_param dp;
    if (m_hs->readBytes(reinterpret_cast<uint8_t*>(&dp), GSDEVPARAMSIZE) < GSDEVPARAMSIZE)
      return false;
    uint8_t id = addr2Index(h.addr); //雷达序号
    m_k0[id] = dp.k0 / 10000.00;
    m_k1[id] = dp.k1 / 10000.00;
    m_b0[id] = dp.b0 / 10000.00;
    m_b1[id] = dp.b1 / 10000.00;
    bias[id] = double(dp.bias) * 0.1;
  }
  return true;
}
