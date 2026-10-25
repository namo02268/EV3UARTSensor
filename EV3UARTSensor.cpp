#include "EV3UARTSensor.h"

EV3UARTSensor::EV3UARTSensor(HardwareSerial *uart, byte id, uint32_t speed)
    : m_uart(uart),
      m_id(id),
      m_speed(speed),
      m_state(STATE_INIT),
      m_modeCount(0),
      m_viewCount(0)
{
}

void EV3UARTSensor::Begin()
{
  OnSetup();
  Reset();
}

void EV3UARTSensor::Run()
{
  EV3Packet packet;

  if (ReceivePacket(packet))
  {
    switch (m_state)
    {
    case STATE_WAIT_ACK:
      HandleWaitAck(packet);
      break;

    case STATE_RUNNING:
      HandleRunning(packet);
      break;

    default:
      break;
    }
  }

  CheckTimeout();

  if (m_modeCount > 0)
    m_modes[m_currentMode]->Update();

  switch (m_state)
  {
  case STATE_INIT:
    OnInit();
    SendInitPacket();
    break;

  case STATE_WAIT_ACK:
    OnWaitACK();
    break;

  case STATE_RUNNING:
    OnRunning();
    break;

  default:
    break;
  }
}

bool EV3UARTSensor::ReceivePacket(EV3Packet &packet)
{
  if (m_uart->available() == 0)
    return false;

  packet = {};
  packet.command = m_uart->peek();

  if (GetMessageType(packet.command) == MESSAGE_SYS)
  {
    packet.payloadlength = 0;
    packet.command = m_uart->read();
    packet.valid = true;
    return true;
  }

  packet.payloadlength = GetLength(GetLengthCode(packet.command));

  if (m_uart->available() < packet.payloadlength + 2)
    return false;

  packet.command = m_uart->read();

  for (byte index = 0; index < packet.payloadlength; index++)
    packet.payload[index] = m_uart->read();

  const byte checksum = m_uart->read();
  byte calculated = 0xFF ^ packet.command;

  for (byte index = 0; index < packet.payloadlength; index++)
    calculated ^= packet.payload[index];

  packet.valid = calculated == checksum;
  return true;
}

void EV3UARTSensor::HandleWaitAck(EV3Packet &packet)
{
  if (!packet.valid)
    return;

  if (packet.command != (MESSAGE_SYS | BYTE_ACK))
    return;

  m_uart->end();
  m_uart->begin(m_speed);
  m_modes[m_currentMode]->Init();
  OnModeChanged(m_currentMode);
  SetState(STATE_RUNNING);
}

void EV3UARTSensor::HandleRunning(EV3Packet &packet)
{
  if (!packet.valid)
    return;

  const byte type = GetMessageType(packet.command);

  if (type == MESSAGE_SYS)
  {
    if (packet.command == (MESSAGE_SYS | BYTE_NACK))
    {
      m_lastNack = millis();
      SendSensorData();
    }
    return;
  }

  if (type != MESSAGE_CMD)
    return;

  if (packet.command != (MESSAGE_CMD | LENGTH_1 | CMD_SELECT))
    return;

  const byte mode = packet.payload[0];

  if (mode >= m_modeCount)
    return;

  if (mode == m_currentMode)
    return;

  m_currentMode = mode;
  OnModeChanged(mode);
  m_modes[m_currentMode]->Init();
}

void EV3UARTSensor::CheckTimeout()
{
  switch (m_state)
  {
  case STATE_WAIT_ACK:
    if (millis() - m_stateTime > ACK_TIMEOUT)
    {
      // Send SYNC
      byte cmd = MESSAGE_SYS | BYTE_SYNC;
      Write(cmd);
      Write(cmd);
      Write(cmd);
      Write(cmd);
      Reset();
    }
    break;

  case STATE_RUNNING:
    if (millis() - m_lastNack > NACK_TIMEOUT)
      Reset();
    break;

  default:
    break;
  }
}

void EV3UARTSensor::SetState(EV3UARTState state)
{
  m_state = state;
  const unsigned long now = millis();

  switch (state)
  {
  case STATE_RUNNING:
    m_lastNack = now;
    break;

  case STATE_WAIT_ACK:
    m_stateTime = now;
    break;

  default:
    break;
  }
}

void EV3UARTSensor::Reset()
{
  m_uart->end();
  m_uart->begin(2400);
  SetState(STATE_INIT);
  m_currentMode = 0;
}

void EV3UARTSensor::SendInitPacket()
{
  byte cmd = 0;
  byte data[16];

  // センサーID
  cmd = MESSAGE_CMD | LENGTH_1 | CMD_TYPE;
  data[0] = m_id; // id
  Write(cmd, data, GetLength(LENGTH_1));

  // モードとViewの数
  cmd = MESSAGE_CMD | LENGTH_2 | CMD_MODES;
  data[0] = m_modeCount - 1;
  data[1] = m_viewCount - 1;
  Write(cmd, data, GetLength(LENGTH_2));

  // 通信速度
  cmd = MESSAGE_CMD | LENGTH_4 | CMD_SPEED;
  memcpy(data, &m_speed, sizeof(m_speed));
  Write(cmd, data, GetLength(LENGTH_4));

  // モード情報
  for (int mode = m_modeCount - 1; mode >= 0; mode--)
  {
    // INFO_NAME
    cmd = MESSAGE_INFO | LENGTH_8 | mode;
    memcpy(data, m_modes[mode]->modeInfo.name, 8);
    Write(cmd, data, GetLength(LENGTH_8), INFO_NAME);
    // INFO_RAW
    cmd = MESSAGE_INFO | LENGTH_8 | mode;
    memcpy(&data[0], &m_modes[mode]->modeInfo.Min, sizeof(float));
    memcpy(&data[4], &m_modes[mode]->modeInfo.Max, sizeof(float));
    Write(cmd, data, GetLength(LENGTH_8), INFO_RAW);
    // INFO_PCT
    cmd = MESSAGE_INFO | LENGTH_8 | mode;
    memcpy(&data[0], &m_modes[mode]->modeInfo.Min, sizeof(float));
    memcpy(&data[4], &m_modes[mode]->modeInfo.Max, sizeof(float));
    Write(cmd, data, GetLength(LENGTH_8), INFO_PCT);
    // INFO_SI
    cmd = MESSAGE_INFO | LENGTH_8 | mode;
    memcpy(&data[0], &m_modes[mode]->modeInfo.Min, sizeof(float));
    memcpy(&data[4], &m_modes[mode]->modeInfo.Max, sizeof(float));
    Write(cmd, data, GetLength(LENGTH_8), INFO_SI);
    // INFO_FORMAT
    cmd = MESSAGE_INFO | LENGTH_4 | mode;
    data[0] = m_modes[mode]->modeInfo.dataSets;
    data[1] = m_modes[mode]->modeInfo.dataFormat;
    data[2] = m_modes[mode]->modeInfo.dataFigures;
    data[3] = m_modes[mode]->modeInfo.dataDecimals;
    Write(cmd, data, GetLength(LENGTH_4), INFO_FORMAT);
  }

  // Send ACK
  cmd = MESSAGE_SYS | BYTE_ACK;
  Write(cmd);

  SetState(STATE_WAIT_ACK);
}

void EV3UARTSensor::SendSensorData()
{
  SensorMode *mode = m_modes[m_currentMode];

  byte data[32] = {0};
  mode->GetData(data);

  byte lengthCode = mode->GetLengthCode();

  byte cmd = MESSAGE_DATA | lengthCode | m_currentMode;

  Write(cmd, data, mode->GetDataLength());
}

void EV3UARTSensor::Write(byte cmd, const byte *data, byte len, int modeInfo)
{
  if (len == 0)
  {
    m_uart->write(cmd);
    return;
  }

  byte buf[35];

  byte checksum = 0xFF ^ cmd;
  byte index = 0;

  buf[index++] = cmd;

  if (modeInfo >= 0)
  {
    buf[index++] = byte(modeInfo);
    checksum ^= (byte)modeInfo;
  }

  for (byte i = 0; i < len; i++)
  {
    buf[index++] = data[i];
    checksum ^= data[i];
  }

  buf[index++] = checksum;

#ifdef DEBUG
  for (byte i = 0; i < index; i++)
  {
    m_uart->print(buf[i], HEX);
    m_uart->print(' ');
  }
  m_uart->println();
#else
  m_uart->write(buf, index);
#endif
}

bool EV3UARTSensor::AddMode(SensorMode *mode)
{
  if (m_modeCount >= MAX_MODES)
    return false;

  m_modes[m_modeCount] = mode;

  m_viewCount++;
  m_modeCount++;

  return true;
}
