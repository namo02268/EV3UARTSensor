#ifndef EV3UARTSENSOR_
#define EV3UARTSENSOR_

#include <Arduino.h>

// 6-7bit メッセージの種類
#define MESSAGE_SYS 0x00
#define MESSAGE_CMD 0x40
#define MESSAGE_INFO 0x80
#define MESSAGE_DATA 0xC0

// 3-5bit メッセージサイズ
#define LENGTH_1 0x00
#define LENGTH_2 0x08
#define LENGTH_4 0x10
#define LENGTH_8 0x18
#define LENGTH_16 0x20
#define LENGTH_32 0x28

// 0-2bit MESSAGE_SYS
#define BYTE_SYNC 0x00
#define BYTE_NACK 0x02
#define BYTE_ACK 0x04
// 0-2bit MESSAGE_CMD
#define CMD_TYPE 0x00
#define CMD_MODES 0x01
#define CMD_SPEED 0x02
#define CMD_SELECT 0x03
#define CMD_WRITE 0x04
#define CMD_EXT_MODE 0x06
#define CMD_VERSION 0x07
// 0-2bit MESSAGE_INFO, MESSAGE_DATA
#define MODE_0 0x00
#define MODE_1 0x01
#define MODE_2 0x02
#define MODE_3 0x03
#define MODE_4 0x04
#define MODE_5 0x05
#define MODE_6 0x06
#define MODE_7 0x07

// MESSAGE_INFO 2バイト目
#define INFO_NAME 0x00
#define INFO_RAW 0x01
#define INFO_PCT 0x02
#define INFO_SI 0x03
#define INFO_UNITS 0x04
#define INFO_FORMAT 0x80

// データフォーマット
#define DATA8 0x00  // int8_t
#define DATA16 0x01 // int16_t
#define DATA32 0x02 // int32_t
#define DATAF 0x03  // float

// タイムアウト
constexpr unsigned long ACK_TIMEOUT = 1000UL;
constexpr unsigned long NACK_TIMEOUT = 500UL;

// #define DEBUG

struct EV3Packet
{
  byte command;
  byte payload[32];
  byte payloadlength;
  bool valid;
};

struct ModeInfo
{
  ModeInfo(
      const char *name,
      float Min,
      float Max,
      byte dataSets,
      byte dataFormat,
      byte dataFigures,
      byte dataDecimals)
      : Min(Min),
        Max(Max),
        dataSets(dataSets),
        dataFormat(dataFormat),
        dataFigures(dataFigures),
        dataDecimals(dataDecimals)
  {
    strncpy(this->name, name, sizeof(this->name));
  }
  char name[8] = {0};    // モード名
  float Min = 0;         // 最小値
  float Max = 0;         // 最大値
  byte dataSets = 0;     // データの数
  byte dataFormat = 0;   // データフォーマット
  byte dataFigures = 0;  // 表示桁数
  byte dataDecimals = 0; // 小数点以下桁数
};

class SensorMode
{
public:
  SensorMode(const ModeInfo &modeInfo) : modeInfo(modeInfo) {}
  virtual ~SensorMode() {}

  virtual void Init() {}
  virtual void Update() {}
  virtual void GetData(byte *buffer) = 0;

  byte GetDataLength() const
  {
    byte size;

    switch (modeInfo.dataFormat)
    {
    case DATA8:
      size = 1;
      break;

    case DATA16:
      size = 2;
      break;

    case DATA32:
    case DATAF:
      size = 4;
      break;

    default:
      return 0;
    }

    return size * modeInfo.dataSets;
  }

  byte GetLengthCode() const
  {
    byte length = GetDataLength();

    if (length <= 1)
      return LENGTH_1;
    if (length <= 2)
      return LENGTH_2;
    if (length <= 4)
      return LENGTH_4;
    if (length <= 8)
      return LENGTH_8;
    if (length <= 16)
      return LENGTH_16;
    if (length <= 32)
      return LENGTH_32;

    return 0;
  }

  const ModeInfo modeInfo;
};

enum EV3UARTState
{
  STATE_INIT,
  STATE_WAIT_ACK,
  STATE_RUNNING
};

class EV3UARTSensor
{
public:
  EV3UARTSensor(HardwareSerial *uart, byte id, uint32_t speed);
  virtual ~EV3UARTSensor() = default;

  void Begin();
  void Run();

  void Reset();

  bool AddMode(SensorMode *mode);

protected:
  virtual void OnSetup() {}
  virtual void OnInit() {}
  virtual void OnWaitACK() {}
  virtual void OnRunning() {}
  virtual void OnModeChanged(byte mode) {}

private:
  bool ReceivePacket(EV3Packet &packet);
  void HandleWaitAck(EV3Packet &packet);
  void HandleRunning(EV3Packet &packet);

  void SendInitPacket();
  void SendSensorData();
  void CheckTimeout();
  void SetState(EV3UARTState state);

  void Write(byte cmd, const byte *data = nullptr, byte len = 0, int info = -1);

private:
  HardwareSerial *m_uart;

  EV3UARTState m_state;

  unsigned long m_stateTime = 0;
  unsigned long m_lastNack = 0;

  byte m_id;
  uint32_t m_speed;
  byte m_modeCount;
  byte m_viewCount;
  byte m_currentMode = 0;

  static const byte MAX_MODES = 8;
  SensorMode *m_modes[MAX_MODES];
};

static inline byte GetLength(byte lengthCode)
{
  return 1 << ((lengthCode & 0x38) >> 3);
}

static inline byte GetMessageType(byte cmd)
{
  return cmd & 0xC0;
}

static inline byte GetLengthCode(byte cmd)
{
  return cmd & 0x38;
}

static inline byte GetMessageId(byte cmd)
{
  return cmd & 0x07;
}

#endif // EV3UART_
