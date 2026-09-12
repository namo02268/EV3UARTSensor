// EV3 UARTセンサー用ライブラリの読み込み
#include <EV3UARTSensor.h>

//-----------------------------------------//
// モード1
// int16_t型のデータを1個送信する例
//-----------------------------------------//
class Mode1 : public SensorMode {
public:
  // モードの設定
  Mode1() : SensorMode({
    "Mode1", // モード名（ポートビュー表示用）
    -32768,  // 送信値の最小値（適当な値でもOK）
    32767,   // 送信値の最大値（適当な値でもOK）
    1,       // 送信データ数（設定可能値: 1, 2, 4, 8）
    DATA16,  // データ型（DATA8: int8_t, DATA16: int16_t, DATA32: int32_t, DATAF: float）
    5,       // 表示桁数（整数部＋小数部の合計桁数）
    0        // 小数点以下の桁数
  }) {}

  // 繰り返し実行される処理
  // センサーの値を更新するなどの処理を行います
  void Update() override {
    value++;
  }

  // EV3へ送信するデータを設定する処理
  void GetData(byte *buffer) override {
    // 変数のデータを送信用のバッファにコピーします
    // 基本的に変数名以外は変更する必要はありません
    memcpy(buffer, &value, sizeof(value));
  }

private:
  // モードで使用する変数
  int16_t value = 0;
};

//-----------------------------------------//
// モード2
// int8_t型のデータを5個送信する例
//-----------------------------------------//
class Mode2 : public SensorMode {
public:
  // モードの設定
  Mode2() : SensorMode({
    "Mode2", // モード名（ポートビュー表示用）
    -128,    // 送信値の最小値（適当な値でもOK）
    127,     // 送信値の最大値（適当な値でもOK）
    8,       // 送信データ数（設定可能値: 1, 2, 4, 8）、5個送信する場合は8に設定し残り3個はダミー
    DATA8,   // データ型（DATA8: int8_t, DATA16: int16_t, DATA32: int32_t, DATAF: float）
    3,       // 表示桁数（整数部＋小数部の合計桁数）
    0        // 小数点以下の桁数
  }){}

  // 繰り返し実行される処理
  // センサーの値を更新するなどの処理を行います
  void Update() override {
    int count = millis() / 1000;
    value[0] = count;
    value[1] = count + 1;
    value[2] = count + 2;
    value[3] = count + 3;
    value[4] = count + 4;
  }

  // EV3へ送信するデータを設定する処理
  void GetData(byte *buffer) override {
    memcpy(buffer, &value, sizeof(value));
  }

private:
  // モードで使用する変数、8個中3個はダミー
  uint8_t value[8] = {0};
};

//-----------------------------------------//
// モード3
// float型のデータを2個送信する例
// floatはポートビュー未対応？
//-----------------------------------------//
class Mode3 : public SensorMode {
public:
  // モードの設定
  Mode3() : SensorMode({
    "Mode3", // モード名（ポートビュー表示用）
    -1,      // 送信値の最小値（適当な値でもOK）
    1,       // 送信値の最大値（適当な値でもOK）
    2,       // 送信データ数（設定可能値: 1, 2, 4, 8）
    DATAF,   // データ型（DATA8: int8_t, DATA16: int16_t, DATA32: int32_t, DATAF: float）
    5,       // 表示桁数（整数部＋小数部の合計桁数）
    4        // 小数点以下の桁数
  }){}

  // 繰り返し実行される処理
  // センサーの値を更新するなどの処理を行います
  void Update() override {
    float t = millis() / 2000.0f;
    value[0] = sin(t);
    value[1] = cos(t);
  }

  // EV3へ送信するデータを設定する処理
  void GetData(byte *buffer) override {
    memcpy(buffer, &value, sizeof(value));
  }
private:
  // モードで使用する変数
  float value[2] = {0.0};
};

//=================================================//
// EV3 UARTセンサーの設定
//=================================================//
// EV3UARTSensorのインスタンスを作成します
//
// Serial : EV3と通信するUART、Arduino Unoの場合SerialでOK
// 90     : センサーID、適当な値でOK
// 57600  : 通信速度（ボーレート）
EV3UARTSensor sensor(&Serial, 90, 57600); // 

//=================================================//
// モードの作成
//=================================================//
Mode1 mode1;
Mode2 mode2;
Mode3 mode3;

//=================================================//
// Arduino起動時に1回だけ実行される処理
//=================================================//
void setup() {
  // 作成したモードをセンサーに登録
  sensor.AddMode(&mode1);
  sensor.AddMode(&mode2);
  sensor.AddMode(&mode3);

  // EV3とのUART通信を開始
  sensor.Begin();
}

//=================================================//
// Arduino起動後、繰り返し実行される処理
//=================================================//
void loop() {
  // EV3との通信を処理
  sensor.Run();
}
