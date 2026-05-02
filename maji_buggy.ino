#include <stdint.h>
#include <time.h>
#include "esp_wps.h"
#include <ESP32Servo.h>
#include <esp_task_wdt.h>
#include "MyconBT.h"
#include "SPIFFSIni.h"

#define MOTOR_F 2
#define MOTOR_B 3
void moter_write(int ch, int value) {
    ledcWrite(ch, value);
}


//3 seconds WDT
#define WDT_TIMEOUT 1

// MyconBT ESP-NOW
MyconReceiverBT receiver;

static Servo servo;

const int pinMoterForward  = 27;
const int pinMoterBackward = 32;
const int pinServoSteering = 25;
const int pinSTBY8833 = 4;
const int pinLED = 2;

static int steeringRight = 50; // center - 40
static int steeringLeft = 120; // center + 40
static int steeringCenter = 90;

const int pwm_freq = 1000;
const int pwm_bit = 8;
const int pwm_max = (1 << pwm_bit);
const int throttleStop = 0;
const int throttleMax = pwm_max;

void setup() {
  Serial.begin(115200);
  Serial.println("hello. maji buggy.");

  SPIFFSIni config("/config.ini", true);
  if (config.exist("steeringRight")) steeringRight = config.read("steeringRight").toInt();
  if (config.exist("steeringLeft")) steeringLeft = config.read("steeringLeft").toInt();
  if (config.exist("steeringCenter")) steeringCenter = config.read("steeringCenter").toInt();
  printConfig();

  servo.setPeriodHertz(50);
  servo.attach(pinServoSteering, 400, 2400); // for sg90
  servo.write(steeringCenter);
  pinMode(pinLED, OUTPUT);
  pinMode(pinMoterForward, OUTPUT);
  digitalWrite(pinMoterForward, LOW);
  pinMode(pinMoterBackward, OUTPUT);
  digitalWrite(pinMoterBackward, LOW);

  receiver.begin();
  receiver.set_debug_output(false);
  mycon_receiver_global_init(&receiver);
  Serial.println("スキャン開始... ESP-NOW mode.");

 
  ledcSetup(MOTOR_F, pwm_freq, pwm_bit);
  ledcSetup(MOTOR_B, pwm_freq, pwm_bit);
  ledcAttachPin(pinMoterForward, MOTOR_F);
  ledcAttachPin(pinMoterBackward, MOTOR_B);

  pinMode(pinLED, OUTPUT);
  digitalWrite(pinLED, LOW);

  //enable panic so ESP32 restarts
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
}


static int steeringAngleLast = steeringCenter;
static int moterOutputForwardLast = throttleStop;
static int moterOutputBackwardLast = throttleStop;
void loop() {
  int steeringAngleCurr = steeringCenter;
  int moterOutputForwardCurr = throttleStop;
  int moterOutputBackwardCurr = throttleStop;

  receiver.update();
  if (receiver.is_key_down(key_A)) {
    steeringAngleCurr = steeringRight;
  }
  if (receiver.is_key_down(key_Y)) {
    steeringAngleCurr = steeringLeft;
  }

  // D-Pad (十字キー)
  if (receiver.is_key_down(key_Upward)) {
    moterOutputForwardCurr = throttleMax;
    moterOutputBackwardCurr = throttleStop;
  }
  if (receiver.is_key_down(key_Downward)) {
    moterOutputForwardCurr = throttleStop;
    moterOutputBackwardCurr = throttleMax;
  }
  if (receiver.is_key_down(key_Right)) {
    steeringAngleCurr = steeringRight;
  }
  if (receiver.is_key_down(key_Left)) {
    steeringAngleCurr = steeringLeft;
  }

  // ジョイスティック（デッドゾーン ±30 以内は無視）
  int lx = receiver.get_joy1_x();
  int ly = receiver.get_joy1_y();
  if (abs(lx) > 30 || abs(ly) > 30) {
    // 左スティック
    if (ly < 0) {
      // forward
      moterOutputForwardCurr = (-ly  * throttleMax ) / 512;
      moterOutputBackwardCurr = 0;
    } else {
      // back
      moterOutputForwardCurr = 0;
      moterOutputBackwardCurr = min((ly  * throttleMax ) / 508, throttleMax);
    }
  }
  int rx = receiver.get_joy2_x();
  int ry = receiver.get_joy2_y();
  if (abs(rx) > 30 || abs(ry) > 30) {
    // 右スティック
    if (rx < 0) {
      // left
      steeringAngleCurr = steeringCenter + ((steeringLeft-steeringCenter) * -rx) / 512;
    } else {
      // right
      steeringAngleCurr = steeringCenter + ((steeringRight - steeringCenter) * rx) / 512;
    }
  }
  // ブレーキ(L1)
  if (receiver.is_key_down(key_L1)) {
    moterOutputForwardCurr = throttleMax;
    moterOutputBackwardCurr = throttleMax;
  }
  // 逆転ロックによるサイドターン(L2)
  if ((receiver.is_key_down(key_L2)) && (moterOutputForwardCurr > 0)) {
    moterOutputForwardCurr = throttleStop;
    moterOutputBackwardCurr = throttleMax;
  }

  // サーボ出力
  if (steeringAngleLast != steeringAngleCurr) {
      servo.write(steeringAngleCurr);
      steeringAngleLast = steeringAngleCurr;
      Serial.print("steeringAngleCurr ");
      Serial.println(steeringAngleCurr);
  }
  // モーター出力
  if (moterOutputForwardLast != moterOutputForwardCurr ||
    moterOutputBackwardLast != moterOutputBackwardCurr ) {
    if (moterOutputForwardCurr == 0) {
      moter_write(MOTOR_F, moterOutputForwardCurr);
      moter_write(MOTOR_B, moterOutputBackwardCurr);
    } else {
      moter_write(MOTOR_B, moterOutputBackwardCurr);
      moter_write(MOTOR_F, moterOutputForwardCurr);
    }
    moterOutputForwardLast = moterOutputForwardCurr;
    moterOutputBackwardLast = moterOutputBackwardCurr;
    Serial.print("moterOutputForwardCurr ");
    Serial.println(moterOutputForwardCurr);
    Serial.print("moterOutputBackwardCurr ");
    Serial.println(moterOutputBackwardCurr);
  }

  // コンフィグ
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // 前後の空白・改行を除去

    if (input.length() == 0) return;  // 空行なら無視

    if (input == "config") {
      printConfig();
    }
    else if (input.indexOf('=') > 0) {
      // 設定コマンド形式： steeringRight=140  のような形式を想定
      int eqPos = input.indexOf('=');
      String key   = input.substring(0, eqPos);
      String value = input.substring(eqPos + 1);

      key.trim();
      value.trim();

      int newValue = value.toInt();
      // 整数変換に失敗した場合（空文字や "abc" など）は 0 になるので簡易チェック
      if (value.length() > 0 && String(newValue) == value) {
        bool updated = false;

        if (key == "steeringRight") {
          steeringRight = newValue;
          updated = true;
        }
        else if (key == "steeringCenter") {
          steeringCenter = newValue;
          updated = true;
        }
        else if (key == "steeringLeft") {
          steeringLeft = newValue;
          updated = true;
        }

        if (updated) {
          // SPIFFS に保存
          SPIFFSIni config("/config.ini", true);
          config.write(key.c_str(), String(newValue));

          Serial.print(key);
          Serial.print(" = ");
          Serial.print(newValue);
        }
        else {
          Serial.print("不明なキー: ");
          Serial.println(key);
        }
      }
      else {
        Serial.println("エラー: 値が整数ではありません");
      }
    }
    else {
      Serial.println("不明なコマンド。例: config  または steeringRight=140");
    }
  }

  // blink
  digitalWrite(pinLED, ((millis() / 100) & 0x0001));

  delay(10);
  esp_task_wdt_reset();
}

void printConfig()
{
  Serial.println("--- config ---");
  Serial.print("steeringRight  = "); Serial.println(steeringRight);
  Serial.print("steeringCenter = "); Serial.println(steeringCenter);
  Serial.print("steeringLeft   = "); Serial.println(steeringLeft);
}