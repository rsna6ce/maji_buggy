#include <stdint.h>
#include <Bluepad32.h>
#include <time.h>
#include "esp_wps.h"
#include <ESP32Servo.h>
#include <esp_task_wdt.h>

#define MOTOR_F 2
#define MOTOR_B 3
void moter_write(int ch, int value) {
    ledcWrite(ch, value);
}


//3 seconds WDT
#define WDT_TIMEOUT 3

// BTコントローラ（最大4台）
ControllerPtr myControllers[BP32_MAX_GAMEPADS] = {nullptr};

static Servo servo;

const int pinMoterForward  = 27;
const int pinMoterBackward = 32;
const int pinServoSteering = 25;
const int pinLED = 2;

const int steeringRight = 50; // center - 40
const int steeringLeft = 120; // center + 40
const int steeringCenter = 90;

const int pwm_freq = 1000;
const int pwm_bit = 8;
const int pwm_max = (1 << pwm_bit);
const int throttleStop = 0;
const int throttleMax = pwm_max;

// BT接続時のコールバック
void onConnectedController(ControllerPtr ctl) {
  bool found = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;
      found = true;
      break;
    }
  }
  if (found) {
    Serial.println("=== コントローラ接続されました！ ===");
    Serial.print("モデル: ");
    Serial.println(ctl->getModelName());

    // MACアドレス出力（配列で取得）
    const uint8_t* addr = ctl->getProperties().btaddr;
    Serial.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
  } else {
    Serial.println("接続スロット満杯！");
  }
}

// BT接続時のコールバック
void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      break;
    }
  }
  Serial.println("=== コントローラ切断されました！ ===");
}

void setup() {
  Serial.begin(115200);
  Serial.println("hello. maji buggy.");

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableNewBluetoothConnections(true);
  Serial.println("スキャン開始... 8BitDo SN30 Pro を B + Start 長押しで起動");

  servo.setPeriodHertz(50);
  servo.attach(pinServoSteering, 400, 2400); // for sg90
  servo.write(steeringCenter);
  pinMode(pinLED, OUTPUT);
  pinMode(pinMoterForward, OUTPUT);
  digitalWrite(pinMoterForward, LOW);
  pinMode(pinMoterBackward, OUTPUT);
  digitalWrite(pinMoterBackward, LOW);

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
  BP32.update();

  int steeringAngleCurr = steeringCenter;
  int moterOutputForwardCurr = throttleStop;
  int moterOutputBackwardCurr = throttleStop;

  // コントローラーから
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] != nullptr && myControllers[i]->isConnected()) {
      ControllerPtr ctl = myControllers[i];

      uint16_t buttons = ctl->buttons();  // フェイス + ショルダー (A/B/X/Y/L1/R1)
      uint8_t dpad = ctl->dpad();         // D-Pad (ビットマスク: 1=上, 2=下, 4=左, 8=右)
      uint16_t misc = ctl->miscButtons(); // Misc (SELECT/START/HOME など)
      int l2 = ctl->brake();              // L2トリガー (0~1023)
      int r2 = ctl->throttle();           // R2トリガー (0~1023)

      // ジョイスティック値取得
      int lx = ctl->axisX();      // 左スティック X (左右)
      int ly = ctl->axisY();      // 左スティック Y (上下)
      int rx = ctl->axisRX();     // 右スティック X
      int ry = ctl->axisRY();     // 右スティック Y

      String output = "";

      // フェイスボタン (ABXY)
      //if (buttons & 0x0001) output += "B ";
      if (buttons & 0x0002) {
        //output += "A ";
        steeringAngleCurr = steeringRight;
      }
      if (buttons & 0x0004) {
        //output += "Y ";
        steeringAngleCurr = steeringLeft;
      }
      //if (buttons & 0x0008) output += "X ";

      // D-Pad (十字キー)
      if (dpad & 0x01) {
        //output += "↑ ";
        moterOutputForwardCurr = throttleMax;
        moterOutputBackwardCurr = throttleStop;
      }
      if (dpad & 0x02) {
        //output += "↓ ";
        moterOutputForwardCurr = throttleStop;
        moterOutputBackwardCurr = throttleMax;
      }
      if (dpad & 0x04) {
        //output += "→ ";
        steeringAngleCurr = steeringRight;
      }
      if (dpad & 0x08) {
        //output += "← ";
        steeringAngleCurr = steeringLeft;
      }

      // L1/R1 (ショルダー)
      //if (buttons & 0x0010) output += "L1 ";
      //if (buttons & 0x0020) output += "R1 ";

      // L2/R2 (アナログトリガー)
      //if (l2 > 50) output += "L2(" + String(l2) + ") ";
      //if (r2 > 50) output += "R2(" + String(r2) + ") ";

      // Miscボタン (SELECT/START/HOME)
      //if (misc & 0x01) output += "HOME ";
      //if (misc & 0x02) output += "SELECT ";
      //if (misc & 0x04) output += "START ";

      // ジョイスティック（デッドゾーン ±30 以内は無視）
      if (abs(lx) > 30 || abs(ly) > 30) {
        //output += "左スティック: X=" + String(lx) + " Y=" + String(ly) + " ";
        if (ly < 0) {
          // forward
          moterOutputForwardCurr = (-ly  * throttleMax ) / 512;
          moterOutputBackwardCurr = 0;
        } else {
          // back
          moterOutputForwardCurr = 0;
          moterOutputBackwardCurr = (ly  * throttleMax ) / 508;
        }
      }
      if (abs(rx) > 30 || abs(ry) > 30) {
        //output += "右スティック: X=" + String(rx) + " Y=" + String(ry) + " ";
        if (rx < 0) {
          // left
          steeringAngleCurr = steeringCenter + ((steeringLeft-steeringCenter) * -rx) / 512;
        } else {
          // right
          steeringAngleCurr = steeringCenter + ((steeringRight - steeringCenter) * rx) / 512;
        }
      }
    }
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
  delay(10);
  esp_task_wdt_reset();
}