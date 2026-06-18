/*
 * 单舵机测试代码
 */

#include <Servo.h>

Servo servo;

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("=== 单舵机测试 ===");
  Serial.println("输入命令:");
  Serial.println("  ATTACH X  - 连接引脚X的舵机");
  Serial.println("  WRITE X   - 转到角度X");
  Serial.println("  DETACH    - 断开舵机");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.startsWith("ATTACH")) {
      int pin = cmd.substring(6).toInt();
      servo.attach(pin);
      Serial.print("已连接引脚 ");
      Serial.println(pin);
    } else if (cmd.startsWith("WRITE")) {
      int angle = cmd.substring(6).toInt();
      servo.write(angle);
      Serial.print("已转到 ");
      Serial.println(angle);
    } else if (cmd == "DETACH") {
      servo.detach();
      Serial.println("已断开");
    }
  }
}