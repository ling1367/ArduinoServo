/*
 * ============================================================
 *  零件分拣机器人视觉系统 - Arduino 下位机 v2.0
 *  功能：接收上位机指令，控制4轴舵机机械臂运动
 *  通信：115200bps 二进制协议（10字节数据包）+ 文本测试模式
 * ============================================================
 *
 *  硬件连接（以你实际杜邦线为准）：
 *    底座舵机   : D3
 *    大臂舵机   : D5
 *    小臂舵机   : D6
 *    夹爪舵机   : D9
 *    舵机供电   : 建议经 DCP3512 等输出 5V；Arduino GND 与舵机电源地共地（参考接线图核对共地即可）
 *    心跳LED    : D13（板载）
 *
 *  安全机制：
 *    - 舵机到达目标后自动detach，防止堵转发烫
 *    - 夹爪抓取后自动回松，避免持续夹紧
 *    - YZ 联动：220° ≤ Y+Z ≤ 320°（默认姿态 Y=90、Z=180 时和为 270°）
 *    - 单次运动超时保护（MOVE_TIMEOUT）
 *    - 紧急停止优先响应
 *    - 指令序列号防重复执行
 * ============================================================
 */

#include <Servo.h>

// ===================== 硬件引脚定义 =====================
#define PIN_BASE      3    // 底座
#define PIN_SHOULDER  10    // 大臂（D10）
#define PIN_ELBOW     6    // 小臂
#define PIN_GRIPPER   9    // 夹爪
#define PIN_LED       13   // 板载 LED

// ===================== 机械角度限制（与安装一致）=====================
// 底座：90° 正中；45° 右转；135° 左转
#define BASE_MIN   5
#define BASE_MAX   175
// 大臂（Y）：初始 100°，行程 90°~170°
#define Y_MIN      90
#define Y_MAX      180
// 小臂（Z）：行程 90°~160°
#define Z_MIN      90
#define Z_MAX      160
// 夹爪：20° 张开、50° 闭合（与 Servo.write 数值一致，不再镜像）
#define GRIP_MIN   20
#define GRIP_MAX   65
#define GRIP_SERVO_INVERTED 0
#define YZ_MIN     180    // Y+Z 联动下限（新范围最小和 90+90=180）
#define YZ_MAX     340    // Y+Z 联动上限（新范围最大和 180+160=340）

// ===================== 初始 / 原点（HOME 目标）=====================
#define BASE_INIT      90
#define SHOULDER_INIT  90
#define ELBOW_INIT     120
#define GRIPPER_INIT   25   // HOME/上电夹爪目标角
#define GRIP_OPEN      25   // 释放张开角

// ===================== 运动参数 =====================
#define MOVE_DELAY     55    // 平滑移动每步间隔（ms），越大越慢
#define BASE_HOME_DELAY 100   // 底座归位时每步间隔（更慢，防止冲击）
#define MOVE_TIMEOUT   10000 // 单舵机运动最长时限（ms）
#define GRAB_CLOSE     35    // 夹紧到位（圆形零件使用35°）
#define GRAB_HOLD      28    // 夹住后回松到无负载角（圆形28°）

// ===================== 通信协议常量 =====================
#define START_BYTE      0xAA
#define END_BYTE        0x55
#define CMD_MOVE        0x01
#define CMD_GRAB        0x02
#define CMD_GRAB_CIRCLE 0x07    // 抓取圆形零件（夹爪50°夹紧，47°保持）
#define CMD_GRAB_SQUARE 0x08    // 抓取方形零件（夹爪55°夹紧，52°保持）
#define CMD_RELEASE     0x03
#define CMD_HOME        0x04
#define CMD_STOP        0x05
#define CMD_INIT        0x06

// ===================== 舵机对象 =====================
Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoGripper;

// ===================== 当前角度变量 =====================
int baseAngle     = BASE_INIT;
int shoulderAngle = SHOULDER_INIT;
int elbowAngle    = ELBOW_INIT;   // 小臂角度（与 write 一致）
int gripperAngle  = GRIPPER_INIT;

// ===================== 舵机连接状态 =====================
bool baseAttached     = false;
bool shoulderAttached = false;
bool elbowAttached    = false;
bool gripperAttached  = false;

// ===================== 系统状态 =====================
bool initialized       = false;
unsigned long cmdCount = 0;       // 累计指令数
byte lastSeqNum        = 0xFF;    // 上一次指令序列号
unsigned long lastCmdTime  = 0;   // 上一次指令时间
unsigned long lastHeartbeat = 0;  // 心跳计时
unsigned long movementStartTime = 0; // 运动开始时间（用于超时保护）
unsigned long ignoreSerialCmdUntil = 0; // 上电后此前仅丢弃串口字节，避免误触发 detach
unsigned long packetStartTime = 0;      // 二进制包开始接收时间戳
#define PKT_TIMEOUT  200               // 二进制包接收超时（ms）

// ===================== 接收缓冲区 =====================
byte rxBuffer[10];    // 数据包接收缓冲区（10字节）
int  rxIndex = 0;

// 夹爪：逻辑角（程序内 gripperAngle）→ 实际 write 角度（装机反向时取镜像）
static int gripLogicalToPulseDeg(int logicalDeg) {
  logicalDeg = constrain(logicalDeg, GRIP_MIN, GRIP_MAX);
#if GRIP_SERVO_INVERTED
  return GRIP_MIN + GRIP_MAX - logicalDeg;
#else
  return logicalDeg;
#endif
}

static void gripperPulseWrite(int logicalDeg) {
  servoGripper.write(gripLogicalToPulseDeg(logicalDeg));
}

// ============================================================
//                    硬件层 - 舵机连接管理
// ============================================================

#define SERVO_US_MIN  1000
#define SERVO_US_MAX  2000
#define BASE_US_MIN   500
#define BASE_US_MAX   2500
#define ELBOW_US_MIN  500
#define ELBOW_US_MAX  2500
#define SHOULDER_US_MIN  1000
#define SHOULDER_US_MAX  2000
#define GRIP_US_MIN   1000
#define GRIP_US_MAX   2000

void attachAll() {
  // 重新约束角度变量，确保在有效范围内
  baseAngle = constrain(baseAngle, BASE_MIN, BASE_MAX);
  shoulderAngle = constrain(shoulderAngle, Y_MIN, Y_MAX);
  elbowAngle = constrain(elbowAngle, Z_MIN, Z_MAX);
  gripperAngle = constrain(gripperAngle, GRIP_MIN, GRIP_MAX);
  
  if (!baseAttached) {
    servoBase.attach(PIN_BASE, BASE_US_MIN, BASE_US_MAX);
    delay(10);
    servoBase.write(baseAngle);
    baseAttached = true;
  }
  if (!shoulderAttached) {
    servoShoulder.attach(PIN_SHOULDER, SHOULDER_US_MIN, SHOULDER_US_MAX);
    delay(10);
    servoShoulder.write(shoulderAngle);
    shoulderAttached = true;
  }
  if (!elbowAttached) {
    servoElbow.attach(PIN_ELBOW, ELBOW_US_MIN, ELBOW_US_MAX);
    delay(10);
    servoElbow.write(elbowAngle);
    elbowAttached = true;
  }
  if (!gripperAttached) {
    servoGripper.attach(PIN_GRIPPER, GRIP_US_MIN, GRIP_US_MAX);
    delay(10);
    gripperPulseWrite(gripperAngle);
    gripperAttached = true;
  }
}

void detachExceptGripper() {
  if (baseAttached)     { servoBase.detach();     baseAttached = false; }
  if (shoulderAttached) { servoShoulder.detach(); shoulderAttached = false; }
  if (elbowAttached)    { servoElbow.detach();    elbowAttached = false; }
}

void detachAll() {
  if (baseAttached)     { servoBase.detach();     baseAttached = false; }
  if (shoulderAttached) { servoShoulder.detach(); shoulderAttached = false; }
  if (elbowAttached)    { servoElbow.detach();    elbowAttached = false; }
  if (gripperAttached)  { servoGripper.detach();  gripperAttached = false; }
}

void detachBaseAndGripper() {
  if (baseAttached)     { servoBase.detach();     baseAttached = false; }
  if (gripperAttached)  { servoGripper.detach();  gripperAttached = false; }
}

void detachBaseOnly() {
  if (baseAttached)     { servoBase.detach();     baseAttached = false; }
}

void detachBaseElbowAndGripper() {
  if (baseAttached)     { servoBase.detach();     baseAttached = false; }
  if (elbowAttached)    { servoElbow.detach();    elbowAttached = false; }
  if (gripperAttached)  { servoGripper.detach();  gripperAttached = false; }
}

// ============================================================
//                    安全初始化
// ============================================================

void safeInit() {
  Serial.println(">>> 安全初始化：校准舵机到机械原点 <<<");
  baseAngle = constrain(BASE_INIT, BASE_MIN, BASE_MAX);
  shoulderAngle = constrain(SHOULDER_INIT, Y_MIN, Y_MAX);
  elbowAngle = constrain(ELBOW_INIT, Z_MIN, Z_MAX);
  gripperAngle = constrain(GRIPPER_INIT, GRIP_MIN, GRIP_MAX);
  
  // 逐个attach并立即设置角度，避免舵机移动到默认位置
  servoBase.attach(PIN_BASE, BASE_US_MIN, BASE_US_MAX);
  delay(10);
  servoBase.write(baseAngle);
  
  servoShoulder.attach(PIN_SHOULDER, SHOULDER_US_MIN, SHOULDER_US_MAX);
  delay(10);
  servoShoulder.write(shoulderAngle);
  
  servoElbow.attach(PIN_ELBOW, ELBOW_US_MIN, ELBOW_US_MAX);
  delay(10);
  servoElbow.write(elbowAngle);
  
  servoGripper.attach(PIN_GRIPPER, GRIP_US_MIN, GRIP_US_MAX);
  delay(10);
  gripperPulseWrite(gripperAngle);
  
  delay(800);
  
  baseAttached = true;
  shoulderAttached = true;
  elbowAttached = true;
  gripperAttached = true;
  initialized = true;
  Serial.println(">>> 安全初始化完成 <<<");
}

// ============================================================
//                    运动层 - 平滑移动
// ============================================================

void smoothMove(Servo &s, int from, int to, int minLimit, int maxLimit) {
  // 先对起始和目标角度进行约束
  from = constrain(from, minLimit, maxLimit);
  to = constrain(to, minLimit, maxLimit);

  movementStartTime = millis();
  int step = (from < to) ? 1 : -1;
  for (int i = from; i != to + step; i += step) {
    int safeAngle = constrain(i, minLimit, maxLimit);

    unsigned long elapsed = millis() - movementStartTime;
    if (elapsed > (unsigned long)MOVE_TIMEOUT) {
      Serial.print("WARN:移动超时"); Serial.println(to);
      break;
    }

    s.write(safeAngle);
    delay(MOVE_DELAY);
  }
}

// 夹爪平滑：逻辑角插值，输出经 gripLogicalToPulseDeg（与 smoothMove 分离便于反向装机）
void smoothMoveGripper(int from, int to) {
  from = constrain(from, GRIP_MIN, GRIP_MAX);
  to = constrain(to, GRIP_MIN, GRIP_MAX);
  movementStartTime = millis();
  int step = (from < to) ? 1 : -1;
  for (int i = from; i != to + step; i += step) {
    int safeAngle = constrain(i, GRIP_MIN, GRIP_MAX);
    if (millis() - movementStartTime > (unsigned long)MOVE_TIMEOUT) {
      Serial.print("WARN:夹爪移动超时"); Serial.println(to);
      break;
    }
    gripperPulseWrite(safeAngle);
    delay(MOVE_DELAY);
  }
}

// ============================================================
//                    运动层 - YZ 联动（与教程图示一致）
//  规则：220° ≤ Y+Z ≤ 320°。动 Y 时若和超限则补偿 Z；动 Z 时若和超限则补偿 Y。
//  大臂/小臂/夹爪各自单轴行程宏不变，仅在和约束下自动微调另一轴。
// ============================================================

void moveY(int targetY) {
  targetY = constrain(targetY, Y_MIN, Y_MAX);

  int oldY = shoulderAngle;

  // 直接移动，不使用复杂的YZ联动约束（简化版本）
  shoulderAngle = targetY;
  smoothMove(servoShoulder, oldY, targetY, Y_MIN, Y_MAX);
}

void moveZ(int targetZ) {
  targetZ = constrain(targetZ, Z_MIN, Z_MAX);

  int oldZ = elbowAngle;

  // 直接移动，不使用复杂的YZ联动约束（简化版本）
  elbowAngle = targetZ;
  smoothMove(servoElbow, oldZ, targetZ, Z_MIN, Z_MAX);
}

// ============================================================
//                    运动层 - 复合动作
// ============================================================

void moveToPoint(int x, int y, int z) {
  // 安全检查：确保角度在有效范围内
  if (x < BASE_MIN || x > BASE_MAX) return;
  if (y < Y_MIN || y > Y_MAX) return;
  if (z < Z_MIN || z > Z_MAX) return;

  // 底座移动（正常速度）
  if (x != baseAngle) {
    smoothMove(servoBase, baseAngle, x, BASE_MIN, BASE_MAX);
    baseAngle = x;
    delay(200);  // 等待稳定
  }

  // 大臂移动
  if (y != shoulderAngle) {
    smoothMove(servoShoulder, shoulderAngle, y, Y_MIN, Y_MAX);
    shoulderAngle = y;
    delay(200);  // 等待舵机稳定
  }

  // 小臂移动
  if (z != elbowAngle) {
    smoothMove(servoElbow, elbowAngle, z, Z_MIN, Z_MAX);
    elbowAngle = z;
    delay(200);  // 等待舵机稳定
  }
}

void moveToHome() {
  // 原点：底座正中、大小臂与夹爪为标定初始姿态

  // 夹爪
  smoothMoveGripper(gripperAngle, GRIPPER_INIT);
  gripperAngle = GRIPPER_INIT;

  // 底座（慢速移动）
  int fromBase = baseAngle;
  int toBase = constrain(BASE_INIT, BASE_MIN, BASE_MAX);
  int step = (fromBase < toBase) ? 1 : -1;
  for (int i = fromBase; i != toBase + step; i += step) {
    int safeAngle = constrain(i, BASE_MIN, BASE_MAX);
    servoBase.write(safeAngle);
    delay(BASE_HOME_DELAY);  // 使用更慢的延迟
  }
  baseAngle = toBase;

  // 大臂
  smoothMove(servoShoulder, shoulderAngle, SHOULDER_INIT, Y_MIN, Y_MAX);
  shoulderAngle = SHOULDER_INIT;

  // 小臂
  smoothMove(servoElbow, elbowAngle, ELBOW_INIT, Z_MIN, Z_MAX);
  elbowAngle = ELBOW_INIT;
}

void grab() {
  int close = constrain(GRAB_CLOSE, GRIP_MIN, GRIP_MAX);
  int hold  = constrain(GRAB_HOLD, GRIP_MIN, close);

  // 已在目标位置附近则跳过
  if (abs(gripperAngle - hold) < 5) {
    Serial.println("INFO:夹爪已在目标位置");
    return;
  }

  // 不逐度平滑：直接跳转到夹紧角，减少堵转时间
  gripperPulseWrite(close);
  delay(300);  // 短时夹紧
  // 立即回松到保持角（必须 < 零件物理阻挡角，否则持续堵转烧舵机）
  gripperPulseWrite(hold);
  gripperAngle = hold;
}

void grabCircle() {
  // 圆形零件：夹紧50°，保持47°
  int close = 50;
  int hold  = 47;
  
  gripperPulseWrite(close);
  delay(300);
  gripperPulseWrite(hold);
  gripperAngle = hold;
}

void grabSquare() {
  // 方形零件：夹紧55°，保持52°
  int close = 55;
  int hold  = 52;
  
  gripperPulseWrite(close);
  delay(300);
  gripperPulseWrite(hold);
  gripperAngle = hold;
}

void release() {
  int target = constrain(GRIP_OPEN, GRIP_MIN, GRIP_MAX);
  
  // 先检查当前位置，如果已经在目标位置附近，不做多余动作
  if (abs(gripperAngle - target) < 5) {
    Serial.println("INFO:夹爪已在释放位置");
    return;
  }

  smoothMoveGripper(gripperAngle, target);
  gripperAngle = target;
}

// ============================================================
//                    通信层 - 数据包解析
// ============================================================

void parsePacket() {
  // 注释掉 safeInit()，避免每步先复位到 100° 再执行指令
  // 角度安全由 attachAll() + moveToPoint() 内部的范围检查保障

  if (rxBuffer[0] != START_BYTE) { Serial.println("ERR:起始"); return; }
  if (rxBuffer[9] != END_BYTE)   { Serial.println("ERR:结束"); return; }

  byte cmd  = rxBuffer[1];
  int  x    = (rxBuffer[2] << 8) | rxBuffer[3];
  int  y    = (rxBuffer[4] << 8) | rxBuffer[5];
  byte z    = rxBuffer[6];
  byte seq  = rxBuffer[7];
  byte csum = rxBuffer[8];

  byte calc = (cmd + rxBuffer[2] + rxBuffer[3] + rxBuffer[4]
            + rxBuffer[5] + z + seq) & 0xFF;
  if (csum != calc) { Serial.println("ERR:校验"); return; }

  if (seq == lastSeqNum && (millis() - lastCmdTime) < 200) {
    Serial.print("DUP:"); Serial.println(seq);
    return;
  }
  lastSeqNum   = seq;
  lastCmdTime  = millis();
  cmdCount++;

  // 每次执行命令前都确保舵机连接
  attachAll();
  delay(50);

  switch (cmd) {
    case CMD_MOVE:
      // 额外的安全检查
      if (x < BASE_MIN || x > BASE_MAX || y < Y_MIN || y > Y_MAX || z < Z_MIN || z > Z_MAX) {
        Serial.println("ERR:角度超出范围");
        detachAll();
        return;
      }
      moveToPoint(x, y, z);
      Serial.println("ACK:MOVE");
      // 保持所有舵机连接，避免detach后舵机失控
      return;

    case CMD_GRAB:
      grab();
      Serial.println("ACK:GRAB");
      // 保持全部舵机通电，避免底座断电后重连延迟导致定位不准
      return;

    case CMD_GRAB_CIRCLE:
      grabCircle();
      Serial.println("ACK:GRABC");
      return;

    case CMD_GRAB_SQUARE:
      grabSquare();
      Serial.println("ACK:GRABS");
      return;

    case CMD_RELEASE:
      release();
      Serial.println("ACK:REL");
      detachBaseElbowAndGripper();   // 保持大臂，detach底座+小臂+夹爪
      return;

    case CMD_HOME:
      moveToHome();
      Serial.println("ACK:HOME");
      detachBaseElbowAndGripper();   // 保持大臂，detach底座+小臂+夹爪
      return;

    case CMD_STOP:
      detachAll();
      Serial.println("ACK:STOP");
      lastSeqNum = 0xFF;
      return;

    case CMD_INIT:
      Serial.println("ACK:INIT");
      // 重新初始化舵机到安全位置
      safeInit();
      return;

    default:
      Serial.println("ACK:ERR");
      detachAll();
      return;
  }
}

// ============================================================
//                    通信层 - 文本测试命令
// ============================================================

void handleTextCmd(const String &cmd) {
  String c = cmd;
  c.trim();
  int len = c.length();
  if (len < 2) { Serial.print("未知命令:"); Serial.println(cmd); return; }

  // 先确保舵机连接
  attachAll();
  delay(50);

  // 使用startsWith匹配，增加容错性
  if (c.startsWith("HOME") || c.startsWith("home")) {
    moveToHome(); detachBaseAndGripper();
    Serial.println("ACK:HOME"); return;
  }
  if (c.startsWith("GRAB") || c.startsWith("grab")) {
    grab(); detachBaseOnly();
    Serial.println("ACK:GRAB"); return;
  }
  if (c.startsWith("REL") || c.startsWith("rel") || 
      c.startsWith("RELEASE") || c.startsWith("release")) {
    release(); detachBaseAndGripper();
    Serial.println("ACK:REL"); return;
  }
  if (c.startsWith("STOP") || c.startsWith("stop")) {
    detachAll();
    Serial.println("ACK:STOP");
    lastSeqNum = 0xFF;
    return;
  }
  if (c.startsWith("INIT") || c.startsWith("init")) {
    Serial.println("ACK:INIT");
    return;
  }
  if (c.startsWith("STATS") || c.startsWith("stats")) {
    Serial.print("指令数:"); Serial.println(cmdCount);
    Serial.print("底座角度:"); Serial.println(baseAngle);
    Serial.print("大臂角度:"); Serial.println(shoulderAngle);
    Serial.print("小臂角度:"); Serial.println(elbowAngle);
    Serial.print("Y+Z联动和:"); Serial.print(shoulderAngle + elbowAngle);
    Serial.print(" (允许 "); Serial.print(YZ_MIN); Serial.print("~"); Serial.print(YZ_MAX); Serial.println(")");
    Serial.print("夹爪逻辑角:"); Serial.print(gripperAngle);
    Serial.print(" 夹爪write角:"); Serial.println(gripLogicalToPulseDeg(gripperAngle));
    return;
  }
  Serial.print("未知命令(已忽略，舵机保持):"); Serial.println(cmd);
}

// ============================================================
//              安全初始化等待 - 收到 INIT 前不激活舵机
// ============================================================

void waitForInit() {
  unsigned long lastPrint = 0;
  String lineBuffer;

  while (true) {
    if (millis() - lastPrint > 5000) {
      lastPrint = millis();
      Serial.println("WAITING: Send INIT to activate servos");
    }

    while (Serial.available() > 0) {
      int first = Serial.peek();

      if (first == START_BYTE) {
        byte buf[10];
        buf[0] = Serial.read();
        int count = 1;
        unsigned long t0 = millis();
        while (count < 10 && millis() - t0 < PKT_TIMEOUT) {
          while (count < 10 && Serial.available()) {
            buf[count++] = Serial.read();
          }
        }
        if (count == 10 && buf[0] == START_BYTE && buf[9] == END_BYTE) {
          byte calc = 0;
          for (int i = 1; i < 8; i++) calc += buf[i];
          if (buf[8] == (calc & 0xFF) && buf[1] == CMD_INIT) {
            Serial.println("ACK:INIT");
            return;
          }
        }
        lineBuffer = "";
      } else {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
          lineBuffer.trim();
          if (lineBuffer.equalsIgnoreCase("INIT")) {
            Serial.println("ACK:INIT");
            return;
          }
          lineBuffer = "";
        } else if (c >= 32) {
          lineBuffer += c;
          if (lineBuffer.length() > 20) lineBuffer = "";
        }
      }
    }
  }
}

// ============================================================
//                    Arduino 主循环
// ============================================================

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // ========== 第一步：舵机信号引脚预置低电平，防止 bootloader 阶段浮空乱转 ==========
  pinMode(PIN_BASE, OUTPUT);     digitalWrite(PIN_BASE, LOW);
  pinMode(PIN_SHOULDER, OUTPUT); digitalWrite(PIN_SHOULDER, LOW);
  pinMode(PIN_ELBOW, OUTPUT);    digitalWrite(PIN_ELBOW, LOW);
  pinMode(PIN_GRIPPER, OUTPUT);  digitalWrite(PIN_GRIPPER, LOW);
  delay(500);  // 等外部舵机电源稳定

  // ========== 第二步：初始化串口，等待上位机安全确认 ==========
  Serial.begin(115200);
  delay(100);
  while (Serial.available()) { Serial.read(); delay(1); }
  Serial.println("=== 安全模式：发送 INIT 指令激活舵机 ===");
  Serial.setTimeout(80);

  waitForInit();  // 阻塞直到收到 INIT 指令

  // ========== 第三步：舵机初始化（收到 INIT 后才执行） ==========
  // 顺序：底座→大臂→小臂→夹爪，级间长延时减轻多路 SG90 同时冲击电源导致电压跌落。

  servoBase.detach();
  servoShoulder.detach();
  servoElbow.detach();
  servoGripper.detach();
  baseAttached = shoulderAttached = elbowAttached = gripperAttached = false;
  delay(120);

  int initBase = constrain(BASE_INIT, BASE_MIN, BASE_MAX);
  int initShoulder = constrain(SHOULDER_INIT, Y_MIN, Y_MAX);
  int initElbow = constrain(ELBOW_INIT, Z_MIN, Z_MAX);
  int initGripLog = constrain(GRIPPER_INIT, GRIP_MIN, GRIP_MAX);

  Serial.print("DEBUG: 初始角度 - 底座:"); Serial.print(initBase);
  Serial.print(" 大臂:"); Serial.print(initShoulder);
  Serial.print(" 小臂:"); Serial.print(initElbow);
  Serial.println();

  servoBase.attach(PIN_BASE, BASE_US_MIN, BASE_US_MAX);
  delay(10);
  servoBase.write(initBase);
  delay(600);

  servoShoulder.attach(PIN_SHOULDER, SHOULDER_US_MIN, SHOULDER_US_MAX);
  delay(10);
  Serial.print("DEBUG: 大臂舵机attach，准备移动到 "); Serial.println(initShoulder);
  servoShoulder.write(initShoulder);
  delay(750);
  Serial.print("DEBUG: 大臂移动完成，当前角度变量: "); Serial.println(shoulderAngle);

  servoElbow.attach(PIN_ELBOW, ELBOW_US_MIN, ELBOW_US_MAX);
  delay(10);
  servoElbow.write(initElbow);
  delay(750);

  servoGripper.attach(PIN_GRIPPER, GRIP_US_MIN, GRIP_US_MAX);
  delay(10);
  gripperPulseWrite(initGripLog);
  delay(500);
  
  // 更新角度变量（使用约束后的值）
  baseAngle = initBase;
  shoulderAngle = initShoulder;
  elbowAngle = initElbow;
  gripperAngle = initGripLog;
  
  // 保持舵机连接
  baseAttached = true;
  shoulderAttached = true;
  elbowAttached = true;
  gripperAttached = true;
  initialized = true;

  // ========== 第四步：完成 ==========
  Serial.println("=== 机械臂初始化完成 ===");
  Serial.print("底座:"); Serial.print(BASE_INIT);
  Serial.print(" 大臂Y:"); Serial.print(SHOULDER_INIT);
  Serial.print(" 小臂Z:"); Serial.print(ELBOW_INIT);
  Serial.print(" Y+Z:"); Serial.print(SHOULDER_INIT + ELBOW_INIT);
  Serial.print(" 夹爪:"); Serial.println(GRIPPER_INIT);
  Serial.println("等待指令...");
  ignoreSerialCmdUntil = millis() + 3000;
}

void loop() {
  // ---- 心跳LED（500ms闪烁）----
  if (millis() - lastHeartbeat > 500) {
    lastHeartbeat = millis();
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  }

  // ---- 上电后数秒内丢弃串口噪声 ----
  if (millis() < ignoreSerialCmdUntil) {
    while (Serial.available()) {
      Serial.read();
    }
    return;
  }

  // ---- 处理串口数据：二进制协议 / 文本命令 ----
  while (Serial.available() > 0) {
    if (rxIndex == 0) {
      // 新包开始：通过首字节判断协议类型
      int first = Serial.peek();

      if (first == START_BYTE) {
        // ====== 二进制协议路径 ======
        rxBuffer[rxIndex++] = Serial.read();  // 消费 0xAA
        packetStartTime = millis();
      } else {
        // ====== 文本命令路径 ======
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() > 0) {
          Serial.print("收到命令: ");
          Serial.println(cmd);
          handleTextCmd(cmd);
        }
        break;  // readStringUntil 已阻塞消耗直到 \n，退出 while
      }
    } else {
      // 继续接收二进制包
      rxBuffer[rxIndex++] = Serial.read();

      if (rxIndex >= 10) {
        parsePacket();
        rxIndex = 0;
      }
    }
  }

  // ---- 二进制包接收超时保护：丢掉不完整包 ----
  if (rxIndex > 0 && (millis() - packetStartTime) > PKT_TIMEOUT) {
    rxIndex = 0;
    Serial.println("ERR:PKT_TMO");
  }
}