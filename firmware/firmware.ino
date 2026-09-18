// CAN-controlled FOC motor node. B-G431B-ESC1 + Nanotec DF45L024048-A (8 pp),
// halls + lowside current sense, velocity mode with foc_current torque.
// Control plane: classic CAN @ 500k (FDCAN1, RX=PA11 TX=PB9, see PROTOCOL.md).
// Serial (115200 over ST-LINK) stays available for tuning: T/C/M/D/E commands,
// S1/S0 toggles a 100 Hz debug stream (format in loop()).
//
// Needs -DHAL_OPAMP_MODULE_ENABLED (build_opt.h) — without it current sensing
// is silently garbage.
//
// Per-board config: MOTOR_ID (or add the MCU UID to uid_table below) and
// CAN_TERM_DEFAULT (1 on exactly one ESC of the bus — the HAT is the other end).

#include <SimpleFOC.h>
#include "hallx.h"
#include <SimpleFOCDrivers.h>
#include "encoders/smoothing/SmoothingSensor.h"
#include "fdcan_simple.h"

#define FW_VERSION 2
#ifndef MOTOR_ID
#define MOTOR_ID 0
#endif
#ifndef CAN_TERM_DEFAULT
#define CAN_TERM_DEFAULT 0
#endif

// ---- protocol (PROTOCOL.md is the reference) ----
static const uint16_t ID_ESTOP      = 0x000;
static const uint16_t ID_SETPOINT   = 0x080;  // 4x int16, 0.01 rad/s
static const uint16_t ID_CMD_BASE   = 0x100;  // + motor id
static const uint16_t ID_TLM_BASE   = 0x200;  // + motor id
static const uint16_t ID_DIAG_BASE  = 0x280;  // + motor id
static const uint16_t ID_HELLO_BASE = 0x300;  // + motor id
static const uint16_t ID_GAIN_BASE  = 0x380;  // + motor id

enum CmdOp : uint8_t {
  OP_STOP = 0x00,      // target = 0 (stays enabled, brakes to zero)
  OP_VEL = 0x01,       // f32 rad/s
  OP_ENABLE = 0x02,
  OP_DISABLE = 0x03,
  OP_CURLIM = 0x04,    // f32 A — caps Iq command (PID_velocity.limit)
  OP_TELEM = 0x05,     // u16 ms, 0 = off
  OP_TERM = 0x06,      // u8 0/1 — onboard 120R
  OP_PING = 0x07,      // reply with hello frame
  OP_WATCHDOG = 0x08,  // u16 ms without any command/setpoint -> disable; 0 = off
  OP_GAIN = 0x09,      // u8 param idx [+ f32 = set]; replies value on 0x380+id
  OP_DIAG = 0x0A,      // u16 ms diag frame period, 0 = off
};

// known boards: MCU UID word0 -> motor id (fill in as boards get flashed;
// UID is printed on boot). Falls back to MOTOR_ID when absent.
struct UidEntry { uint32_t uid0; uint8_t id; };
static const UidEntry uid_table[] = {
    {0x2D0065, 0},  // ST-LINK serial 066AFF525266555067075243
    {0x460065, 1},  // ST-LINK serial 066BFF525266555067075648
    {0x49004E, 2},  // ST-LINK serial 0669FF565382515067184024
    {0x2E001C, 3},  // ST-LINK serial 066AFF565382515067183929
};

uint8_t motor_id = MOTOR_ID;

BLDCMotor motor = BLDCMotor(8);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH,
                                       A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
HallSensorX sensor = HallSensorX(A_HALL1, A_HALL2, A_HALL3, 8);
// serial debug stream (Commander 'S1'/'S0'): 100 Hz samples + every hall edge
volatile bool stream_on = false;
struct Trans { uint32_t t; uint8_t state; };
static const uint16_t RING = 128;
volatile Trans ring[RING];
volatile uint16_t ring_w = 0;
uint16_t ring_r = 0;
volatile uint32_t ntrans = 0;

static inline void logTrans() {
  ntrans++;
  if (!stream_on) return;
  uint16_t w = ring_w;
  ring[w].t = micros();
  ring[w].state = (digitalRead(A_HALL1) << 2) | (digitalRead(A_HALL2) << 1) |
                  digitalRead(A_HALL3);
  ring_w = (w + 1) % RING;
}
void doA() { sensor.handleA(); sensor.noteEdge(); logTrans(); }
void doB() { sensor.handleB(); sensor.noteEdge(); logTrans(); }
void doC() { sensor.handleC(); sensor.noteEdge(); logTrans(); }

// halls step 60 deg electrical; foc_current needs a continuous angle
SmoothingSensor smooth = SmoothingSensor(sensor, motor);

// 3 mOhm shunts, opamp gain -64/7 — board-specific
LowsideCurrentSense currentSense =
    LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

Commander command = Commander(Serial);
void onTarget(char* cmd)  { command.scalar(&motor.target, cmd); }
void onCurLim(char* cmd)  { command.scalar(&motor.PID_velocity.limit, cmd); }
void onMotor(char* cmd)   { command.motor(&motor, cmd); }
void onDisable(char* cmd) { (void)cmd; motor.disable(); Serial.println("disabled"); }
void onEnable(char* cmd)  { (void)cmd; motor.enable();  Serial.println("enabled"); }
void onStream(char* cmd)  { stream_on = cmd[0] != '0'; Serial.print("stream="); Serial.println(stream_on); }

// OP_GAIN param index -> tunable float (keep in sync with PROTOCOL.md)
static float* const gain_table[] = {
    &motor.PID_velocity.P,            // 0
    &motor.PID_velocity.I,            // 1
    &motor.PID_velocity.D,            // 2
    &motor.PID_velocity.output_ramp,  // 3
    &motor.LPF_velocity.Tf,           // 4
    &motor.PID_velocity.limit,        // 5 (Iq cap, same as OP_CURLIM)
    &motor.PID_current_q.P,           // 6
    &motor.PID_current_q.I,           // 7
    &motor.LPF_current_q.Tf,          // 8
    &motor.PID_current_d.P,           // 9
    &motor.PID_current_d.I,           // 10
    &motor.LPF_current_d.Tf,          // 11
};
static const uint8_t N_GAINS = sizeof(gain_table) / sizeof(gain_table[0]);

uint8_t init_flags = 0;              // bit0 cs_ok, bit1 foc_ok
uint32_t telem_period_ms = 100;      // 0 = off
uint32_t diag_period_ms = 0;         // 0 = off
uint32_t watchdog_ms = 0;            // 0 = off
uint32_t last_rx_ms = 0;
bool watchdog_tripped = false;

static int16_t clamp16(float v) {
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)v;
}

void sendHello() {
  uint8_t b[8];
  uint32_t uid0 = HAL_GetUIDw0();
  memcpy(&b[0], &uid0, 4);
  b[4] = FW_VERSION;
  b[5] = init_flags;
  b[6] = motor_id;
  b[7] = 0;
  canSend(ID_HELLO_BASE + motor_id, b, 8);
}

void sendTelemetry() {
  uint8_t b[8];
  int16_t vel = clamp16(motor.shaft_velocity * 100.0f);
  int16_t tgt = clamp16(motor.target * 100.0f);
  int16_t iq  = clamp16(motor.current.q * 1000.0f);
  memcpy(&b[0], &vel, 2);
  memcpy(&b[2], &tgt, 2);
  memcpy(&b[4], &iq, 2);
  b[6] = (motor.enabled ? 1 : 0) | ((init_flags & 0x02) ? 2 : 0) |
         (watchdog_tripped ? 4 : 0);
  float uq10 = motor.voltage.q * 10.0f;
  b[7] = (int8_t)((uq10 > 127.0f) ? 127 : (uq10 < -128.0f) ? -128 : uq10);
  canSend(ID_TLM_BASE + motor_id, b, 8);
}

// velocity-PID output + current-loop internals; P/I split is reconstructed
// offline from the main telemetry (P-term = P*(target-vel), I-term = rest)
void sendDiag() {
  uint8_t b[8];
  int16_t iqsp = clamp16(motor.current_sp * 1000.0f);
  int16_t id   = clamp16(motor.current.d * 1000.0f);
  int16_t uq   = clamp16(motor.voltage.q * 100.0f);
  uint16_t ang = (uint16_t)(int32_t)(_normalizeAngle(motor.electrical_angle) *
                                     (65536.0f / _2PI));
  memcpy(&b[0], &iqsp, 2);
  memcpy(&b[2], &id, 2);
  memcpy(&b[4], &uq, 2);
  memcpy(&b[6], &ang, 2);
  canSend(ID_DIAG_BASE + motor_id, b, 8);
}

void handleCommand(const CanFrame& f) {
  switch (f.buf[0]) {
    case OP_STOP: motor.target = 0.0f; break;
    case OP_VEL:
      if (f.len >= 5) memcpy(&motor.target, &f.buf[1], 4);
      break;
    case OP_ENABLE:  watchdog_tripped = false; motor.enable(); break;
    case OP_DISABLE: motor.target = 0.0f; motor.disable(); break;
    case OP_CURLIM:
      if (f.len >= 5) memcpy(&motor.PID_velocity.limit, &f.buf[1], 4);
      break;
    case OP_TELEM:
      if (f.len >= 3) { uint16_t p; memcpy(&p, &f.buf[1], 2); telem_period_ms = p; }
      break;
    case OP_TERM:
      if (f.len >= 2) digitalWrite(A_CAN_TERM, f.buf[1] ? HIGH : LOW);
      break;
    case OP_PING: sendHello(); break;
    case OP_WATCHDOG:
      if (f.len >= 3) { uint16_t w; memcpy(&w, &f.buf[1], 2); watchdog_ms = w; }
      break;
    case OP_GAIN: {
      if (f.len < 2 || f.buf[1] >= N_GAINS) break;
      if (f.len >= 6) memcpy(gain_table[f.buf[1]], &f.buf[2], 4);
      uint8_t b[5];  // echo current value (also acks a set)
      b[0] = f.buf[1];
      memcpy(&b[1], gain_table[f.buf[1]], 4);
      canSend(ID_GAIN_BASE + motor_id, b, 5);
      break;
    }
    case OP_DIAG:
      if (f.len >= 3) { uint16_t p; memcpy(&p, &f.buf[1], 2); diag_period_ms = p; }
      break;
  }
}

void handleCan() {
  CanFrame f;
  while (canRead(f)) {
    if (f.id == ID_ESTOP) {
      motor.target = 0.0f;
      motor.disable();
    } else if (f.id == ID_SETPOINT && f.len >= 2u * (motor_id + 1)) {
      int16_t v;
      memcpy(&v, &f.buf[2 * motor_id], 2);
      motor.target = v * 0.01f;
    } else if (f.id == ID_CMD_BASE + motor_id) {
      handleCommand(f);
    }
    last_rx_ms = millis();
  }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  SimpleFOCDebug::enable(&Serial);

  uint32_t uid0 = HAL_GetUIDw0();
  for (auto& e : uid_table)
    if (e.uid0 == uid0) motor_id = e.id;
  Serial.print("uid0=0x"); Serial.print(uid0, HEX);
  Serial.print(" motor_id="); Serial.println(motor_id);

  // CAN transceiver on (SHDN is active-high), termination per board role
  pinMode(A_CAN_SHDN, OUTPUT);
  digitalWrite(A_CAN_SHDN, LOW);
  pinMode(A_CAN_TERM, OUTPUT);
  digitalWrite(A_CAN_TERM, CAN_TERM_DEFAULT ? HIGH : LOW);

  const uint16_t rx_ids[] = {ID_ESTOP, ID_SETPOINT,
                             (uint16_t)(ID_CMD_BASE + motor_id)};
  bool can_ok = canInit(20, rx_ids, 3);  // prescaler 20 = 500 kbit
  Serial.print("canInit="); Serial.println(can_ok);

  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);
  smooth.phase_correction = -_PI_6;  // hall-specific: half a sector
  motor.linkSensor(&smooth);

  driver.voltage_power_supply = 22.0f;
  driver.init();
  motor.linkDriver(&driver);
  currentSense.linkDriver(&driver);
  currentSense.skip_align = true;   // pins/gains verified: align made no changes
  motor.sensor_direction = Direction::CW;  // measured; skips flaky auto-detect

  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::foc_current;

  motor.voltage_limit = 6.0f;
  motor.velocity_limit = 100.0f;
  motor.current_limit = 2.0f;
  motor.voltage_sensor_align = 2.0f;

  // bare-rotor tune 2026-07-04; retune with wheel inertia attached
  motor.PID_velocity.P = 0.02f;
  motor.PID_velocity.I = 0.2f;
  motor.PID_velocity.output_ramp = 100.0f;
  motor.LPF_velocity.Tf = 0.02f;

  motor.PID_current_q.P = 1.0f;
  motor.PID_current_q.I = 300.0f;
  motor.PID_current_d.P = 1.0f;
  motor.PID_current_d.I = 300.0f;
  motor.LPF_current_q.Tf = 0.005f;
  motor.LPF_current_d.Tf = 0.005f;

  motor.init();

  // after init (init resets it to current_limit): windup cap just above
  // measured breakaway (0.40-0.45 A) — excess integral bursts past target
  motor.PID_velocity.limit = 0.65f;

  int cs_ok = currentSense.init();
  motor.linkCurrentSense(&currentSense);
  Serial.print("currentSense.init="); Serial.println(cs_ok);

  int foc_ok = motor.initFOC();
  Serial.print("initFOC="); Serial.print(foc_ok);
  Serial.print(" zero_electric_angle="); Serial.println(motor.zero_electric_angle, 4);

  init_flags = (cs_ok == 1 ? 1 : 0) | (foc_ok == 1 ? 2 : 0);
  motor.target = 0.0f;

  command.add('T', onTarget, "target rad/s");
  command.add('C', onCurLim, "Iq cap A");
  command.add('M', onMotor, "motor cmds");
  command.add('D', onDisable, "disable");
  command.add('E', onEnable, "enable");
  command.add('S', onStream, "debug stream 0/1");

  last_rx_ms = millis();
  sendHello();
}

void loop() {
  motor.loopFOC();
  motor.move();
  command.run();
  handleCan();

  if (watchdog_ms && motor.enabled && millis() - last_rx_ms > watchdog_ms) {
    motor.target = 0.0f;
    motor.disable();
    watchdog_tripped = true;
  }

  static unsigned long last_tlm = 0;
  if (telem_period_ms && millis() - last_tlm >= telem_period_ms) {
    last_tlm = millis();
    sendTelemetry();
  }

  static unsigned long last_diag = 0;
  if (diag_period_ms && millis() - last_diag >= diag_period_ms) {
    last_diag = millis();
    sendDiag();
  }

  if (stream_on) {
    // S <us> <target> <target> <raw hall vel> <shaft_velocity> <ntrans> <Iq mA> <lib hall vel>, mrad/s
    static uint32_t last_sample = 0;
    uint32_t now = micros();
    if (now - last_sample >= 10000) {
      last_sample = now;
      char msg[96];
      snprintf(msg, sizeof msg, "S %lu %ld %ld %ld %ld %lu %ld %ld", (unsigned long)now,
               (long)(motor.target * 1000), (long)(motor.target * 1000),
               (long)(sensor.getVelocity() * 1000), (long)(motor.shaft_velocity * 1000),
               (unsigned long)ntrans, (long)(motor.current.q * 1000),
               (long)(sensor.getVelocityLib() * 1000));
      Serial.println(msg);
    }
    while (ring_r != ring_w) {
      Trans t = {ring[ring_r].t, ring[ring_r].state};
      ring_r = (ring_r + 1) % RING;
      char msg[32];
      snprintf(msg, sizeof msg, "H %lu %d", (unsigned long)t.t, t.state);
      Serial.println(msg);
    }
    return;
  }

  static unsigned long last_print = 0;
  if (millis() - last_print >= 500) {
    last_print = millis();
    Serial.print("tgt=");  Serial.print(motor.target, 2);
    Serial.print(" vel="); Serial.print(motor.shaft_velocity, 2);
    Serial.print(" Iq=");  Serial.print(motor.current.q, 3);
    Serial.print(" Uq=");  Serial.println(motor.voltage.q, 2);
  }
}
