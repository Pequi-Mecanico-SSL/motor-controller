// HallSensor whose velocity decays with silence instead of dropping to zero:
// with no edge for t seconds the rotor cannot be faster than one sector / t.
#pragma once
#include <SimpleFOC.h>

class HallSensorX : public HallSensor {
 public:
  HallSensorX(int a, int b, int c, int pp)
      : HallSensor(a, b, c, pp), sector_(_2PI / (6.0f * pp)) {}

  // call from each hall ISR after handleA/B/C
  void noteEdge() {
    if (hall_state == last_state_) return;  // same glitch filter as updateState()
    last_state_ = hall_state;
    uint32_t now = micros();
    dt_ = (direction == dir_) ? now - t_ : 0;  // reversal: unknown, as the library
    dir_ = direction;
    t_ = now;
  }

  float getVelocity() override {
    noInterrupts();
    uint32_t t = t_, dt = dt_;
    int8_t d = dir_;
    interrupts();
    if (dt == 0) return 0.0f;
    uint32_t silence = micros() - t;
    if (silence > 1000000u) return 0.0f;  // < 0.13 rad/s for 1 s: call it stopped
    if (silence > dt) dt = silence;
    return d * sector_ / (dt * 1e-6f);
  }

  float getVelocityLib() { return HallSensor::getVelocity(); }

 private:
  float sector_;
  volatile int8_t last_state_ = -1;
  volatile int8_t dir_ = 0;
  volatile uint32_t t_ = 0, dt_ = 0;
};
