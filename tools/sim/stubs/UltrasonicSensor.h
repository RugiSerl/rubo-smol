#pragma once
// Simulated Grove ultrasonic ranger: same interface as
// robot/UltrasonicSensor.h, same timing. A chirp every 60 ms; its echo comes
// back after the time sound takes to go there and back, or not at all after
// 30 ms. It sees the walls from where the robot was when it chirped.
#include "SimWorld.h"
#include "Clock.h"
#include "Config.h"

class UltrasonicSensor {
public:
  static const uint32_t MEASUREMENT_INTERVAL_MS = sim::US_INTERVAL_MS;
  static const uint32_t MAX_RANGE_MM            = (uint32_t)sim::US_RANGE_MM;

  // The robot's begin() waits for an echo to decide the sensor is there.
  // Here it always is, unless it is switched off in Config.h.
  bool begin() {
    sim::frontEnabled = FRONT_SENSOR_ENABLED;
    _connected = FRONT_SENSOR_ENABLED;
    return _connected;
  }

  void update() {
    if (!_connected) return;
    uint32_t now = nowMs();

    if (_waiting) {
      if ((int32_t)(now - _readyAt) < 0) return;
      _waiting = false;
      if (_echo >= 0) {
        _status     = 0;
        _distanceMm = _echo;
      } else {
        _status     = _echo == -2 ? 2 : 1;   // 2: beyond its range, 1: no echo
        _distanceMm = -1;
      }
      if (sim::started()) {
        sim::frontReadings++;
        if (_status != 0) sim::frontMissing++;
      }
      sim::frontShownMm = _distanceMm;
      sim::frontShownOk = valid();
      return;
    }

    if (now - _triggeredAt >= MEASUREMENT_INTERVAL_MS) {
      _triggeredAt = now;
      _waiting     = true;
      // Unplugged, the line stays low: no echo.
      _echo = sim::frontUnplugged ? -1 : sim::frontEcho();
      if (_echo >= 0) _readyAt = now + (uint32_t)std::ceil(_echo * sim::US_MS_PER_MM);
      else            _readyAt = now + sim::US_TIMEOUT_MS + 1;
    }
  }

  bool connected() const  { return _connected; }
  bool valid() const      { return _connected && _status == 0; }
  int  distanceMm() const { return _distanceMm; }
  int  quality() const    { return -1; }
  int  status() const     { return _status; }

private:
  uint32_t _triggeredAt = 0;
  uint32_t _readyAt     = 0;
  bool     _waiting     = false;
  int      _echo        = -1;

  bool _connected  = false;
  int  _distanceMm = -1;
  int  _status     = -1;
};
