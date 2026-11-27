#pragma once
// Simulated VL53L1X: same interface as robot/TofSensor.h, same timing. Each
// measurement sees the walls from where the robot was when it started.
//
// It reads the clock through nowMs(), as robot/TofSensor.h does, so a loop
// that calls side.update() until a reading arrives lets time pass, as it
// would on the robot.
#include "SimWorld.h"
#include "Clock.h"
#include "Config.h"

class TofSensor {
public:
  static const uint32_t INTERVAL_MS      = sim::TOF_INTERVAL_MS;
  static const uint32_t POLL_INTERVAL_MS = sim::TOF_POLL_MS;

  // As on the robot, a sensor switched off in Config.h never reads anything.
  bool begin() {
    sim::sideEnabled = SIDE_SENSOR_ENABLED;
    if (!SIDE_SENSOR_ENABLED) return false;
    return start();
  }

  void update() {
    if (!_connected) {
      lookAgain();
      return;
    }
    uint32_t now = nowMs();
    if (now - _lastPollMs < POLL_INTERVAL_MS) return;
    _lastPollMs = now;

    // Unplugged: the next transfer fails, as on the robot.
    if (sim::sideUnplugged) {
      _connected  = false;
      _status     = -1;
      _distanceMm = -1;
      _quality    = -1;
      _pending    = false;
      publish();
      return;
    }

    if (_pending && (int32_t)(now - _readyAt) >= 0) {
      _pending    = false;
      _distanceMm = _measured;
      _status     = _measured >= 0 ? 0 : 2;     // 2: not enough light came back
      _quality    = _measured >= 0 ? 4000 : 0;
      if (sim::started()) {
        sim::sideReadings++;
        if (_measured < 0) sim::sideMissing++;
      }
      publish();
    }

    if ((int32_t)(now - _nextStart) >= 0) {
      // The chip keeps its own 40 ms rhythm. After a sleep_ms() or a busy
      // wait, the measurements it made meanwhile are gone: start from the
      // latest one, rather than catching up one per poll.
      _nextStart += (now - _nextStart) / INTERVAL_MS * INTERVAL_MS;
      _measured  = sim::sideRay();
      _readyAt   = _nextStart + sim::TOF_MEASURE_MS;
      _pending   = true;
      _nextStart += INTERVAL_MS;
    }
  }

  bool connected() const { return _connected; }
  bool valid() const { return _connected && _status == 0; }

  int distanceMm() const { return _distanceMm; }
  int quality()    const { return _quality; }
  int status()     const { return _status; }

private:
  uint32_t _lastPollMs = 0;
  uint32_t _lastLookMs = 0;
  uint32_t _nextStart  = 0;
  uint32_t _readyAt    = 0;
  bool     _pending    = false;
  int      _measured   = -1;

  bool _connected  = false;
  int  _distanceMm = -1;
  int  _quality    = -1;
  int  _status     = -1;

  bool start() {
    if (sim::sideUnplugged) return false;
    _connected = true;
    _nextStart = nowMs();
    return true;
  }

  // As on the robot: a lost sensor is looked for once a second.
  void lookAgain() {
    if (!SIDE_SENSOR_ENABLED) return;
    if (nowMs() - _lastLookMs < sim::TOF_RETRY_MS) return;
    _lastLookMs = nowMs();
    start();
  }

  void publish() {
    sim::sideShownMm = _distanceMm;
    sim::sideShownOk = valid();
  }
};
