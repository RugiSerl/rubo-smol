#pragma once
// Simulated world for tools/sim.py and client/simrobot.js: a robot on a flat
// floor among walls. The stub headers in tools/sim/stubs/ read and write this
// state in place of the Pico's hardware, so robot/Robot.h compiles unchanged.
//
// Coordinates are in mm and radians. The robot starts at y = 0 or below,
// heading along +x. A positive turn rate turns left. In the wall scenes (mur,
// patrouille) the wall is the line y = 0, on the robot's left, with gaps for
// doors. The scenes are set up in setScene() below.
//
// Every value under "Model" is invented but plausible for the class chassis
// (4WD, TT gear motors, skid steering). They were chosen so that the
// behaviours described for the TPs appear; amplitudes, periods and stopping
// distances measured here will differ on the robot.

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <random>
#include <string>
#include <vector>

typedef unsigned int uint;   // as in pico/types.h

#ifndef M_PI
#define M_PI 3.14159265358979323846   // POSIX, not standard C++
#endif

namespace sim {

// ---- Model -------------------------------------------------------------
//
// Defaults for the SIM_ values. Any of them can be changed without editing:
//     python3 tools/sim.py -D SIM_MOTOR_DELAY_MS=50

// Wheels. Speed is zero up to the dead zone, then grows linearly with the
// command; it follows the command after a dead time, then with a lag.
#ifndef SIM_DEAD_ZONE
#define SIM_DEAD_ZONE        50     // |command| that just fails to turn a wheel
#endif
#ifndef SIM_MM_S_PER_STEP
#define SIM_MM_S_PER_STEP    2.5    // mm/s per command step: 150 gives 250 mm/s
#endif
#ifndef SIM_LEFT_STRENGTH
#define SIM_LEFT_STRENGTH    0.92   // left side 8% weaker: it drifts left
#endif
#ifndef SIM_MOTOR_DELAY_MS
#define SIM_MOTOR_DELAY_MS   100    // dead time: gear backlash, static friction
#endif
#ifndef SIM_MOTOR_LAG_MS
#define SIM_MOTOR_LAG_MS     80     // time constant of a wheel reaching its speed
#endif

// Skid steering scrubs the tyres sideways, so the robot turns as if its
// wheels were further apart than they are.
#ifndef SIM_TRACK_MM
#define SIM_TRACK_MM         360    // effective distance between left and right wheels
#endif

// Side sensor on the left flank, this far ahead of the point the robot turns
// about. The further forward, the earlier it sees the robot turning.
#ifndef SIM_SENSOR_AHEAD_MM
#define SIM_SENSOR_AHEAD_MM  60
#endif

constexpr double DEAD_ZONE       = SIM_DEAD_ZONE;
constexpr double MM_S_PER_STEP   = SIM_MM_S_PER_STEP;
constexpr double LEFT_STRENGTH   = SIM_LEFT_STRENGTH;
constexpr int    MOTOR_DELAY_MS  = SIM_MOTOR_DELAY_MS;
constexpr double MOTOR_LAG_MS    = SIM_MOTOR_LAG_MS;
constexpr double TRACK_MM        = SIM_TRACK_MM;
constexpr double SENSOR_AHEAD_MM = SIM_SENSOR_AHEAD_MM;

// Body, centred on the point the robot turns about. The front sensor sits
// in the middle of the front.
constexpr double HALF_LENGTH_MM  = 100;
constexpr double HALF_WIDTH_MM   = 70;

// VL53L1X, short mode, as set up in robot/TofSensor.h.
constexpr uint32_t TOF_INTERVAL_MS   = 40;     // a measurement starts every 40 ms
constexpr uint32_t TOF_MEASURE_MS    = 33;     // and is ready 33 ms later
constexpr uint32_t TOF_POLL_MS       = 5;      // TofSensor polls this often
constexpr uint32_t TOF_RETRY_MS      = 1000;   // a lost sensor is looked for this often
constexpr double   TOF_RANGE_MM      = 1300;   // further than this: no reading
constexpr double   TOF_MAX_ANGLE_DEG = 60;     // a wall more oblique than this returns too little light
constexpr double   TOF_HALF_CONE_DEG = 13.5;   // a 27 degree cone: it reports the nearest point in it
constexpr double   TOF_NOISE_MM      = 3;      // plus 0.5% of the distance
constexpr double   TOF_DROPOUT       = 0.01;   // share of readings lost for no visible reason

// Grove ultrasonic ranger, as driven by robot/UltrasonicSensor.h. Sound
// bounces off a surface like light off a mirror: one more oblique than
// US_MAX_ANGLE_DEG sends the echo elsewhere.
constexpr uint32_t US_INTERVAL_MS    = 60;     // a chirp every 60 ms
constexpr uint32_t US_TIMEOUT_MS     = 30;     // no echo by then: nothing there
constexpr double   US_MS_PER_MM      = 0.0058; // echo time, there and back
constexpr double   US_RANGE_MM       = 3500;   // further than this: no reading
constexpr double   US_MAX_ANGLE_DEG  = 40;
constexpr double   US_HALF_CONE_DEG  = 15;     // a 30 degree cone
constexpr double   US_NOISE_MM       = 2;      // plus 1% of the distance
constexpr double   US_DROPOUT        = 0.02;   // chirps lost for no visible reason

// ---- Timeline ----------------------------------------------------------
//
// As on the robot: main.cpp sleeps 2 s before Robot::begin(), and the
// first command comes later still. Meanwhile the loop runs and the sensors
// measure, so the mission's first pass already has a reading, and nowMs()
// is well past 0. The trace and the summary count time from the command.
constexpr uint32_t BOOT_MS    = 2000;   // Robot::begin()
constexpr uint32_t COMMAND_MS = 5000;   // the first command arrives

// ---- Scene -------------------------------------------------------------

// A wall from (ax, ay) to (bx, by), less its gaps. The gaps are measured
// along the wall from (ax, ay). Walls marked as the obstacle can be taken
// away and put back during a run.
struct Gap { double from, to; };
struct Wall {
  double ax, ay, bx, by;
  std::vector<Gap> gaps;
  bool obstacle;
};

inline std::vector<Wall> walls;
inline bool        obstaclePresent = true;
inline bool        wallScene       = false;   // mur, patrouille: the wall y = 0 is on the left
inline std::string sceneName;
inline double      startX = 0, startY = 0, startHeading = 0;

// Calls f(ax, ay, bx, by, obstacle) for every stretch of wall that is there.
template <typename F>
inline void forEachPiece(F f) {
  for (const Wall& w : walls) {
    if (w.obstacle && !obstaclePresent) continue;
    double length = std::hypot(w.bx - w.ax, w.by - w.ay);
    double ux = (w.bx - w.ax) / length, uy = (w.by - w.ay) / length;
    double at = 0;
    for (const Gap& g : w.gaps) {
      if (g.from > at) f(w.ax + ux * at, w.ay + uy * at, w.ax + ux * g.from, w.ay + uy * g.from, w.obstacle);
      if (g.to > at) at = g.to;
    }
    if (at < length) f(w.ax + ux * at, w.ay + uy * at, w.bx, w.by, w.obstacle);
  }
}

// ---- State -------------------------------------------------------------

inline uint32_t now = 0;           // simulated milliseconds since boot

inline double x = 0, y = 0, heading = 0;       // heading kept within -pi..pi
inline double leftSpeed = 0, rightSpeed = 0;   // mm/s, actual
// volatile: main.cpp's watchdog reads them from a signal handler.
inline volatile int leftCommand = 0, rightCommand = 0;

// Commands as the wheels will feel them, MOTOR_DELAY_MS late.
inline int pastLeft[MOTOR_DELAY_MS + 1]  = {};
inline int pastRight[MOTOR_DELAY_MS + 1] = {};

// One generator per sensor, so that one sensor seeing something new does not
// change the other's noise.
inline std::mt19937 rng, frontRng;

inline bool     live          = false;  // paced to the clock, for client/simrobot.js
inline bool     bumping       = false;  // live: pushing against a wall
inline uint32_t durationMs    = 20000;  // counted from COMMAND_MS
inline FILE*    trace         = nullptr;
inline FILE*    transcript    = nullptr;  // what went to and from the robot
inline long     stops         = 0;      // wheels turning, then stopped: enforceSafety() or the mission
inline long     doneMoving    = 0;      // "done":true lines sent while the wheels were still turning
inline long     sideReadings  = 0, sideMissing  = 0;
inline long     frontReadings = 0, frontMissing = 0;
inline bool     sideEnabled   = false, frontEnabled = false;
inline bool     missionAsked  = false;  // a command with a mission other than 0
inline double   travelled     = 0;
inline uint32_t lastMovedAt   = COMMAND_MS;
inline bool     everMoved     = false;
inline long     callsThisTick = 0;

// The link, and what the computer or the phone has sent that the robot has
// not read yet.
inline bool linkUp = true;
inline std::deque<std::string> inbox;

// Sensors pulled out of their socket, by --unplug or from the live view.
inline bool frontUnplugged = false, sideUnplugged = false;

// Published by the sensor stubs, recorded in the trace.
inline int  sideShownMm  = -1;
inline bool sideShownOk  = false;
inline int  frontShownMm = -1;
inline bool frontShownOk = false;

// Set by every tick, cleared by main.cpp's watchdog: a loop in Robot.h that
// never lets simulated time pass leaves it cleared.
inline volatile std::sig_atomic_t ticked = 0;

// Live mode: called once per simulated millisecond, to keep pace with the
// clock and take in what the phone sends. Set by main.cpp.
inline void (*onTick)() = nullptr;

// Has the first command arrived?
inline bool started() { return now >= COMMAND_MS; }

// Milliseconds since the first command, as the trace and summary show them.
inline uint32_t sinceCommand() { return started() ? now - COMMAND_MS : 0; }

inline void note(char direction, const char* text) {
  if (transcript && started())
    fprintf(transcript, "%8.3f %c %s\n", sinceCommand() / 1000.0, direction, text);
}

// ---- Timed events, for tools/sim.py ------------------------------------

enum class Event { Send, LinkDown, LinkUp, Remove, Place, UnplugFront, ReplugFront, UnplugSide, ReplugSide };
struct Timed {
  uint32_t    at;      // simulated ms since boot
  Event       what;
  std::string text;    // the command, for Send
};
inline std::vector<Timed> timeline;   // in order of time
inline size_t             nextEvent = 0;
inline uint32_t           lastEventAt = 0;

inline void apply(const Timed& e) {
  lastEventAt = e.at;
  switch (e.what) {
    case Event::Send:        inbox.push_back(e.text); break;
    case Event::LinkDown:    linkUp = false;          note('-', "link lost"); break;
    case Event::LinkUp:      linkUp = true;           note('-', "link back"); break;
    case Event::Remove:      obstaclePresent = false; note('-', "obstacle removed"); break;
    case Event::Place:       obstaclePresent = true;  note('-', "obstacle put back"); break;
    case Event::UnplugFront: frontUnplugged = true;   note('-', "front sensor unplugged"); break;
    case Event::ReplugFront: frontUnplugged = false;  note('-', "front sensor plugged in again"); break;
    case Event::UnplugSide:  sideUnplugged = true;    note('-', "side sensor unplugged"); break;
    case Event::ReplugSide:  sideUnplugged = false;   note('-', "side sensor plugged in again"); break;
  }
}

// ---- Geometry ----------------------------------------------------------

inline double cross(double ax, double ay, double bx, double by) { return ax * by - ay * bx; }

// Gap between the left flank and the wall y = 0, at the point the robot
// turns about, measured square to the wall. This is the path drawn in
// run.svg for the wall scenes.
inline double clearance() { return -y - HALF_WIDTH_MM; }

// Does the segment a-b reach into the body? It is clipped against the
// body's four sides, in the robot's own frame.
inline bool crossesBody(double ax, double ay, double bx, double by) {
  double c = std::cos(heading), s = std::sin(heading);
  double au = (ax - x) * c + (ay - y) * s, av = -(ax - x) * s + (ay - y) * c;
  double bu = (bx - x) * c + (by - y) * s, bv = -(bx - x) * s + (by - y) * c;
  double t0 = 0, t1 = 1;
  // Keeps the part of the segment where p * t <= q.
  auto clip = [&](double p, double q) {
    if (p == 0) return q >= 0;
    double r = q / p;
    if (p < 0) { if (r > t1) return false; if (r > t0) t0 = r; }
    else       { if (r < t0) return false; if (r < t1) t1 = r; }
    return true;
  };
  double du = bu - au, dv = bv - av;
  return clip(du, HALF_LENGTH_MM - au) && clip(-du, HALF_LENGTH_MM + au) &&
         clip(dv, HALF_WIDTH_MM - av)  && clip(-dv, HALF_WIDTH_MM + av) && t0 <= t1;
}

// Is the body touching a wall? Sets touchedObstacle to say which kind.
inline bool touchedObstacle = false;
inline bool hitSomething() {
  bool hit = false;
  forEachPiece([&](double ax, double ay, double bx, double by, bool obstacle) {
    if (!hit && crossesBody(ax, ay, bx, by)) { hit = true; touchedObstacle = obstacle; }
  });
  return hit;
}

// The body's four corners, in order round it.
inline void corners(double cx[4], double cy[4]) {
  double c = std::cos(heading), s = std::sin(heading);
  int n = 0;
  for (int i : {1, 1, -1, -1}) {            // front, front, back, back
    int j = (n == 0 || n == 3) ? 1 : -1;    // left, right, right, left
    cx[n] = x + i * HALF_LENGTH_MM * c - j * HALF_WIDTH_MM * s;
    cy[n] = y + i * HALF_LENGTH_MM * s + j * HALF_WIDTH_MM * c;
    n++;
  }
}

inline double toSegment(double px, double py, double ax, double ay, double bx, double by) {
  double dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
  double t = l2 > 0 ? ((px - ax) * dx + (py - ay) * dy) / l2 : 0;
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  return std::hypot(ax + t * dx - px, ay + t * dy - py);
}

// Gap between the body and the closest wall: a corner, once the robot is
// at an angle. 0 if they touch.
inline double nearest() {
  double cx[4], cy[4];
  corners(cx, cy);
  double best = INFINITY;
  forEachPiece([&](double ax, double ay, double bx, double by, bool) {
    if (crossesBody(ax, ay, bx, by)) { best = 0; return; }
    for (int n = 0; n < 4; n++) {
      best = std::fmin(best, toSegment(cx[n], cy[n], ax, ay, bx, by));
      int m = (n + 1) % 4;
      best = std::fmin(best, toSegment(ax, ay, cx[n], cy[n], cx[m], cy[m]));
      best = std::fmin(best, toSegment(bx, by, cx[n], cy[n], cx[m], cy[m]));
    }
  });
  return best;
}

// Side sensor position: ahead of the turning point, on the left flank.
inline void sensorAt(double& sx, double& sy) {
  double c = std::cos(heading), s = std::sin(heading);
  sx = x + SENSOR_AHEAD_MM * c - HALF_WIDTH_MM * s;
  sy = y + SENSOR_AHEAD_MM * s + HALF_WIDTH_MM * c;
}

// Front sensor position: the middle of the front.
inline void frontAt(double& fx, double& fy) {
  fx = x + HALF_LENGTH_MM * std::cos(heading);
  fy = y + HALF_LENGTH_MM * std::sin(heading);
}

// Distance from (sx, sy) along the direction `angle` to the first wall, or
// -1 if there is none. Sets square to the distance from (sx, sy) to that
// wall's line, measured square to it.
inline double rayToWall(double sx, double sy, double angle, double* square = nullptr) {
  double dx = std::cos(angle), dy = std::sin(angle), best = -1;
  forEachPiece([&](double ax, double ay, double bx, double by, bool) {
    double ex = bx - ax, ey = by - ay;
    double d = cross(dx, dy, ex, ey);
    if (d == 0) return;
    double t = cross(ax - sx, ay - sy, ex, ey) / d;   // along the ray
    double u = cross(ax - sx, ay - sy, dx, dy) / d;   // along the wall
    if (t < 0 || u < 0 || u > 1) return;
    if (best >= 0 && t >= best) return;
    best = t;
    if (square) *square = std::fabs(cross(ex, ey, sx - ax, sy - ay)) / std::hypot(ex, ey);
  });
  return best;
}

// What a sensor at (sx, sy), pointing along `angle`, sees: the nearest
// point of any wall inside its cone, on a wall it faces squarely enough.
// -1 if there is none. On a long straight wall this is the distance square
// to the wall while the wall's perpendicular is inside the cone, and along
// the edge of the cone nearest to it once the robot turns further.
inline double nearestInCone(double sx, double sy, double angle, double halfConeDeg, double maxAngleDeg) {
  double h = halfConeDeg * M_PI / 180;
  double ax = std::cos(angle), ay = std::sin(angle);
  double lx = std::cos(angle + h), ly = std::sin(angle + h);   // left edge of the cone
  double rx = std::cos(angle - h), ry = std::sin(angle - h);   // right edge
  double squarely = std::cos(maxAngleDeg * M_PI / 180);
  double best = -1;
  forEachPiece([&](double px, double py, double qx, double qy, bool) {
    double dx = qx - px, dy = qy - py, length = std::hypot(dx, dy);
    if (length == 0) return;
    double nx = -dy / length, ny = dx / length;
    double side = (sx - px) * nx + (sy - py) * ny;          // which side of the wall the sensor is on
    double facing = -(ax * nx + ay * ny) * (side > 0 ? 1 : -1);
    if (side == 0 || facing < squarely) return;             // pointing away, or too oblique
    // The part of the wall inside the cone, from t0 to t1 along it.
    double t0 = 0, t1 = 1;
    auto keep = [&](double f0, double f1) {                 // keep where f >= 0
      if (f0 < 0 && f1 < 0) { t0 = 1; t1 = 0; return; }
      if (f0 >= 0 && f1 >= 0) return;
      double r = f0 / (f0 - f1);
      if (f0 < 0) t0 = std::fmax(t0, r); else t1 = std::fmin(t1, r);
    };
    keep(cross(rx, ry, px - sx, py - sy), cross(rx, ry, qx - sx, qy - sy));
    keep(-cross(lx, ly, px - sx, py - sy), -cross(lx, ly, qx - sx, qy - sy));
    if (t0 > t1) return;
    double t = ((sx - px) * dx + (sy - py) * dy) / (length * length);
    t = t < t0 ? t0 : t > t1 ? t1 : t;
    double d = std::hypot(px + t * dx - sx, py + t * dy - sy);
    if (best < 0 || d < best) best = d;
  });
  return best;
}

// Distance from the side sensor to the wall it points at, square to that
// wall: what the sensor would read if it pointed straight at it. -1 if
// there is none. In the wall scenes, the wall y = 0, doors or not.
inline double sensorToWall() {
  double sx, sy;
  sensorAt(sx, sy);
  if (wallScene) return sy < 0 ? -sy : -1;
  double square = -1;
  return rayToWall(sx, sy, heading + M_PI / 2, &square) >= 0 ? square : -1;
}

// Distance from the front sensor to the first wall straight ahead, or -1.
inline double frontToWall() {
  double fx, fy;
  frontAt(fx, fy);
  return rayToWall(fx, fy, heading);
}

// What the side sensor sees from the current pose: distance in mm, or -1.
inline int sideRay() {
  double sx, sy;
  sensorAt(sx, sy);
  double range = nearestInCone(sx, sy, heading + M_PI / 2, TOF_HALF_CONE_DEG, TOF_MAX_ANGLE_DEG);
  if (range < 0 || range > TOF_RANGE_MM) return -1;

  std::normal_distribution<double> noise(0, TOF_NOISE_MM + 0.005 * range);
  std::uniform_real_distribution<double> unit(0, 1);
  if (unit(rng) < TOF_DROPOUT) return -1;
  int mm = (int)std::lround(range + noise(rng));
  return mm < 0 ? 0 : mm;
}

// What the front sensor's echo says from the current pose: distance in mm,
// -1 for no echo at all, -2 for an echo from beyond its range.
inline int frontEcho() {
  double fx, fy;
  frontAt(fx, fy);
  double range = nearestInCone(fx, fy, heading, US_HALF_CONE_DEG, US_MAX_ANGLE_DEG);
  if (range < 0 || range > US_TIMEOUT_MS / US_MS_PER_MM) return -1;
  if (range > US_RANGE_MM) return -2;

  std::normal_distribution<double> noise(0, US_NOISE_MM + 0.01 * range);
  std::uniform_real_distribution<double> unit(0, 1);
  if (unit(frontRng) < US_DROPOUT) return -1;
  int mm = (int)std::lround(range + noise(frontRng));
  return mm < 1 ? 1 : mm;
}

inline double wheelSpeed(int command, double strength) {
  double magnitude = std::abs(command) - DEAD_ZONE;
  if (magnitude <= 0) return 0;
  return (command > 0 ? 1 : -1) * magnitude * MM_S_PER_STEP * strength;
}

// ---- Scenes ------------------------------------------------------------

constexpr double FAR = 100000;   // 100 m: the wall of the mur scene has no end in sight

// The removable obstacle of the patrouille scene: a 300 mm box in the
// robot's lane.
constexpr double BOX_FROM_X  = 1500;
constexpr double BOX_SIDE    = 300;
constexpr double PATROL_END  = 3500;   // the far wall, across the end of the wall

inline void box(double x0, double y0, double x1, double y1, bool obstacle) {
  walls.push_back({x0, y0, x1, y0, {}, obstacle});
  walls.push_back({x1, y0, x1, y1, {}, obstacle});
  walls.push_back({x1, y1, x0, y1, {}, obstacle});
  walls.push_back({x0, y1, x0, y0, {}, obstacle});
}

// The start distance each scene uses when none is given.
inline double defaultStart(const std::string& name) {
  if (name == "obstacle") return 800;
  if (name == "salle")    return 1400;
  return 260;
}

// Builds the scene and places the robot. `start` is the gap between the
// left flank and the wall in the wall scenes, and between the front and
// the wall ahead in the others. Returns false for an unknown name.
inline bool setScene(const std::string& name, double start, double headingDeg,
                     const std::vector<Gap>& doors) {
  sceneName = name;
  walls.clear();
  double h = std::remainder(headingDeg * M_PI / 180, 2 * M_PI);
  if (name == "mur" || name == "patrouille") {
    wallScene = true;
    double from = name == "mur" ? -FAR : -1000;
    double to   = name == "mur" ?  FAR : PATROL_END;
    std::vector<Gap> gaps;
    for (const Gap& d : doors) gaps.push_back({d.from - from, d.to - from});
    std::sort(gaps.begin(), gaps.end(), [](const Gap& p, const Gap& q) { return p.from < q.from; });
    walls.push_back({from, 0, to, 0, gaps, false});
    startX = 0;
    startY = -(start + HALF_WIDTH_MM);
    if (name == "patrouille") {
      walls.push_back({PATROL_END, 0, PATROL_END, -2000, {}, false});
      box(BOX_FROM_X, startY - BOX_SIDE / 2, BOX_FROM_X + BOX_SIDE, startY + BOX_SIDE / 2, true);
    }
  } else if (name == "obstacle") {
    // A wall across the robot's path, 2 m wide.
    wallScene = false;
    double at = HALF_LENGTH_MM + start;
    walls.push_back({at, -1000, at, 1000, {}, true});
    startX = 0;
    startY = 0;
  } else if (name == "salle") {
    // A room of 3 m by 2.4 m, the robot on its middle line facing +x.
    wallScene = false;
    box(-1500, -1200, 1500, 1200, false);
    startX = 1500 - HALF_LENGTH_MM - start;
    startY = 0;
  } else {
    return false;
  }
  startHeading = h;
  x = startX;
  y = startY;
  heading = h;
  return true;
}

// Picks the robot up and puts it back where it started, wheels at rest.
inline void putBack() {
  x = startX;
  y = startY;
  heading = startHeading;
  leftSpeed = rightSpeed = 0;
}

// ---- End of a run ------------------------------------------------------

inline void finish(const char* why) {
  if (trace) fclose(trace);
  if (transcript) fclose(transcript);
  printf("end      %s at %.2f s\n", why, sinceCommand() / 1000.0);
  if (wallScene)
    printf("position x = %.0f mm, clearance %.0f mm, heading %.1f deg\n",
           x, clearance(), heading * 180 / M_PI);
  else
    printf("position x = %.0f mm, y = %.0f mm, heading %.1f deg\n",
           x, y, heading * 180 / M_PI);
  printf("nearest  %.0f mm between the body and the closest wall\n", nearest());
  printf("driven   %.2f m\n", travelled / 1000);
  if (frontEnabled) printf("front    %ld readings, %ld missing\n", frontReadings, frontMissing);
  if (sideEnabled)  printf("side     %ld readings, %ld missing\n", sideReadings, sideMissing);
  printf("safety   %ld stops while the wheels were turning\n", stops);
  if (doneMoving > 0)
    printf("warning  %ld \"done\":true lines sent while the wheels were turning"
           " (sendTelemetry(true) in a mission?)\n", doneMoving);
  if (!everMoved && stops == 0 && missionAsked && sceneName == "mur" &&
      frontEnabled && frontReadings == frontMissing)
    printf("note     the wheels never turned. Nothing stands in front of the robot\n"
           "         in this scene, so the front sensor never reads anything: a\n"
           "         mission that waits for it does not move (--scene patrouille).\n");
  std::exit(0);
}

// One millisecond of simulated time.
inline void tick() {
  callsThisTick = 0;
  ticked = 1;
  constexpr double dt = 0.001;
  double k = MOTOR_LAG_MS > 1 ? 1.0 / MOTOR_LAG_MS : 1.0;   // per ms; above 1 it would overshoot
  int slot = now % (MOTOR_DELAY_MS + 1);
  pastLeft[slot]  = leftCommand;
  pastRight[slot] = rightCommand;
  int felt = (now + 1) % (MOTOR_DELAY_MS + 1);    // the oldest one kept
  leftSpeed  += (wheelSpeed(pastLeft[felt],  LEFT_STRENGTH) - leftSpeed)  * k;
  rightSpeed += (wheelSpeed(pastRight[felt], 1.0)           - rightSpeed) * k;

  double v = (leftSpeed + rightSpeed) / 2;
  double was[3] = {x, y, heading};
  heading += (rightSpeed - leftSpeed) / TRACK_MM * dt;
  heading = std::remainder(heading, 2 * M_PI);
  x += v * std::cos(heading) * dt;
  y += v * std::sin(heading) * dt;
  // Live, a robot that drives into a wall stays against it, its wheels
  // slipping, as a real one would.
  if (live) {
    bumping = hitSomething();
    if (bumping) { x = was[0]; y = was[1]; heading = was[2]; v = 0; }
  }
  travelled += std::abs(v) * dt;
  if (leftCommand != 0 || rightCommand != 0) {
    lastMovedAt = now;
    everMoved   = true;
  }

  now += 1;
  while (nextEvent < timeline.size() && timeline[nextEvent].at <= now) apply(timeline[nextEvent++]);
  if (onTick) onTick();
  if (live || !started()) return;

  uint32_t t = sinceCommand();
  if (trace && t % 10 == 0 && t > 0)
    fprintf(trace, "%u,%.1f,%.1f,%.2f,%d,%d,%d,%d,%.0f,%.0f,%.1f,%.1f,%d,%d,%.1f\n",
            t, x, clearance(), heading * 180 / M_PI,
            leftCommand, rightCommand, sideShownMm, sideShownOk ? 1 : 0,
            leftSpeed, rightSpeed, sensorToWall(),
            y, frontShownMm, frontShownOk ? 1 : 0, frontToWall());

  if (hitSomething()) finish(touchedObstacle ? "HIT THE OBSTACLE" : "HIT THE WALL");
  if (wallScene && std::fabs(y) - HALF_WIDTH_MM > 3000) finish("more than 3 m from the wall");
  if (t >= durationMs) finish("time up");
  // Only once nothing more is due to happen, and the robot has had 3 s to
  // answer the last thing that did: a robot waiting for the obstacle to go
  // is not finished.
  if (t > 3000 && now - lastMovedAt > 3000 && now - lastEventAt > 3000 &&
      nextEvent == timeline.size())
    finish("the wheels have not turned for 3 s");
}

}  // namespace sim

// ---- What the Pico SDK would provide -----------------------------------

// Time passes while the robot waits, but the loop does not run: nothing
// checks the deadlines and nothing reads the sensors.
inline void sleep_ms(uint32_t ms) { for (uint32_t i = 0; i < ms; i++) sim::tick(); }
inline void sleep_us(uint64_t us) { sleep_ms((uint32_t)(us / 1000)); }
inline void tight_loop_contents() {}
