// Runs robot/Robot.h against the simulated world in stubs/SimWorld.h.
// Built and started by tools/sim.py, which explains the options, and in
// real time by client/simrobot.js.
//
//     sim [--live] [--trace FILE] [--transcript FILE] [--duration MS]
//         [--scene NAME] [--start MM] [--heading DEG] [--seed N] [--door A:B]...
//         [--obstacle-size MM] [--send MS:LINE]... [--offline A:B]...
//         [--remove MS] [--place MS] [--unplug front|side:A:B]...
//
// Times are in ms after the first command (sim::COMMAND_MS).
//
// --live runs at the speed of the clock. It reads lines from stdin:
//
//     cmd <line>              a command, as typed in the console
//     heard                   the phone made a request: the WiFi link is up
//     console 0|1             the console is closed or open
//     obstacle 0|1            take the obstacle away or put it back
//     obstacle size MM        resize the obstacle
//     unplug front|side 0|1   plug a sensor back in, or pull it out
//     place                   put the robot back where it started
//
// and writes to file descriptor 3: the scene once, then the robot's pose
// every 50 ms and every line the robot sends. What Robot.h prints goes to
// stdout, as on the console.
//
// Exit status: 0 for a run that ended normally, 2 for bad arguments or a
// robot placed against a wall, 3 for a run that stopped letting time pass.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <algorithm>
#include <string>
#if defined(__unix__) || defined(__APPLE__)
#include <unistd.h>
#include <poll.h>
#include <time.h>
#define SIM_WATCHDOG 1
#endif

#include "Robot.h"     // robot/Robot.h, copied next to this file by tools/sim.py

static FILE* liveOut = nullptr; // file descriptor 3, in live mode
static void describeScene(FILE* out);

// The link to the computer or the phone. Lines in sim::inbox arrive when the
// timeline (tools/sim.py) or the live input (client/simrobot.js) puts them
// there.
class SimRadio : public Radio {
public:
  bool begin() override { return true; }
  void poll() override {}

  bool connected() override {
    if (!sim::live) return sim::linkUp;
    // As in WebRadio.h: up while the phone keeps asking. The console, while
    // open, is a cable.
    bool web = heardFrom && sim::now - lastHeard < WEB_TIMEOUT_MS;
    return consoleOpen || web;
  }

  bool receive(char* line, size_t maxLength) override {
    if (sim::inbox.empty()) return false;
    snprintf(line, maxLength, "%s", sim::inbox.front().c_str());
    sim::inbox.pop_front();
    sim::note('>', line);
    JsonReader command;
    if (command.parse(line) && command.has("mission") && command.integer("mission") != 0)
      sim::missionAsked = true;
    return true;
  }

  void send(const char* line) override {
    if (strstr(line, "\"done\":true") && (sim::leftCommand != 0 || sim::rightCommand != 0))
      sim::doneMoving++;
    if (liveOut) {
      fprintf(liveOut, "line %s\n", line);
      return;
    }
    sim::note('<', line);
    if (strstr(line, "\"error\"")) printf("robot    %s\n", line);
  }

  const char* describe() override { return "simulation"; }

  bool     consoleOpen = false;
  bool     heardFrom   = false;
  uint32_t lastHeard   = 0;
};

static SimRadio radio;

#ifdef SIM_WATCHDOG
// Every 2 s of real time: has simulated time moved? Time passes through the
// main loop, sleep_ms() and nowMs(). A loop in Robot.h that calls none of
// them never ends, on the robot as here.
static void watchdog(int) {
  if (sim::ticked) {
    sim::ticked = 0;
    alarm(2);
    return;
  }
  static const char stuck[] =
      "stuck    simulated time has not moved for 2 s of real time: a loop in\n"
      "         Robot.h never ends. A sensor changes only in update(), and time\n"
      "         passes only through nowMs() or sleep_ms(). On the robot the loop\n"
      "         would never end either, and nothing would check the deadlines.\n";
  static const char moving[] =
      "         The wheels were turning: on the robot they would not stop.\n";
  ssize_t ignored = write(1, stuck, sizeof(stuck) - 1);
  if (sim::leftCommand != 0 || sim::rightCommand != 0)
    ignored = write(1, moving, sizeof(moving) - 1);
  (void)ignored;
  _exit(3);
}

// ---- Live mode ----------------------------------------------------------

static double clockMs() {
  timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec * 1000.0 + t.tv_nsec / 1e6;
}

static double clockOrigin = 0;     // the clock when sim::now was 0
static std::string pending;        // a line from stdin, not complete yet

static void obey(const std::string& line) {
  if (line.rfind("cmd ", 0) == 0) {
    sim::inbox.push_back(line.substr(4));
  } else if (line == "heard") {
    radio.heardFrom = true;
    radio.lastHeard = sim::now;
  } else if (line.rfind("console ", 0) == 0) {
    radio.consoleOpen = line[8] == '1';
  } else if (line.rfind("obstacle size ", 0) == 0) {
    const char* text = line.c_str() + 14;
    char* end = nullptr;
    double mm = std::strtod(text, &end);
    if (end != text && sim::setObstacleSize(mm) && liveOut) describeScene(liveOut);
  } else if (line.rfind("obstacle ", 0) == 0) {
    sim::obstaclePresent = line[9] == '1';
  } else if (line.rfind("unplug front ", 0) == 0) {
    sim::frontUnplugged = line[13] == '1';
  } else if (line.rfind("unplug side ", 0) == 0) {
    sim::sideUnplugged = line[12] == '1';
  } else if (line == "place") {
    sim::putBack();
  }
}

// Reads what has arrived on stdin, waiting up to `ms` for something.
static void listen(int ms) {
  pollfd in = {0, POLLIN, 0};
  if (::poll(&in, 1, ms) <= 0) return;
  char buffer[4096];
  ssize_t n = read(0, buffer, sizeof(buffer));
  if (n <= 0) exit(0);             // client/simrobot.js has gone
  pending.append(buffer, (size_t)n);
  size_t end;
  while ((end = pending.find('\n')) != std::string::npos) {
    obey(pending.substr(0, end));
    pending.erase(0, end + 1);
  }
}

static void publishPose() {
  fprintf(liveOut, "pose {\"ms\":%u,\"x\":%.1f,\"y\":%.1f,\"heading\":%.2f,"
                "\"left\":%d,\"right\":%d,\"front_mm\":%d,\"front_ok\":%s,"
                "\"side_mm\":%d,\"side_ok\":%s,\"bumping\":%s,\"linked\":%s,"
                "\"obstacle\":%s,\"obstacle_mm\":%g,\"front_unplugged\":%s,"
                "\"side_unplugged\":%s}\n",
          sim::now, sim::x, sim::y, sim::heading * 180 / M_PI,
          sim::leftCommand, sim::rightCommand,
          sim::frontShownMm, sim::frontShownOk ? "true" : "false",
          sim::sideShownMm, sim::sideShownOk ? "true" : "false",
          sim::bumping ? "true" : "false", radio.connected() ? "true" : "false",
          sim::obstaclePresent ? "true" : "false", sim::obstacleSize,
          sim::frontUnplugged ? "true" : "false", sim::sideUnplugged ? "true" : "false");
}

// Once per simulated millisecond: take in what has arrived, then wait for
// the clock to catch up.
static void keepPace() {
  for (;;) {
    double ahead = sim::now - (clockMs() - clockOrigin);
    if (ahead <= 0) {
      listen(0);
      break;
    }
    listen((int)std::ceil(ahead));
  }
  if (sim::now % 50 == 0) publishPose();
}
#endif

// The robot's size and the scene, one JSON object per line, for the
// picture (tools/sim.py) or the live view (client/simrobot.js).
static void describeScene(FILE* out) {
  fprintf(out, "robot {\"half_length\":%g,\"half_width\":%g,\"side_ahead\":%g,"
               "\"side_cone\":%g,\"side_range\":%g,\"front_cone\":%g,\"front_range\":%g}\n",
          sim::HALF_LENGTH_MM, sim::HALF_WIDTH_MM, sim::SENSOR_AHEAD_MM,
          sim::TOF_HALF_CONE_DEG, sim::TOF_RANGE_MM, sim::US_HALF_CONE_DEG, sim::US_RANGE_MM);
  for (const sim::Wall& w : sim::walls) {
    fprintf(out, "wall {\"ax\":%g,\"ay\":%g,\"bx\":%g,\"by\":%g,\"obstacle\":%s,\"gaps\":[",
            w.ax, w.ay, w.bx, w.by, w.obstacle ? "true" : "false");
    for (size_t g = 0; g < w.gaps.size(); g++)
      fprintf(out, "%s[%g,%g]", g ? "," : "", w.gaps[g].from, w.gaps[g].to);
    fprintf(out, "]}\n");
  }
  fprintf(out, "scene {\"name\":\"%s\",\"wall_scene\":%s,\"start\":[%g,%g,%g]}\n",
          sim::sceneName.c_str(), sim::wallScene ? "true" : "false",
          sim::startX, sim::startY, sim::startHeading * 180 / M_PI);
}

static bool numbers(const char* text, double& a, double& b) {
  return sscanf(text, "%lf:%lf", &a, &b) == 2;
}

static void usage(const char* what) {
  fprintf(stderr, "sim: %s (run this through tools/sim.py)\n", what);
  exit(2);
}

int main(int argc, char** argv) {
  const char* tracePath = nullptr;
  const char* transcriptPath = nullptr;
  std::string scene = "mur";
  double start = -1, headingDeg = 0, obstacleSize = -1;
  unsigned seed = 1;
  std::vector<sim::Gap> doors;
  auto at = [](double ms) { return sim::COMMAND_MS + (uint32_t)(ms < 0 ? 0 : ms); };

  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--live") { sim::live = true; continue; }
    if (i + 1 >= argc) usage(("no value after " + arg).c_str());
    const char* value = argv[++i];
    double a, b;
    if      (arg == "--trace")      tracePath = value;
    else if (arg == "--transcript") transcriptPath = value;
    else if (arg == "--duration")   sim::durationMs = (uint32_t)atol(value);
    else if (arg == "--scene")      scene = value;
    else if (arg == "--start")      start = atof(value);
    else if (arg == "--heading")    headingDeg = atof(value);
    else if (arg == "--obstacle-size") obstacleSize = atof(value);
    else if (arg == "--seed")       seed = (unsigned)atol(value);
    else if (arg == "--door") {
      if (!numbers(value, a, b)) usage("--door A:B");
      doors.push_back({std::min(a, b), std::max(a, b)});
    } else if (arg == "--send") {
      const char* colon = strchr(value, ':');
      if (!colon) usage("--send MS:LINE");
      sim::timeline.push_back({at(atof(value)), sim::Event::Send, colon + 1});
    } else if (arg == "--offline") {
      if (!numbers(value, a, b)) usage("--offline A:B");
      sim::timeline.push_back({at(a), sim::Event::LinkDown, ""});
      sim::timeline.push_back({at(b), sim::Event::LinkUp, ""});
    } else if (arg == "--remove") {
      sim::timeline.push_back({at(atof(value)), sim::Event::Remove, ""});
    } else if (arg == "--place") {
      sim::timeline.push_back({at(atof(value)), sim::Event::Place, ""});
    } else if (arg == "--unplug") {
      bool front = strncmp(value, "front:", 6) == 0;
      bool side  = strncmp(value, "side:", 5) == 0;
      if ((!front && !side) || !numbers(value + (front ? 6 : 5), a, b))
        usage("--unplug front:A:B or side:A:B");
      sim::timeline.push_back({at(a), front ? sim::Event::UnplugFront : sim::Event::UnplugSide, ""});
      sim::timeline.push_back({at(b), front ? sim::Event::ReplugFront : sim::Event::ReplugSide, ""});
    } else {
      usage(("unknown option " + arg).c_str());
    }
  }
  // In order of time; events at the same time keep their order.
  std::stable_sort(sim::timeline.begin(), sim::timeline.end(),
                   [](const sim::Timed& p, const sim::Timed& q) { return p.at < q.at; });

  sim::rng.seed(seed);
  sim::frontRng.seed(seed + 1000);
  if (start < 0) start = sim::defaultStart(scene);
  if (!sim::setScene(scene, start, headingDeg, doors)) usage("unknown scene");
  if (obstacleSize >= 0 && !sim::setObstacleSize(obstacleSize))
    usage("--obstacle-size: this scene has no obstacle");

  if (tracePath) {
    sim::trace = fopen(tracePath, "w");
    if (!sim::trace) { perror(tracePath); return 2; }
    // One row at a time, so that a run that crashes or gets stuck still
    // leaves everything it recorded.
    setvbuf(sim::trace, nullptr, _IOLBF, 0);
    fprintf(sim::trace, "t_ms,x_mm,clearance_mm,heading_deg,left_cmd,right_cmd,"
                        "side_mm,side_ok,left_mm_s,right_mm_s,wall_mm,"
                        "y_mm,front_mm,front_ok,front_wall_mm\n");
  }
  if (transcriptPath) {
    sim::transcript = fopen(transcriptPath, "w");
    if (!sim::transcript) { perror(transcriptPath); return 2; }
    setvbuf(sim::transcript, nullptr, _IOLBF, 0);
  }
  setvbuf(stdout, nullptr, _IOLBF, 0);   // read through a pipe

  if (sim::hitSomething()) {
    printf("error    the robot starts against a wall: raise --start or bring"
           " --heading closer to 0\n");
    return 2;
  }

  // For the picture: tools/sim.py reads these lines and does not show them.
  if (!sim::live) describeScene(stdout);

#ifdef SIM_WATCHDOG
  if (sim::live) {
    liveOut = fdopen(3, "w");
    if (!liveOut) usage("--live needs file descriptor 3");
    setvbuf(liveOut, nullptr, _IOLBF, 0);
    describeScene(liveOut);
    sim::onTick = keepPace;
    clockOrigin = clockMs() - sim::BOOT_MS;
  }
  signal(SIGALRM, watchdog);
  alarm(2);
#else
  if (sim::live) usage("--live needs Linux or macOS");
#endif

  // Static, as robot is a global in robot/main.cpp: a member with no
  // starting value then starts at 0, as on the robot.
  static Robot robot(radio);

  // robot/main.cpp sleeps 2 s, then calls begin().
  sim::now = sim::BOOT_MS;
  robot.begin();

  for (;;) {
    robot.update();
    sim::tick();       // calls sim::finish(), which exits, when the run is over
  }
}
