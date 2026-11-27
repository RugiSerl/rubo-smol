'use strict';

/**
 * The robot, simulated on the computer, for the phone page.
 *
 *     node client/simrobot.js                    a room, the page on http://localhost:8080/
 *     node client/simrobot.js --scene obstacle   another scene: salle, obstacle, mur, patrouille
 *     node client/simrobot.js --console          also type commands here, as in tools/pico.py console
 *     node client/simrobot.js --sans-firmware    no Robot.h: invented readings, wheels that move nothing
 *
 * robot/Robot.h is compiled by tools/sim.py, as for a simulation run, and
 * runs at the speed of the clock among the walls of tools/sim/stubs/SimWorld.h.
 * This program serves robot/web/ from disk and answers the routes of
 * robot/WebRadio.h for it, with the same limits:
 *
 *     http://localhost:8080/      the phone page
 *     http://localhost:8080/sim   the same page, beside the room seen from above
 *
 * Other options, as for tools/sim.py: --start MM, --heading DEG, --door A:B,
 * --obstacle-size MM, -D NAME=VALUE. --http-port N changes the port.
 */

const childProcess = require('child_process');
const fs = require('fs');
const http = require('http');
const os = require('os');
const path = require('path');
const readline = require('readline');
const { FakeRobot, nowMs, parseCommand, MAX_LINE } = require('./fakerobot.js');

const HTTP_PORT = 8080;          // the robot uses 80, which needs root here
const LINES_KEPT = 16;           // as in robot/WebRadio.h
const COMMANDS_KEPT = 8;
const WEB_TIMEOUT_MS = 1500;     // as in robot/Config.h, for --sans-firmware
const SCENES = ['salle', 'obstacle', 'mur', 'patrouille'];
const OBSTACLE_SCENES = ['obstacle', 'patrouille'];

const ROOT = path.join(__dirname, '..');
const WEB_DIR = path.join(ROOT, 'robot', 'web');
const SIM_PY = path.join(ROOT, 'tools', 'sim.py');
const VIEW = path.join(__dirname, 'simview.html');

/** What the robot has sent, numbered from 1, as in WebRadio.h. */
class Lines {
  constructor() {
    this.kept = [];
    this.lastId = 0;
  }

  push(text) {
    this.lastId += 1;
    this.kept.push({ id: this.lastId, text });
    if (this.kept.length > LINES_KEPT) this.kept.shift();
  }

  /** Every line after line n; a page ahead of us is talking to a robot that restarted. */
  after(n) {
    if (n > this.lastId) n = 0;
    return this.kept.filter((line) => line.id > n).map((line) => line.text + '\n').join('');
  }
}

/** What a run that stopped by itself means, for the terminal. */
function describeEnd(code, signal) {
  if (signal === 'SIGSEGV' || signal === 'SIGBUS') {
    return `crashed  ${signal} in the robot's code: it used memory it should not touch,\n` +
           '         often through an array index out of range.';
  }
  if (signal === 'SIGFPE') {
    return `crashed  ${signal} in the robot's code: an integer division by zero.`;
  }
  if (signal) return `crashed  ${signal} in the robot's code`;
  if (code === 3) return '';      // the program has said why already
  return `stopped  with status ${code}`;
}

/**
 * robot/Robot.h, compiled and running in the simulated room. Talks to the
 * program through the lines described in tools/sim/main.cpp.
 */
class Firmware {
  constructor(options, onLine) {
    this.options = options;
    this.onLine = onLine;          // every line the robot sends
    this.onRestart = () => {};
    this.child = null;
    this.dir = null;
    this.status = 'starting';      // running, stuck, crashed, stopped
    this.message = '';
    this.version = 0;              // goes up at each start, with the scene
    this.world = { robot: null, walls: [], scene: null, revision: 0 };
    this.pending = null;           // a scene description still arriving
    this.pose = null;
    this.consoleOpen = false;
  }

  get alive() { return this.status === 'running'; }

  /** Compile robot/Robot.h. Calls back with null, or with the compiler's messages. */
  compile(done) {
    const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'simrobot-'));
    const args = [SIM_PY, '--build', dir, ...this.options.defines.flatMap((d) => ['-D', d])];
    childProcess.execFile('python3', args, { encoding: 'utf8', cwd: ROOT }, (error, stdout, stderr) => {
      if (error) {
        fs.rmSync(dir, { recursive: true, force: true });
        done(stderr || String(error));
        return;
      }
      if (stderr) process.stderr.write(stderr);     // warnings
      this.next = { dir, binary: stdout.trim() };
      done(null);
    });
  }

  /** Stop whatever runs, and start what compile() built. */
  start() {
    this.stop();
    const { dir, binary } = this.next;
    this.dir = dir;
    const o = this.options;
    const args = ['--live', '--scene', o.scene, '--heading', String(o.heading)];
    if (o.start !== null) args.push('--start', String(o.start));
    if (o.obstacleSize !== null) args.push('--obstacle-size', String(o.obstacleSize));
    for (const door of o.doors) args.push('--door', door);

    const child = childProcess.spawn(binary, args, { stdio: ['pipe', 'pipe', 'inherit', 'pipe'] });
    this.child = child;
    this.status = 'running';
    this.message = '';
    this.version += 1;
    this.world = { robot: null, walls: [], scene: null, revision: 0 };
    this.pending = null;
    this.pose = null;
    this.onRestart();

    readline.createInterface({ input: child.stdio[3] }).on('line', (line) => {
      const space = line.indexOf(' ');
      const kind = line.slice(0, space);
      const rest = line.slice(space + 1);
      if (kind === 'line') this.onLine(rest);
      else if (kind === 'pose') this.pose = JSON.parse(rest);
      else if (kind === 'robot') this.pending = { robot: JSON.parse(rest), walls: [] };
      else if (kind === 'wall' && this.pending) this.pending.walls.push(JSON.parse(rest));
      else if (kind === 'scene') {
        const scene = JSON.parse(rest);
        if (this.pending) {
          this.world = { ...this.pending, scene, revision: this.world.revision + 1 };
          this.pending = null;
        } else {
          this.world.scene = scene;
          this.world.revision += 1;
        }
      }
    });
    // What Robot.h prints, as the console would show it.
    readline.createInterface({ input: child.stdout }).on('line', (line) => {
      console.log(line);
      if (line.startsWith('stuck')) this.message = '';
      if (line.startsWith('stuck') || (this.message && line.startsWith(' '))) {
        this.message = (this.message + ' ' + line.slice(9)).trim();
      }
    });
    child.stdin.on('error', () => {});      // it may have gone
    child.on('exit', (code, signal) => {
      if (this.child !== child) return;      // replaced on purpose
      this.child = null;
      const said = describeEnd(code, signal);
      if (said) console.log(said);
      this.status = signal ? 'crashed' : code === 3 ? 'stuck' : 'stopped';
      if (said) this.message = said.replace(/\s+/g, ' ').replace(/^\w+ +/, '');
      console.log('         The robot no longer answers. Correct Robot.h, then reload it from\n' +
                  `         http://localhost:${o.httpPort}/sim or restart this program.`);
    });
    if (this.consoleOpen) this.send('console 1');
  }

  stop() {
    if (this.child) {
      const child = this.child;
      this.child = null;
      child.kill();
    }
    if (this.dir) fs.rmSync(this.dir, { recursive: true, force: true });
    this.dir = null;
  }

  send(text) {
    if (this.child && this.child.stdin.writable) this.child.stdin.write(text + '\n');
  }

  command(line) { this.send('cmd ' + line); }
  heard() { this.send('heard'); }

  setConsole(open) {
    this.consoleOpen = open;
    this.send(open ? 'console 1' : 'console 0');
  }
}

/** client/fakerobot.js, for --sans-firmware. */
class Invented {
  constructor(options, onLine) {
    this.onLine = onLine;
    this.onRestart = () => {};
    this.robot = new FakeRobot(!options.quiet);
    this.heardAt = null;
    this.consoleOpen = false;
    this.alive = true;
    setInterval(() => {
      for (const line of this.robot.update(this.connected())) this.onLine(line);
    }, 5);
  }

  connected() {
    return this.consoleOpen || (this.heardAt !== null && nowMs() - this.heardAt < WEB_TIMEOUT_MS);
  }

  command(line) {
    const [command, error] = parseCommand(line);
    if (error) this.onLine(error);
    else for (const reply of this.robot.execute(command)) this.onLine(reply);
  }

  heard() { this.heardAt = nowMs(); }
  setConsole(open) { this.consoleOpen = open; }
  stop() {}
}

function readBody(request, done) {
  let body = '';
  request.setEncoding('utf8');
  request.on('data', (chunk) => { body += chunk; });
  request.on('end', () => done(body));
}

function serve(options) {
  const lines = new Lines();
  const onLine = (line) => {
    lines.push(line);
    if (options.console) console.log(line);
  };
  const robot = options.firmware ? new Firmware(options, onLine) : new Invented(options, onLine);
  const plain = { 'Content-Type': 'text/plain; charset=utf-8', 'Cache-Control': 'no-store' };
  const json = (response, status, value) => {
    response.writeHead(status, { 'Content-Type': 'application/json', 'Cache-Control': 'no-store' });
    response.end(JSON.stringify(value));
  };

  // The room, for client/simview.html. Not part of the robot: these
  // requests do not keep the WiFi link up.
  const simRoute = (request, response, url) => {
    const post = request.method === 'POST';
    if (url.pathname === '/sim') {
      fs.readFile(VIEW, (error, data) => {
        response.writeHead(error ? 404 : 200, { 'Content-Type': 'text/html; charset=utf-8', 'Cache-Control': 'no-store' });
        response.end(error ? 'nothing here\n' : data);
      });
    } else if (url.pathname === '/sim/state') {
      json(response, 200, options.firmware
        ? { firmware: true, status: robot.status, message: robot.message, version: robot.version,
            world: robot.world, pose: robot.pose, console: robot.consoleOpen }
        : { firmware: false });
    } else if (post && url.pathname === '/sim/reload' && options.firmware) {
      robot.compile((errors) => {
        if (errors) {
          process.stderr.write(errors);
          json(response, 200, { ok: false, message: errors });
          return;
        }
        robot.start();
        console.log('restarted with the new robot/Robot.h');
        json(response, 200, { ok: true });
      });
    } else if (post && url.pathname === '/sim/place' && options.firmware) {
      robot.send('place');
      json(response, 200, { ok: true });
    } else if (post && url.pathname === '/sim/obstacle' && options.firmware) {
      robot.send(`obstacle ${url.searchParams.get('present') === '1' ? 1 : 0}`);
      json(response, 200, { ok: true });
    } else if (post && url.pathname === '/sim/obstacle-size' && options.firmware) {
      const size = Number(url.searchParams.get('mm'));
      if (!Number.isFinite(size) || size <= 0) {
        json(response, 400, { ok: false, message: 'expected a size in mm above 0' });
        return;
      }
      robot.send(`obstacle size ${Math.round(size)}`);
      json(response, 200, { ok: true });
    } else if (post && url.pathname === '/sim/unplug' && options.firmware) {
      const sensor = url.searchParams.get('sensor') === 'side' ? 'side' : 'front';
      robot.send(`unplug ${sensor} ${url.searchParams.get('unplugged') === '1' ? 1 : 0}`);
      json(response, 200, { ok: true });
    } else {
      response.writeHead(404, plain);
      response.end('nothing here\n');
    }
  };

  const web = http.createServer((request, response) => {
    const url = new URL(request.url, 'http://robot');
    if (url.pathname === '/sim' || url.pathname.startsWith('/sim/')) {
      simRoute(request, response, url);
      return;
    }

    // A robot whose program has stopped answers nothing at all.
    if (!robot.alive) {
      request.socket.destroy();
      return;
    }
    robot.heard();

    if (request.method === 'POST' && url.pathname === '/api/command') {
      readBody(request, (body) => {
        // As WebRadio.h: one command per line, a trailing \r dropped, blank
        // lines skipped, at most COMMANDS_KEPT at once.
        const commands = body.split('\n').map((l) => l.replace(/\r$/, '')).filter((l) => l.length > 0);
        for (const line of commands.slice(0, COMMANDS_KEPT)) robot.command(line.slice(0, MAX_LINE - 1));
        if (commands.length > COMMANDS_KEPT) {
          response.writeHead(503, plain);
          response.end('too many commands at once\n');
        } else {
          response.writeHead(204, { 'Cache-Control': 'no-store' });
          response.end();
        }
      });
      return;
    }

    if (request.method === 'GET' && url.pathname === '/api/lines') {
      const after = Number(url.searchParams.get('after')) || 0;
      const text = lines.after(after);
      response.writeHead(200, { ...plain, 'X-Last-Line': String(lines.lastId) });
      response.end(text);
      return;
    }

    if (request.method === 'GET') {
      const name = url.pathname === '/' ? 'index.html' : url.pathname.slice(1);
      // The types robot/EmbedWeb.cmake gives; anything else is served as bytes.
      const types = { '.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css',
                      '.svg': 'image/svg+xml' };
      // Serve current files on every request.
      if (!name.includes('/') && !name.startsWith('.')) {
        fs.readFile(path.join(WEB_DIR, name), (error, data) => {
          if (error) {
            response.writeHead(404, plain);
            response.end('nothing here\n');
            return;
          }
          const type = types[path.extname(name)] || 'application/octet-stream';
          response.writeHead(200, { 'Content-Type': type + '; charset=utf-8',
                                    'Cache-Control': 'no-store' });
          response.end(data);
        });
        return;
      }
    }

    response.writeHead(404, plain);
    response.end('nothing here\n');
  });

  // A restarted robot has said nothing yet: the page sees the numbers go
  // back, as after a reboot.
  robot.onRestart = () => {
    lines.kept = [];
    lines.lastId = 0;
  };

  const listen = () => {
    web.listen(options.httpPort, () => {
      const what = options.firmware ? `robot/Robot.h, scene ${options.scene}` : 'no firmware, invented readings';
      console.log(`simulated robot: ${what}\n` +
                  `  phone page       http://localhost:${options.httpPort}/\n` +
                  (options.firmware ? `  page and room    http://localhost:${options.httpPort}/sim\n` : '') +
                  (options.console ? '  console          type commands here; ctrl-D closes it\n' : '') +
                  'ctrl-C to stop\n');
    });
  };

  // Lines typed before the robot runs wait in stdin until it does.
  const openConsole = () => {
    if (!options.console) return;
    robot.setConsole(true);
    const input = readline.createInterface({ input: process.stdin });
    input.on('line', (line) => {
      const command = line.replace(/\r$/, '');
      if (command.length > 0) robot.command(command.slice(0, MAX_LINE - 1));
    });
    input.on('close', () => {
      robot.setConsole(false);
      console.log('console closed: the robot has lost its link, as when tools/pico.py console\n' +
                  'is closed. The page can still drive it. ctrl-C to stop.');
    });
  };

  const quit = () => {
    robot.stop();
    console.log('\nstopped');
    process.exit(0);
  };
  process.on('SIGINT', quit);
  process.on('SIGTERM', quit);

  if (!options.firmware) {
    listen();
    openConsole();
    return;
  }
  robot.compile((errors) => {
    if (errors) {
      process.stderr.write(errors);
      console.error('\nThe page can be worked on without Robot.h:\n' +
                    '    node client/simrobot.js --sans-firmware');
      process.exit(1);
    }
    robot.start();
    listen();
    openConsole();
  });
}

function usage(problem) {
  console.error(`simrobot: ${problem}\n` +
                'usage: node client/simrobot.js [--scene NAME] [--console] [--start MM] [--heading DEG]\n' +
                '                               [--door A:B] [--obstacle-size MM] [-D NAME=VALUE]\n' +
                '                               [--http-port N] [--sans-firmware]');
  process.exit(1);
}

function main() {
  const options = {
    scene: 'salle', start: null, heading: 0, obstacleSize: null, doors: [], defines: [],
    httpPort: HTTP_PORT, console: false, firmware: true, quiet: false,
  };
  const args = process.argv.slice(2);
  for (let i = 0; i < args.length; i++) {
    const arg = args[i];
    const value = () => {
      if (i + 1 >= args.length) usage(`no value after ${arg}`);
      return args[++i];
    };
    const number = (text) => {
      const n = Number(text);
      if (!Number.isFinite(n)) usage(`${arg} ${text}: expected a number`);
      return n;
    };
    if (arg === '--scene') options.scene = value();
    else if (arg === '--start') options.start = number(value());
    else if (arg === '--heading') options.heading = number(value());
    else if (arg === '--obstacle-size') options.obstacleSize = number(value());
    else if (arg === '--door') options.doors.push(value());
    else if (arg === '-D') options.defines.push(value());
    else if (arg === '--http-port') options.httpPort = number(value());
    else if (arg === '--console') options.console = true;
    else if (arg === '--sans-firmware') options.firmware = false;
    else if (arg === '--quiet') options.quiet = true;
    else usage(`unknown option ${arg}`);
  }
  if (!SCENES.includes(options.scene)) usage(`--scene ${options.scene}: expected one of ${SCENES.join(', ')}`);
  if (options.obstacleSize !== null && options.obstacleSize <= 0)
    usage('--obstacle-size: expected a size in mm above 0');
  if (options.obstacleSize !== null && !OBSTACLE_SCENES.includes(options.scene))
    usage(`--obstacle-size: the ${options.scene} scene has no obstacle (--scene obstacle or patrouille)`);
  if (options.httpPort <= 0) usage('--http-port: expected a port number');
  for (const door of options.doors) {
    if (!/^-?[\d.]+:-?[\d.]+$/.test(door)) usage(`--door ${door}: expected FROM:TO in mm, such as 2000:2800`);
    if (options.scene !== 'mur' && options.scene !== 'patrouille') {
      usage(`--door: the ${options.scene} scene has no wall beside the robot (--scene mur or patrouille)`);
    }
  }
  serve(options);
}

if (require.main === module) main();

module.exports = { Lines };
