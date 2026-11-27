'use strict';

/**
 * A robot with no firmware behind it, for `node client/simrobot.js
 * --sans-firmware`: the page can be worked on even while robot/Robot.h does
 * not compile.
 *
 * Commands, movement deadlines, telemetry and link timeout mirror the
 * firmware; its distance readings are invented, and its wheels move nothing.
 */

// Match the defaults in robot/Config.h.
const DEFAULT_DURATION_MS = 1000;
const MAX_DURATION_MS = 5000;
const DEFAULT_TELEMETRY_MS = 200;
const MOTOR_MAX_SPEED = 255;

const started = Date.now();

/** Milliseconds since startup, matching nowMs() in Clock.h. */
function nowMs() {
  return Date.now() - started;
}

/**
 * Robot state and command handling, independent of HTTP.
 * update() enforces movement deadlines and link safety without blocking.
 */
class FakeRobot {
  constructor(verbose = true) {
    this.left = 0;
    this.right = 0;
    this.seq = -1;
    this.mission = 0;
    this.telemetryEvery = DEFAULT_TELEMETRY_MS;
    this.verbose = verbose;

    this._deadline = 0;
    this._hasDeadline = false;
    this._lastTelemetry = 0;
  }

  /** Set wheel speeds and arm a deadline, as in Robot::drive(). */
  drive(left, right, durationMs = DEFAULT_DURATION_MS) {
    const duration = Math.min(durationMs, MAX_DURATION_MS);
    this.left = clamp(left);
    this.right = clamp(right);
    this._deadline = nowMs() + duration;
    this._hasDeadline = true;
  }

  stop() {
    this.left = 0;
    this.right = 0;
    this._hasDeadline = false;
  }

  moving() { return this.left !== 0 || this.right !== 0; }

  /**
   * Apply a parsed command and return any response lines. Settings are applied
   * before stop, which cancels movement and the mission.
   */
  execute(command) {
    const out = [];

    if (command.seq !== undefined && command.seq !== null) {
      this.seq = asNumber(command.seq);
    }
    if (command.telemetry !== undefined && command.telemetry !== null) {
      let period = asNumber(command.telemetry);
      if (period < 0) period = 0;
      this.telemetryEvery = period;
    }
    if (command.mission !== undefined && command.mission !== null) {
      this.mission = asNumber(command.mission);
    }
    if (command.info) {
      out.push(JSON.stringify({ info: true, robot: 'faux-robot', missions: '', camera: false }));
    }

    if (command.stop) {
      this.mission = 0;
      this.stop();
      return out;
    }

    const hasLeft = command.left !== undefined && command.left !== null;
    const hasRight = command.right !== undefined && command.right !== null;
    if (!hasLeft && !hasRight) return out;

    const left = hasLeft ? asNumber(command.left) : 0;
    const right = hasRight ? asNumber(command.right) : 0;
    let duration = command.duration === undefined || command.duration === null
      ? DEFAULT_DURATION_MS
      : asNumber(command.duration);
    if (duration < 0) duration = DEFAULT_DURATION_MS;   // matches Robot::execute()

    this.drive(left, right, duration);
    if (this.verbose) {
      const held = Math.min(duration, MAX_DURATION_MS);
      console.log(`  wheels ${pad(this.left)} ${pad(this.right)}  for ${held} ms`);
    }
    return out;
  }

  /** Enforce safety and return telemetry lines due this update. */
  update(connected) {
    const out = [];

    if (this.moving()) {
      const timeIsUp = !this._hasDeadline || nowMs() >= this._deadline;
      if (timeIsUp || !connected) {
        this.stop();
        if (!connected) this.mission = 0;
        out.push(this.telemetry(true));
      }
    } else {
      this._hasDeadline = false;
    }

    if (this.telemetryEvery && nowMs() - this._lastTelemetry >= this.telemetryEvery) {
      out.push(this.telemetry(false));
    }

    return out;
  }

  telemetry(done = false) {
    this._lastTelemetry = nowMs();

    const message = {
      seq: this.seq,
      ms: nowMs(),
      left: this.left,
      right: this.right,
      moving: this.moving(),
      mission: this.mission,
      ...inventedDistance('front'),
      ...inventedDistance('side'),
    };
    if (done) message.done = true;

    return JSON.stringify(message);
  }
}

/**
 * Match JsonReader's command values: booleans become 1 or 0, and other
 * non-numbers become 0.
 */
function asNumber(value) {
  if (typeof value === 'number') return value;
  if (typeof value === 'boolean') return value ? 1 : 0;
  return 0;
}

function clamp(speed) {
  return Math.max(-MOTOR_MAX_SPEED, Math.min(MOTOR_MAX_SPEED, Math.trunc(speed)));
}

function pad(value) {
  return String(value).padStart(4);
}

/**
 * Simulate a wall at 500 mm with ±4 mm noise and a 2% read failure rate.
 */
function inventedDistance(name) {
  if (Math.random() < 0.02) {
    return { [`${name}_mm`]: -1, [`${name}_ok`]: false, [`${name}_q`]: -1 };
  }
  return {
    [`${name}_mm`]: 500 + Math.floor(Math.random() * 9) - 4,
    [`${name}_ok`]: true,
    [`${name}_q`]: 180 + Math.floor(Math.random() * 41) - 20,
  };
}

// As in robot/Json.h and robot/Radio.h.
const MAX_FIELDS = 12;
const MAX_KEY = 16;
const MAX_LINE = 256;
const LONG_MAX = 2147483647;     // long is 32 bits on the Pico: strtol saturates here
const LONG_MIN = -2147483648;

/**
 * JsonReader::parse() from robot/Json.h, step for step, so that a line the
 * robot refuses is refused here too. Returns an object of whole numbers, or
 * null where the robot answers "that was not valid JSON":
 *
 *     {"left":90.5}          null: the robot refuses fractions
 *     {"stop":"true"}        {stop: 0}: a string is 0, so this does not stop
 *     {"a":{"b":1}}          null: no nested objects or arrays
 *     {"left":1}}            {left: 1}: anything after the first } is ignored
 */
function parseLikeFirmware(text) {
  const fields = Object.create(null);
  let count = 0;
  let p = 0;
  const at = () => text[p];
  const skipSpaces = () => { while (' \t\r\n'.includes(text[p]) && p < text.length) p++; };
  const isDigit = (c) => c !== undefined && c >= '0' && c <= '9';

  skipSpaces();
  if (at() !== '{') return null;
  p++;
  skipSpaces();
  if (at() === '}') return fields;

  for (;;) {
    skipSpaces();
    if (at() !== '"') return null;
    p++;
    const keyStart = p;
    while (p < text.length && at() !== '"') p++;
    if (at() !== '"') return null;
    const key = text.slice(keyStart, p).slice(0, MAX_KEY - 1);
    p++;
    skipSpaces();
    if (at() !== ':') return null;
    p++;
    skipSpaces();

    let store = true;
    let value = 0;
    if (at() === '-' || isDigit(at())) {
      // strtol(p, &end, 10): an optional sign, then at least one digit.
      const match = /^-?[0-9]+/.exec(text.slice(p));
      if (!match) return null;
      value = Math.max(LONG_MIN, Math.min(LONG_MAX, Number(match[0])));
      p += match[0].length;
      if (at() === '.') return null;
    } else if (text.startsWith('true', p)) {
      value = 1; p += 4;
    } else if (text.startsWith('false', p)) {
      value = 0; p += 5;
    } else if (text.startsWith('null', p)) {
      store = false; p += 4;
    } else if (at() === '"') {
      p++;
      while (p < text.length && at() !== '"') { if (at() === '\\' && p + 1 < text.length) p++; p++; }
      if (at() !== '"') return null;
      p++;
    } else {
      return null;
    }

    if (store && (key in fields || count++ < MAX_FIELDS)) fields[key] = value;

    skipSpaces();
    if (at() === ',') { p++; continue; }
    if (at() === '}') return fields;
    return null;
  }
}

/**
 * Parses one line the way the firmware does; returns [command] or [null, error line].
 * The robot keeps MAX_LINE - 1 characters of a line, as WebRadio.h does.
 */
function parseCommand(line) {
  const command = parseLikeFirmware(line.slice(0, MAX_LINE - 1));
  if (command === null) return [null, JSON.stringify({ error: 'that was not valid JSON' })];
  return [command, null];
}

module.exports = { FakeRobot, nowMs, parseCommand, MAX_LINE };
