/**
 * Missions run on the phone, using polled telemetry from the robot.
 *
 * Each entry appears under "Missions sur le téléphone" and takes the page's
 * { speed, duration, distance } settings. Try edits with simrobot.js at
 * http://localhost:8080/; reflash firmware to try them on the real robot.
 */

import { Mission } from './robot.js';

export const missions = {
  /** One step: forward for `duration` ms. */
  avancer: ({ speed = 180, duration = 1000 } = {}) =>
    new Mission('avancer')
      .forward(speed, duration),

  /** Three steps: forward, stand still, come back. */
  'aller-retour': ({ speed = 180, duration = 1000 } = {}) =>
    new Mission('aller-retour')
      .forward(speed, duration)
      .pause(300)
      .backward(speed, duration),
};
