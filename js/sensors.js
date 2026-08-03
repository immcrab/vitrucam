// Turns raw DeviceOrientation/DeviceMotion events into Blender-space
// {x,y,z,qx,qy,qz,qw} samples.
//
// Rotation: driven primarily by DeviceMotion's rotationRate (gyroscope,
// deg/s about the device's own body axes), integrated into a running
// quaternion each sample. This is deliberately NOT recomputed from
// alpha/beta/gamma every event: that Tait-Bryan decomposition has a real
// gimbal-lock singularity at beta = +-90 degrees, which is exactly the
// pose of "phone held upright like a camera" — the primary use case here.
// Near that pose, alpha (compass yaw) and gamma (roll) become numerically
// coupled, so continuously re-deriving the quaternion from them makes a
// physical yaw motion show up partly/wholly as roll (and vice versa).
// Gyro integration has no such singularity. alpha/beta/gamma are still
// used to bootstrap the very first reading (before any gyro data has
// arrived) and, once gyro is flowing, to apply a very light drift
// correction — same idea as any complementary filter, just heavily
// weighted toward the gyro so its own singularity-adjacent noise can't
// dominate. If a browser exposes no rotationRate at all, orientation
// events drive rotation directly as before (still singularity-prone, but
// strictly better than no rotation).
//
// Both the gyro-integrated and orientation-derived quaternions are sent
// as-is: the device's flat-frame axes (X right, Y toward the top edge, Z
// out of the screen toward the viewer) already match Blender's
// camera-local convention once "out the back" (-Z device) is read as
// camera-forward and "top edge" (+Y device) as camera-up, so no extra
// correction quaternion is needed. There's no universally "correct"
// absolute mapping though — every phone-as-camera tool picks a
// convention — so the Recenter control exists to let the operator zero
// out any residual yaw/tilt offset by holding the phone in their
// intended "forward" pose and tapping it.
//
// Position: DeviceMotion's linear acceleration (gravity already removed
// by the browser when available) is double-integrated with velocity
// damping to bound drift. Damping is applied as a per-second decay (not
// a flat per-sample multiplier) so it doesn't eat a normal hand movement
// before it has a chance to accumulate into visible position change.
// This is inherently approximate — there is no drift-free position from
// phone-only accelerometer data — so Recenter also zeroes the integrated
// position back to the origin.
(function (global) {
  "use strict";

  const DEG2RAD = Math.PI / 180;
  const VELOCITY_DAMPING_PER_SECOND = 0.35; // fraction of velocity kept after a full second of no further accel
  const ACCEL_DEADZONE = 0.08; // m/s^2, ignore sensor noise below this
  const ROTATION_SMOOTH_T = 0.45; // per-sample nlerp weight toward raw orientation, used only when no gyro is available
  const ROTATION_DRIFT_CORRECTION_T = 0.02; // per-sample nlerp weight pulling gyro-integrated rotation toward the raw absolute reading
  const ROTATION_JUMP_REJECT_DOT = 0.3; // below this dot product, treat as a sensor glitch and drop the sample
  const GRAVITY_LPF = 0.95; // slow-adapting gravity estimate, used when the browser only gives gravity-included accel

  function hasNumericFields(v) {
    return !!v && typeof v.x === "number" && typeof v.y === "number" && typeof v.z === "number";
  }

  function quatMultiply(a, b) {
    const [ax, ay, az, aw] = a;
    const [bx, by, bz, bw] = b;
    return [
      aw * bx + ax * bw + ay * bz - az * by,
      aw * by - ax * bz + ay * bw + az * bx,
      aw * bz + ax * by - ay * bx + az * bw,
      aw * bw - ax * bx - ay * by - az * bz,
    ];
  }

  function quatDot(a, b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  }

  // Normalized-lerp toward b, taking the shorter arc of the quaternion's
  // double cover (same "q and -q are the same rotation" issue that needs
  // handling here as on the Blender side) — cheap per-event smoothing to
  // knock down raw sensor jitter before it's even transmitted.
  function nlerpQuat(a, b, t) {
    let [bx, by, bz, bw] = b;
    if (quatDot(a, b) < 0) {
      bx = -bx; by = -by; bz = -bz; bw = -bw;
    }
    const rx = a[0] + (bx - a[0]) * t;
    const ry = a[1] + (by - a[1]) * t;
    const rz = a[2] + (bz - a[2]) * t;
    const rw = a[3] + (bw - a[3]) * t;
    const len = Math.hypot(rx, ry, rz, rw) || 1;
    return [rx / len, ry / len, rz / len, rw / len];
  }

  // Raw device-world orientation from alpha/beta/gamma.
  function orientationToWorldQuaternion(alpha, beta, gamma) {
    const z = (alpha || 0) * DEG2RAD;
    const x = (beta || 0) * DEG2RAD;
    const y = (gamma || 0) * DEG2RAD;

    const cX = Math.cos(x / 2), sX = Math.sin(x / 2);
    const cY = Math.cos(y / 2), sY = Math.sin(y / 2);
    const cZ = Math.cos(z / 2), sZ = Math.sin(z / 2);

    const w = cX * cY * cZ - sX * sY * sZ;
    const qx = sX * cY * cZ - cX * sY * sZ;
    const qy = cX * sY * cZ + sX * cY * sZ;
    const qz = cX * cY * sZ + sX * sY * cZ;

    return [qx, qy, qz, w];
  }

  function quatNormalize(a) {
    const [x, y, z, w] = a;
    const len = Math.hypot(x, y, z, w) || 1;
    return [x / len, y / len, z / len, w / len];
  }

  // Small-angle body-frame rotation from an angular velocity vector
  // (rad/s) integrated over dt seconds — the standard quaternion
  // kinematics update, with no Euler-angle decomposition anywhere in it,
  // so it carries no gimbal-lock singularity.
  function bodyDeltaQuaternion(wx, wy, wz, dt) {
    const half = dt / 2;
    return quatNormalize([wx * half, wy * half, wz * half, 1]);
  }

  function createSensorSession() {
    let position = { x: 0, y: 0, z: 0 };
    let velocity = { x: 0, y: 0, z: 0 };
    let lastMotionTime = null;
    let lastGyroTime = null;
    let sampleHandler = null;
    let orientationListener = null;
    let motionListener = null;
    let worldQuat = null; // device-world orientation; null until the first orientation event seeds it
    let latestQuat = [0, 0, 0, 1]; // same as worldQuat once seeded; always defined so emitSample() never crashes before the first reading
    let hasGyro = false; // true once a devicemotion event has delivered a usable rotationRate
    let gravityEstimate = null; // {x,y,z}, only used when accelerationIncludingGravity is the only field available

    function emitSample() {
      if (!sampleHandler) return;
      sampleHandler({
        x: position.x,
        y: position.y,
        z: position.z,
        qx: latestQuat[0],
        qy: latestQuat[1],
        qz: latestQuat[2],
        qw: latestQuat[3],
      });
    }

    function applyWorldQuat(q) {
      // The device's own flat-frame axes (X right, Y toward the top edge,
      // Z out of the screen toward the viewer) already line up exactly
      // with Blender's camera-local convention (X right, Y up, -Z
      // forward) once "out the back" (-Z device) is read as
      // camera-forward and "top edge" (+Y device) as camera-up — so no
      // extra local-frame correction quaternion is needed here. (An
      // earlier -90-about-X "CORRECTION" step used to live here; it
      // silently swapped yaw and roll, which is what made turning the
      // phone left/right show up as tilting up/down.)
      worldQuat = quatNormalize(q);
      latestQuat = worldQuat;
    }

    function handleOrientation(event) {
      if (typeof event.alpha !== "number" || typeof event.beta !== "number" || typeof event.gamma !== "number") {
        // Incomplete reading — common for the first event or two before
        // the sensor finishes calibrating. Blender uses the first sample
        // it ever receives as the "zero" reference pose, so sending a
        // null/zero reading here would make that reference garbage and
        // the camera would jump away from its starting pose almost
        // immediately once real readings arrive.
        return;
      }

      const raw = orientationToWorldQuaternion(event.alpha, event.beta, event.gamma);

      if (worldQuat === null) {
        // Bootstrap: nothing to integrate from yet, so this one absolute
        // reading (taken as-is, singularity risk and all) becomes the
        // starting point. A single bad sample here is far less harmful
        // than continuously re-deriving orientation from this formula.
        applyWorldQuat(raw);
        emitSample();
        return;
      }

      if (!hasGyro) {
        // No rotationRate available on this device/browser: fall back to
        // driving rotation directly from orientation events, same as
        // before (still gimbal-lock-prone near beta=90, but better than
        // no rotation at all).
        const dot = Math.abs(quatDot(worldQuat, raw));
        if (dot < ROTATION_JUMP_REJECT_DOT) {
          // Sudden large jump (compass glitch, gimbal-lock discontinuity
          // in the alpha/beta/gamma formula) — drop this one reading
          // rather than snapping the camera to it.
          return;
        }
        applyWorldQuat(nlerpQuat(worldQuat, raw, ROTATION_SMOOTH_T));
        emitSample();
        return;
      }

      // Gyro is driving rotation continuously (see handleMotion); this is
      // just a gentle long-term drift correction, so a low weight keeps
      // any single glitchy/singularity-adjacent reading from doing
      // visible damage — it gets averaged out over many samples instead.
      applyWorldQuat(nlerpQuat(worldQuat, raw, ROTATION_DRIFT_CORRECTION_T));
      // No emitSample() here: the gyro integration step already emits
      // every sample at its own (typically higher) rate.
    }

    function integrateGyro(rotationRate, dt) {
      if (worldQuat === null || dt <= 0) return;
      hasGyro = true;

      // DeviceRotationRate's alpha/beta/gamma are angular velocity about
      // the device's own Z/X/Y axes respectively (deg/s) — same axis
      // labeling as the orientation event, but body-frame rates, so this
      // integration has no Euler-angle decomposition and no singularity.
      const wx = (rotationRate.beta || 0) * DEG2RAD;
      const wy = (rotationRate.gamma || 0) * DEG2RAD;
      const wz = (rotationRate.alpha || 0) * DEG2RAD;

      const delta = bodyDeltaQuaternion(wx, wy, wz, dt);
      applyWorldQuat(quatMultiply(worldQuat, delta));
      emitSample();
    }

    function handleMotion(event) {
      const now = event.timeStamp || performance.now();

      if (event.rotationRate && typeof event.rotationRate.alpha === "number") {
        if (lastGyroTime === null) {
          lastGyroTime = now;
        } else {
          const gdt = Math.min((now - lastGyroTime) / 1000, 0.1);
          lastGyroTime = now;
          integrateGyro(event.rotationRate, gdt);
        }
      }

      let ax, ay, az;

      if (hasNumericFields(event.acceleration)) {
        // Best case: the browser already removed gravity for us.
        ax = event.acceleration.x;
        ay = event.acceleration.y;
        az = event.acceleration.z;
      } else if (hasNumericFields(event.accelerationIncludingGravity)) {
        // Common case (most Android/Chrome devices): only the
        // gravity-included reading is populated — event.acceleration
        // exists as an object but with null x/y/z, which doesn't fall
        // through a plain `||` fallback. Estimate gravity ourselves as a
        // slow low-pass of this signal and subtract it.
        const g = event.accelerationIncludingGravity;
        if (!gravityEstimate) {
          gravityEstimate = { x: g.x, y: g.y, z: g.z };
        } else {
          gravityEstimate.x = gravityEstimate.x * GRAVITY_LPF + g.x * (1 - GRAVITY_LPF);
          gravityEstimate.y = gravityEstimate.y * GRAVITY_LPF + g.y * (1 - GRAVITY_LPF);
          gravityEstimate.z = gravityEstimate.z * GRAVITY_LPF + g.z * (1 - GRAVITY_LPF);
        }
        ax = g.x - gravityEstimate.x;
        ay = g.y - gravityEstimate.y;
        az = g.z - gravityEstimate.z;
      } else {
        // Neither field is usable on this device/browser at all.
        lastMotionTime = now;
        return;
      }

      if (lastMotionTime === null) {
        lastMotionTime = now;
        return;
      }

      const dt = Math.min((now - lastMotionTime) / 1000, 0.1);
      lastMotionTime = now;
      if (dt <= 0) return;

      // Per-second decay (not a flat per-sample multiplier) so the decay
      // rate doesn't depend on how often devicemotion happens to fire —
      // at a typical ~60Hz, a flat 0.90-per-sample factor would have cut
      // velocity to near zero within a couple of tens of milliseconds,
      // erasing a normal accelerate-then-decelerate hand movement before
      // it could integrate into any visible position change at all.
      const damping = Math.pow(VELOCITY_DAMPING_PER_SECOND, dt);
      const accel = { x: ax, y: ay, z: az };
      ["x", "y", "z"].forEach((axis) => {
        let a = accel[axis];
        if (Math.abs(a) < ACCEL_DEADZONE) a = 0;
        velocity[axis] = (velocity[axis] + a * dt) * damping;
        position[axis] += velocity[axis] * dt;
      });

      emitSample();
    }

    async function requestPermissionIfNeeded() {
      const needsOrientationPermission =
        typeof DeviceOrientationEvent !== "undefined" &&
        typeof DeviceOrientationEvent.requestPermission === "function";
      const needsMotionPermission =
        typeof DeviceMotionEvent !== "undefined" &&
        typeof DeviceMotionEvent.requestPermission === "function";

      if (!needsOrientationPermission && !needsMotionPermission) {
        return true;
      }

      try {
        const results = await Promise.all([
          needsOrientationPermission ? DeviceOrientationEvent.requestPermission() : "granted",
          needsMotionPermission ? DeviceMotionEvent.requestPermission() : "granted",
        ]);
        return results.every((r) => r === "granted");
      } catch (err) {
        return false;
      }
    }

    function start(onSample) {
      sampleHandler = onSample;
      orientationListener = handleOrientation;
      motionListener = handleMotion;
      lastMotionTime = null;
      lastGyroTime = null;
      window.addEventListener("deviceorientation", orientationListener, true);
      window.addEventListener("devicemotion", motionListener, true);
    }

    function stop() {
      if (orientationListener) {
        window.removeEventListener("deviceorientation", orientationListener, true);
      }
      if (motionListener) {
        window.removeEventListener("devicemotion", motionListener, true);
      }
      sampleHandler = null;
    }

    function recenter() {
      position = { x: 0, y: 0, z: 0 };
      velocity = { x: 0, y: 0, z: 0 };
    }

    return { requestPermissionIfNeeded, start, stop, recenter };
  }

  global.createSensorSession = createSensorSession;
})(window);
