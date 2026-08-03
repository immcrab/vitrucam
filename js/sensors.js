// Turns raw DeviceOrientation/DeviceMotion events into Blender-space
// {x,y,z,qx,qy,qz,qw} samples.
//
// Rotation: alpha/beta/gamma (the W3C "Z-X'-Y''" Tait-Bryan angles) are
// converted to a quaternion using the standard device-orientation formula,
// then rotated by a fixed offset so the phone's "out the back" direction
// (the way you'd naturally aim it like a camera) maps to Blender's
// camera-forward (-Z) with the phone's top edge mapping to camera-up (+Y).
// There's no universally "correct" mapping for this — every phone-as-
// camera tool picks a convention — so the Recenter control exists to let
// the operator zero out any residual yaw/tilt offset by holding the
// phone in their intended "forward" pose and tapping it.
//
// Position: DeviceMotion's linear acceleration (gravity already removed
// by the browser when available) is double-integrated with velocity
// damping to bound drift. This is inherently approximate — there is no
// drift-free position from phone-only accelerometer data — so Recenter
// also zeroes the integrated position back to the origin.
(function (global) {
  "use strict";

  const DEG2RAD = Math.PI / 180;
  const VELOCITY_DAMPING = 0.90; // per-sample decay to bound accel drift
  const ACCEL_DEADZONE = 0.08; // m/s^2, ignore sensor noise below this
  const ROTATION_SMOOTH_T = 0.45; // per-sample nlerp weight toward the new raw reading
  const ROTATION_JUMP_REJECT_DOT = 0.3; // below this dot product, treat as a sensor glitch and drop the sample

  // Fixed corrective rotation: -90 degrees about the device X axis,
  // mapping the device's "world" frame onto Blender's camera convention.
  const CORRECTION = quatFromAxisAngle([1, 0, 0], -Math.PI / 2);

  function quatFromAxisAngle(axis, angle) {
    const half = angle / 2;
    const s = Math.sin(half);
    return [axis[0] * s, axis[1] * s, axis[2] * s, Math.cos(half)];
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

  function orientationToQuaternion(alpha, beta, gamma) {
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

    return quatMultiply([qx, qy, qz, w], CORRECTION);
  }

  function createSensorSession() {
    let position = { x: 0, y: 0, z: 0 };
    let velocity = { x: 0, y: 0, z: 0 };
    let lastMotionTime = null;
    let sampleHandler = null;
    let orientationListener = null;
    let motionListener = null;
    let latestQuat = [0, 0, 0, 1]; // always defined, so emitSample() never crashes before the first reading
    let rawQuatForSmoothing = null; // null until the first real orientation event

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

    function handleOrientation(event) {
      const raw = orientationToQuaternion(event.alpha, event.beta, event.gamma);

      if (rawQuatForSmoothing) {
        const dot = Math.abs(quatDot(rawQuatForSmoothing, raw));
        if (dot < ROTATION_JUMP_REJECT_DOT) {
          // Sudden large jump (compass glitch, gimbal-lock discontinuity
          // in the alpha/beta/gamma formula) — drop this one reading
          // rather than snapping the camera to it.
          return;
        }
        latestQuat = nlerpQuat(latestQuat, raw, ROTATION_SMOOTH_T);
      } else {
        latestQuat = raw;
      }

      rawQuatForSmoothing = raw;
      emitSample();
    }

    function handleMotion(event) {
      const accel = event.acceleration || event.accelerationIncludingGravity;
      const now = event.timeStamp || performance.now();

      if (!accel || lastMotionTime === null) {
        lastMotionTime = now;
        return;
      }

      const dt = Math.min((now - lastMotionTime) / 1000, 0.1);
      lastMotionTime = now;
      if (dt <= 0) return;

      ["x", "y", "z"].forEach((axis) => {
        let a = accel[axis] || 0;
        if (Math.abs(a) < ACCEL_DEADZONE) a = 0;
        velocity[axis] = (velocity[axis] + a * dt) * VELOCITY_DAMPING;
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
