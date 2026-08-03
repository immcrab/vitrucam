// Real 6DOF pose tracking via the WebXR Device API (ARCore, on Android
// Chrome / Samsung Internet / Edge). Camera+IMU visual-inertial fusion
// gives genuine metric position with no drift, unlike the
// DeviceMotion-based double-integration in sensors.js — which remains
// the automatic fallback on every device/browser without WebXR AR
// support (notably all of iOS Safari; Apple has never shipped the
// WebXR AR module there).
//
// Produces samples in the exact same {x,y,z,qx,qy,qz,qw} shape sensors.js
// does, so app.js can swap the source without touching the send/recenter/
// zoom/smoothing pipeline at all.
(function (global) {
  "use strict";

  async function isSupported() {
    if (!navigator.xr) return false;
    try {
      return await navigator.xr.isSessionSupported("immersive-ar");
    } catch (err) {
      return false;
    }
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

  // WebXR is right-handed Y-up (X right, Y up, Z toward the viewer).
  // Blender is right-handed Z-up (X right, Y forward, Z up). A -90deg
  // rotation about X maps one onto the other — the same convention
  // sensors.js's CORRECTION quaternion establishes for the gyro path, so
  // both sources land in the same Blender-space frame regardless of
  // which one is active.
  const HALF = -Math.PI / 4;
  const S = Math.sin(HALF), C = Math.cos(HALF);
  const CHANGE_OF_BASIS = [S, 0, 0, C]; // -90deg about X, (x,y,z,w)
  const CHANGE_OF_BASIS_INV = [-S, 0, 0, C]; // +90deg about X

  function remapPosition(p) {
    return { x: p.x, y: p.z, z: -p.y };
  }

  function remapOrientation(q) {
    const combined = quatMultiply(quatMultiply(CHANGE_OF_BASIS, [q.x, q.y, q.z, q.w]), CHANGE_OF_BASIS_INV);
    return combined;
  }

  function createXrSession() {
    let session = null;
    let referenceSpace = null;
    let sampleHandler = null;
    let errorHandler = null;
    let endHandler = null;
    let active = false;

    function onXRFrame(time, frame) {
      if (!active || !session) return;
      session.requestAnimationFrame(onXRFrame);

      const pose = frame.getViewerPose(referenceSpace);
      if (!pose || !sampleHandler) return;

      const pos = remapPosition(pose.transform.position);
      const rot = remapOrientation(pose.transform.orientation);

      sampleHandler({
        x: pos.x,
        y: pos.y,
        z: pos.z,
        qx: rot[0],
        qy: rot[1],
        qz: rot[2],
        qw: rot[3],
      });
    }

    async function start({ onSample, onError, onEnd, overlayElement }) {
      sampleHandler = onSample;
      errorHandler = onError;
      endHandler = onEnd;

      const canvas = document.createElement("canvas");
      const gl = canvas.getContext("webgl", { xrCompatible: true });
      if (!gl) {
        throw new Error("WebGL is unavailable, required for WebXR");
      }

      const sessionInit = { optionalFeatures: ["local-floor", "dom-overlay"] };
      if (overlayElement) {
        sessionInit.domOverlay = { root: overlayElement };
      }

      session = await navigator.xr.requestSession("immersive-ar", sessionInit);

      session.addEventListener("end", () => {
        active = false;
        session = null;
        if (endHandler) endHandler();
      });

      await gl.makeXRCompatible();
      session.updateRenderState({ baseLayer: new XRWebGLLayer(session, gl) });

      try {
        referenceSpace = await session.requestReferenceSpace("local-floor");
      } catch (err) {
        referenceSpace = await session.requestReferenceSpace("local");
      }

      active = true;
      session.requestAnimationFrame(onXRFrame);
    }

    function stop() {
      active = false;
      if (session) {
        session.end().catch(() => {});
      }
      session = null;
      sampleHandler = null;
    }

    return {
      start(opts) {
        return start(opts).catch((err) => {
          active = false;
          session = null;
          if (errorHandler) errorHandler(err);
          throw err;
        });
      },
      stop,
      isActive: () => active,
    };
  }

  global.createXrSession = createXrSession;
  global.isWebXrArSupported = isSupported;
})(window);
