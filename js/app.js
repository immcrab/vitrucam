(function () {
  "use strict";

  const els = {
    pairingScreen: document.getElementById("screen-pairing"),
    controlScreen: document.getElementById("screen-control"),
    codeInput: document.getElementById("code-input"),
    connectBtn: document.getElementById("connect-btn"),
    statusLine: document.getElementById("status-line"),
    trustPanel: document.getElementById("trust-panel"),
    trustLink: document.getElementById("trust-link"),
    downloadAddonBtn: document.getElementById("download-addon-btn"),
    installInstructions: document.getElementById("install-instructions"),
    connDot: document.getElementById("conn-dot"),
    connLabel: document.getElementById("conn-label"),
    disconnectBtn: document.getElementById("disconnect-btn"),
    motionGate: document.getElementById("motion-gate"),
    enableMotionBtn: document.getElementById("enable-motion-btn"),
    controlPanels: document.getElementById("control-panels"),
    recenterBtn: document.getElementById("recenter-btn"),
    zoomSlider: document.getElementById("zoom-slider"),
    zoomValue: document.getElementById("zoom-value"),
    smoothingSlider: document.getElementById("smoothing-slider"),
    smoothingValue: document.getElementById("smoothing-value"),
    controlError: document.getElementById("control-error"),
    cameraPreview: document.getElementById("camera-preview"),
    viewfinderPlaceholder: document.getElementById("viewfinder-placeholder"),
    arCard: document.getElementById("ar-card"),
    arToggleBtn: document.getElementById("ar-toggle-btn"),
  };

  let wsClient = null;
  let sensors = null;
  let xrSession = null;
  let pendingRecenter = false;
  let connectTrustTimer = null;
  let previousPreviewUrl = null;

  function onPreviewFrame(blob) {
    const url = URL.createObjectURL(blob);
    els.cameraPreview.src = url;
    els.cameraPreview.hidden = false;
    els.viewfinderPlaceholder.hidden = true;
    if (previousPreviewUrl) {
      URL.revokeObjectURL(previousPreviewUrl);
    }
    previousPreviewUrl = url;
  }

  function showScreen(name) {
    els.pairingScreen.dataset.active = String(name === "pairing");
    els.controlScreen.dataset.active = String(name === "control");
  }

  function setStatus(text, isError) {
    els.statusLine.textContent = text || "";
    els.statusLine.classList.toggle("status-line-error", Boolean(isError));
  }

  function setConnLabel(text, warn) {
    els.connLabel.textContent = text;
    els.connDot.classList.toggle("dot-warn", Boolean(warn));
  }

  function formatCodeInput() {
    const raw = els.codeInput.value.toUpperCase().replace(/[^A-Z2-7]/g, "");
    const groups = raw.match(/.{1,4}/g) || [];
    els.codeInput.value = groups.slice(0, 4).join("-");
  }

  function currentZoom() {
    return Number(els.zoomSlider.value) / 100;
  }

  function currentSmoothing() {
    return Number(els.smoothingSlider.value) / 100;
  }

  function onSensorSample(sample) {
    const payload = {
      ...sample,
      t: Date.now(),
      zoom: currentZoom(),
      smoothing: currentSmoothing(),
    };
    if (pendingRecenter) {
      payload.recenter = true;
      pendingRecenter = false;
    }
    wsClient.send(payload);
  }

  async function beginMotionCapture() {
    const granted = await sensors.requestPermissionIfNeeded();
    if (!granted) {
      els.controlError.hidden = false;
      els.controlError.textContent =
        "Motion permission denied. Enable it in Settings → Safari → Motion & Orientation Access, then reload.";
      return;
    }
    els.motionGate.hidden = true;
    els.controlPanels.hidden = false;
    els.controlError.hidden = true;
    sensors.start(onSensorSample);
  }

  function needsExplicitMotionGesture() {
    return (
      (typeof DeviceOrientationEvent !== "undefined" &&
        typeof DeviceOrientationEvent.requestPermission === "function") ||
      (typeof DeviceMotionEvent !== "undefined" &&
        typeof DeviceMotionEvent.requestPermission === "function")
    );
  }

  async function offerArTrackingIfSupported() {
    if (typeof isWebXrArSupported !== "function") return;
    const supported = await isWebXrArSupported();
    if (supported) {
      els.arCard.hidden = false;
    }
  }

  async function startArTracking() {
    els.arToggleBtn.disabled = true;
    try {
      if (sensors) sensors.stop();
      xrSession = createXrSession();
      await xrSession.start({
        onSample: onSensorSample,
        overlayElement: els.controlScreen,
        onEnd: () => {
          xrSession = null;
          els.arToggleBtn.textContent = "Start AR Tracking";
          els.arToggleBtn.disabled = false;
          if (sensors) sensors.start(onSensorSample);
        },
        onError: () => {
          xrSession = null;
          els.controlError.hidden = false;
          els.controlError.textContent = "Couldn't start AR tracking on this device. Falling back to motion tracking.";
          if (sensors) sensors.start(onSensorSample);
        },
      });
      els.arToggleBtn.textContent = "Stop AR Tracking";
      els.arToggleBtn.disabled = false;
      pendingRecenter = true;
    } catch (err) {
      els.arToggleBtn.disabled = false;
    }
  }

  function stopArTracking() {
    if (xrSession) {
      xrSession.stop();
      xrSession = null;
    }
    els.arToggleBtn.textContent = "Start AR Tracking";
    if (sensors) sensors.start(onSensorSample);
    pendingRecenter = true;
  }

  function enterControlScreen(ip, port, tokenHex) {
    showScreen("control");
    setConnLabel("Streaming", false);
    els.controlError.hidden = true;
    els.arCard.hidden = true;
    els.arToggleBtn.textContent = "Start AR Tracking";

    sensors = createSensorSession();

    if (needsExplicitMotionGesture()) {
      els.motionGate.hidden = false;
      els.controlPanels.hidden = true;
    } else {
      els.motionGate.hidden = true;
      els.controlPanels.hidden = false;
      sensors.start(onSensorSample);
    }

    offerArTrackingIfSupported();
  }

  function teardownSession() {
    if (xrSession) {
      xrSession.stop();
      xrSession = null;
    }
    if (sensors) {
      sensors.stop();
      sensors = null;
    }
    if (wsClient) {
      wsClient.close();
      wsClient = null;
    }
    if (connectTrustTimer) {
      clearTimeout(connectTrustTimer);
      connectTrustTimer = null;
    }
    if (previousPreviewUrl) {
      URL.revokeObjectURL(previousPreviewUrl);
      previousPreviewUrl = null;
    }
    els.cameraPreview.hidden = true;
    els.cameraPreview.removeAttribute("src");
    els.viewfinderPlaceholder.hidden = false;
  }

  function attemptConnect() {
    let parsed;
    try {
      parsed = decodePairingCode(els.codeInput.value);
    } catch (err) {
      setStatus(err.message || "Invalid pairing code.", true);
      return;
    }

    const { ip, port, tokenHex } = parsed;

    // A prior failed attempt may still have a socket alive with its own
    // reconnect loop running (ws-client.js retries on unexpected close).
    // Leaving it running would let it race this new attempt for the
    // addon's single-session slot, or fire a stray onOpen/trust-panel
    // callback after this attempt has already resolved.
    if (wsClient) {
      wsClient.close();
      wsClient = null;
    }
    if (connectTrustTimer) {
      clearTimeout(connectTrustTimer);
      connectTrustTimer = null;
    }

    els.connectBtn.disabled = true;
    setStatus("Connecting…", false);
    els.trustLink.href = `https://${ip}:${port}/pair`;

    // A failed handshake (bad code, or an untrusted TLS cert) is
    // indistinguishable from JS's perspective — browsers don't expose the
    // HTTP status of a rejected WebSocket upgrade. Surface the trust-cert
    // path proactively rather than guessing at a more specific reason.
    connectTrustTimer = setTimeout(() => {
      els.trustPanel.hidden = false;
    }, 3500);

    wsClient = createWsClient({
      ip,
      port,
      tokenHex,
      onOpen: () => {
        clearTimeout(connectTrustTimer);
        els.trustPanel.hidden = true;
        els.connectBtn.disabled = false;
        setStatus("", false);
        enterControlScreen(ip, port, tokenHex);
      },
      onClose: () => {
        if (els.controlScreen.dataset.active === "true") {
          setConnLabel("Reconnecting…", true);
        } else {
          els.connectBtn.disabled = false;
          setStatus("Couldn't connect. Check the code, or trust the connection below.", true);
          els.trustPanel.hidden = false;
        }
      },
      onGiveUp: () => {
        setConnLabel("Disconnected", true);
        els.controlError.hidden = false;
        els.controlError.textContent = "Lost connection to Blender. Reconnect from the pairing screen.";
      },
      onPreviewFrame,
    });
  }

  els.downloadAddonBtn.addEventListener("click", () => {
    els.installInstructions.hidden = false;
  });

  els.codeInput.addEventListener("input", formatCodeInput);

  els.connectBtn.addEventListener("click", attemptConnect);

  els.enableMotionBtn.addEventListener("click", beginMotionCapture);

  els.arToggleBtn.addEventListener("click", () => {
    if (xrSession) {
      stopArTracking();
    } else {
      startArTracking();
    }
  });

  els.recenterBtn.addEventListener("click", () => {
    if (sensors) sensors.recenter();
    pendingRecenter = true;
  });

  els.zoomSlider.addEventListener("input", () => {
    els.zoomValue.textContent = `${els.zoomSlider.value}%`;
  });

  els.smoothingSlider.addEventListener("input", () => {
    els.smoothingValue.textContent = `${els.smoothingSlider.value}%`;
  });

  els.disconnectBtn.addEventListener("click", () => {
    teardownSession();
    showScreen("pairing");
    setStatus("", false);
  });
})();
