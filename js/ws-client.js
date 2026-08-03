// Thin wrapper around the WebSocket connection to the Blender addon's
// server. One connection at a time, matching the addon's single-session
// design; no client-initiated reconnect loop for the initial pairing
// attempt (a failed handshake almost always means the TLS cert hasn't
// been trusted yet or the code is wrong, so app.js prompts the user
// rather than retrying blindly). Once connected, a lightweight
// reconnect-on-drop is used to ride out brief LAN hiccups.
(function (global) {
  "use strict";

  const RECONNECT_DELAY_MS = 1500;
  const MAX_RECONNECT_ATTEMPTS = 5;

  function createWsClient({ ip, port, tokenHex, onOpen, onClose, onError, onGiveUp }) {
    let socket = null;
    let manualClose = false;
    let reconnectAttempts = 0;
    let reconnectTimer = null;

    const url = `wss://${ip}:${port}/ws?token=${tokenHex}`;

    function connect() {
      manualClose = false;
      socket = new WebSocket(url);

      socket.onopen = () => {
        reconnectAttempts = 0;
        if (onOpen) onOpen();
      };

      socket.onclose = (event) => {
        if (onClose) onClose(event.code, event.reason);
        if (!manualClose && reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
          reconnectAttempts += 1;
          reconnectTimer = setTimeout(connect, RECONNECT_DELAY_MS);
        } else if (!manualClose && onGiveUp) {
          onGiveUp();
        }
      };

      socket.onerror = () => {
        if (onError) onError();
      };
    }

    function send(payload) {
      if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify(payload));
      }
    }

    function close() {
      manualClose = true;
      if (reconnectTimer) {
        clearTimeout(reconnectTimer);
        reconnectTimer = null;
      }
      if (socket) {
        socket.close();
      }
    }

    connect();

    return { send, close };
  }

  global.createWsClient = createWsClient;
})(window);
