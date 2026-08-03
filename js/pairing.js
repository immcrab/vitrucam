// Decodes a pairing code (typed on the phone) into the connection info
// the Blender addon's server needs: {ip, port, tokenHex}. Mirrors
// pairing.py's decode_code() on the addon side byte-for-byte.
(function (global) {
  "use strict";

  function bytesToHex(bytes) {
    return Array.from(bytes)
      .map((b) => b.toString(16).padStart(2, "0"))
      .join("");
  }

  function decodePairingCode(code) {
    const raw = base32Decode(code.replace(/-/g, "").replace(/\s+/g, ""));

    if (raw.length !== 10) {
      throw new Error("Pairing code looks incomplete or mistyped.");
    }

    const ip = Array.from(raw.slice(0, 4)).join(".");
    const port = (raw[4] << 8) | raw[5];
    const tokenHex = bytesToHex(raw.slice(6, 10));

    if (port < 1 || port > 65535) {
      throw new Error("Pairing code looks invalid.");
    }

    return { ip, port, tokenHex };
  }

  global.decodePairingCode = decodePairingCode;
})(window);
