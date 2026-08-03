// RFC 4648 base32 decode (no padding required on input). Mirrors the
// encoding Blender's addon (pairing.py) uses for the pairing code.
(function (global) {
  "use strict";

  const ALPHABET = "ABCDEFGHIJKLMNOPQRSTUVWXYZ234567";

  function base32Decode(input) {
    const clean = input.toUpperCase().replace(/[^A-Z2-7]/g, "");
    if (clean.length === 0) {
      throw new Error("Empty pairing code");
    }

    let bits = 0;
    let value = 0;
    const bytes = [];

    for (let i = 0; i < clean.length; i++) {
      const idx = ALPHABET.indexOf(clean[i]);
      if (idx === -1) {
        throw new Error("Invalid character in pairing code");
      }
      value = (value << 5) | idx;
      bits += 5;
      if (bits >= 8) {
        bits -= 8;
        bytes.push((value >>> bits) & 0xff);
      }
    }

    return new Uint8Array(bytes);
  }

  global.base32Decode = base32Decode;
})(window);
