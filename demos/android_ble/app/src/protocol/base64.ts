/**
 * Base64 <-> bytes. react-native-ble-plx hands characteristic values over as
 * base64 strings; this avoids depending on a global atob/btoa or Buffer polyfill.
 */
const ALPHABET = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/';
const LOOKUP = new Int16Array(128).fill(-1);
for (let i = 0; i < ALPHABET.length; i++) LOOKUP[ALPHABET.charCodeAt(i)] = i;

export function base64ToBytes(b64: string): Uint8Array {
  let len = b64.length;
  while (len > 0 && b64[len - 1] === '=') len--;
  const out = new Uint8Array(Math.floor((len * 3) / 4));
  let bits = 0;
  let acc = 0;
  let o = 0;
  for (let i = 0; i < len; i++) {
    const code = b64.charCodeAt(i);
    const v = code < 128 ? LOOKUP[code] : -1;
    if (v < 0) continue; // skip whitespace / line breaks
    acc = (acc << 6) | v;
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      out[o++] = (acc >> bits) & 0xff;
    }
  }
  return o === out.length ? out : out.slice(0, o);
}

export function bytesToBase64(bytes: Uint8Array): string {
  let s = '';
  let i = 0;
  for (; i + 2 < bytes.length; i += 3) {
    const n = (bytes[i] << 16) | (bytes[i + 1] << 8) | bytes[i + 2];
    s += ALPHABET[(n >> 18) & 63] + ALPHABET[(n >> 12) & 63] + ALPHABET[(n >> 6) & 63] + ALPHABET[n & 63];
  }
  const rest = bytes.length - i;
  if (rest === 1) {
    const n = bytes[i] << 16;
    s += ALPHABET[(n >> 18) & 63] + ALPHABET[(n >> 12) & 63] + '==';
  } else if (rest === 2) {
    const n = (bytes[i] << 16) | (bytes[i + 1] << 8);
    s += ALPHABET[(n >> 18) & 63] + ALPHABET[(n >> 12) & 63] + ALPHABET[(n >> 6) & 63] + '=';
  }
  return s;
}

/** ASCII command line -> bytes. Eris commands are plain ASCII. */
export function asciiBytes(text: string): Uint8Array {
  const out = new Uint8Array(text.length);
  for (let i = 0; i < text.length; i++) out[i] = text.charCodeAt(i) & 0x7f;
  return out;
}
