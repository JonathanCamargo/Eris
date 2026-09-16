#include "Eris.h"
#include "serialcommands.h"
#include "signals.h"

#include <stdlib.h>
#include <string.h>

// Demo commands, one per button in the Android app.
//
// Replies go out as TEXT packets (not Serial.println): over BLE, Serial is the
// unplugged USB port, so only packets reach the phone.

static const char * rgbState = "OFF";

static void reply(const char * text) {
  packet.start(Packet::PacketType::TEXT);
  packet.append((const uint8_t *)text, strlen(text));
  packet.send();
}

// FREQ <hz> -- WAVES frequency, clamped to 0.1..20 Hz.
static void setFreq() {
  char * arg = SerialCom::sCmd.next();
  if (arg != NULL) {
    float f = atof(arg);
    Demo::freqHz = constrain(f, 0.1f, 20.0f);
  }
  String msg = "FREQ " + String((float)Demo::freqHz, 2) + " Hz";
  reply(msg.c_str());
}

// AMP <a> -- WAVES amplitude, clamped to 0..10.
static void setAmp() {
  char * arg = SerialCom::sCmd.next();
  if (arg != NULL) {
    float a = atof(arg);
    Demo::amp = constrain(a, 0.0f, 10.0f);
  }
  String msg = "AMP " + String((float)Demo::amp, 2);
  reply(msg.c_str());
}

// RGB <R|G|B|W|OFF> -- the XIAO's on-board RGB LED.
static void setRgb() {
  char * arg = SerialCom::sCmd.next();
  if (arg == NULL) {
    reply("RGB expects R, G, B, W or OFF");
    return;
  }
  bool r = false, g = false, b = false;
  switch (toupper(arg[0])) {
    case 'R': r = true;                     rgbState = "R";   break;
    case 'G': g = true;                     rgbState = "G";   break;
    case 'B': b = true;                     rgbState = "B";   break;
    case 'W': r = true; g = true; b = true; rgbState = "W";   break;
    default:                                rgbState = "OFF"; break;
  }
#if defined(LED_RED) && defined(LED_GREEN) && defined(LED_BLUE)
  digitalWrite(LED_RED,   r ? RGB_LED_ON : RGB_LED_OFF);
  digitalWrite(LED_GREEN, g ? RGB_LED_ON : RGB_LED_OFF);
  digitalWrite(LED_BLUE,  b ? RGB_LED_ON : RGB_LED_OFF);
#else
  (void)r; (void)g; (void)b;
#endif
  String msg = String("RGB ") + rgbState;
  reply(msg.c_str());
}

// PING -- liveness check; replies with firmware uptime.
static void ping() {
  String msg = "PONG " + String(millis()) + " ms";
  reply(msg.c_str());
}

// STATUS -- current demo settings in one line.
static void status() {
  String msg = "STATUS freq=" + String((float)Demo::freqHz, 2) +
               " amp=" + String((float)Demo::amp, 2) +
               " rgb=" + rgbState +
               " up=" + String(millis() / 1000) + "s";
  reply(msg.c_str());
}

namespace SerialCom {

void registerCommands(SerialCommand & sCmd) {
  sCmd.addCommand("FREQ",   setFreq);
  sCmd.addCommand("AMP",    setAmp);
  sCmd.addCommand("RGB",    setRgb);
  sCmd.addCommand("PING",   ping);
  sCmd.addCommand("STATUS", status);
}

}
