#include "Eris.h"
#include "gcode.h"
#include "motion.h"
#include "serialcommands.h"

#include <stdlib.h>
#include <ctype.h>

namespace GCode{

// Report the axis the way a G-code controller answers M114, which is also what
// a bare "G0" prints.
static void reportState(void){
  eriscommon::print("X:");     eriscommon::print((double)Motion::positionMm(), 3);
  eriscommon::print(" TGT:");  eriscommon::print((double)Motion::targetMm(), 3);
  eriscommon::print(" F:");    eriscommon::print((double)Motion::feed(), 1);
  eriscommon::print(" V:");    eriscommon::print((double)Motion::velocityMmS(), 2);
  eriscommon::println(Motion::isMoving() ? " MOVING" : " IDLE");
}

/*
 * Parse one G-code word. `tok` is a whole token from the command line, e.g.
 * "X12.5". A bare letter is accepted too ("X 12.5"), because a person typing
 * into a serial monitor will write it that way even though no sender does.
 *
 * Returns false when the token is not a number, which is the case worth
 * catching: strtod() alone would read "Xfoo" as 0 and cheerfully drive the
 * axis to the origin.
 */
static bool parseWord(char * tok, char & letter, float & value){
  letter = (char)toupper((unsigned char)tok[0]);
  const char * num = tok + 1;

  if (*num == '\0'){
    char * next = SerialCom::sCmd.next();
    if (next == NULL) return false;
    num = next;
  }

  char * end = NULL;
  double v = strtod(num, &end);
  if (end == num || *end != '\0') return false;
  value = (float)v;
  return true;
}

void G0(void){
  bool  haveX = false, haveF = false;
  float x = 0.0f, f = 0.0f;

  char * tok;
  while ((tok = SerialCom::sCmd.next()) != NULL){
    char  letter;
    float value;
    if (!parseWord(tok, letter, value)){
      eriscommon::print("G0: not a number: ");
      eriscommon::println(tok);
      Error::RaiseError(Error::COMMAND, (char *)"G0 malformed word");
      return;                        // parse error moves nothing
    }
    switch (letter){
      case 'X': x = value; haveX = true; break;
      case 'F': f = value; haveF = true; break;
      default:
        // Silently ignoring an unknown word is how an axis ends up somewhere
        // nobody asked for. Say so, and do nothing.
        eriscommon::print("G0: unsupported word: ");
        eriscommon::println(tok);
        Error::RaiseError(Error::COMMAND, (char *)"G0 supports X and F only");
        return;
    }
  }

  // Feedrate first: on a real controller "G0 X10 F600" performs THIS move at
  // the new rate, and the rate sticks for the moves after it.
  if (haveF){
    float adopted = Motion::setFeed(f);
    if (adopted != f){
      eriscommon::print("F clamped to ");
      eriscommon::print((double)adopted, 1);
      eriscommon::println(" mm/min");
    }
  }

  if (haveX){
    if (!Motion::moveTo(x)) return;   // moveTo already reported why
  }

  if (!haveX && !haveF){ reportState(); return; }

  eriscommon::println("ok");
}

} // namespace GCode
