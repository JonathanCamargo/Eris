#ifndef SERIALCOMMANDS_H
#define SERIALCOMMANDS_H

#include <modules/serialcom.h>

/*
 * Flavor-specific commands. The universal set (INFO, S_F, S_ON, S_MODE, DESC,
 * ...) comes from eriscommon; registerCommands() in serialcommand.cpp adds:
 *
 *   EMG    dump the buffered EMG samples as text
 *
 * The file is serialcommandS.h, plural, on purpose: on Windows a local
 * "serialcommand.h" shadows the SerialCommand library header (lessons_learned
 * section 5).
 */
namespace SerialCom {
    void TransmitEMG();
}

#endif
