#ifndef LOADCELL_H
#define LOADCELL_H

#include <Arduino.h>
#include "customtypes.h"
#include <sriloadcell.h>

namespace Loadcell{

typedef struct {
  LoadcellDataRaw_t data;
  bool inuse;
}buffer_t;

extern long missed;
extern mailbox_t buffer;

void start(void);
void kill();

void UpdateState(LoadcellDataRaw_t data);

}

#endif
