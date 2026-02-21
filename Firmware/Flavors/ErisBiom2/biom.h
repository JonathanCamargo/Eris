#ifndef BIOM_H
#define BIOM_H

#include "Eris.h"
#include <eriscommon.h>

#include <Arduino.h>

namespace Biom{

  extern ErisBuffer<BiomSample_t> buffer;
  
  void start(void);
}

#endif
