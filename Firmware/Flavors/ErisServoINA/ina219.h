#ifndef INA219_H
#define INA219_H

#include "Eris.h"

namespace INA219{

extern ErisBuffer<inaSample_t> buffer;

void start(void);

}

#endif
