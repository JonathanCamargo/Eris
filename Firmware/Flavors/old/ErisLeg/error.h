#ifndef ERROR_H
#define ERROR_H

#include <Arduino.h>

typedef enum errorCodes {
  BUFFER,  // Buffers where not read on time,missing data
  ENCODERS,  // Encoders disconnected, inconsistent value read
  CANBUS,  // Can communication errors
  LOWBAT, // Low battery on the device
  MEMORY,  // Memory problems
  COMMAND // Unsupported command
} ErrorCode_t;

namespace Error{

void start(void);
void RaiseError(ErrorCode_t errorCode);
void RaiseError(ErrorCode_t errorCode, char * text);


// Error handlers:
void BufferErrorHandler(void);

void EncodersErrorHandler(void);

void LowBatErrorHandler(void);


}

#endif
