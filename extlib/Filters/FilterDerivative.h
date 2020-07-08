#ifndef FilterDerivative_h
#define FilterDerivative_h

// returns the derivative
struct FilterDerivative {
  long LastUS=0;
  float LastInput=0;
  
  float Derivative;
  
  float input( float inVal );
  
  float output();
};

void testFilterDerivative();

#endif
