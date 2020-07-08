#ifndef FASTDUALBUFFER_H
#define FASTDUALBUFFER_H

#define FASTDUALBUFFERSIZE 200
class FastDualBuffer{
  
  private:
    float buf0[FASTDUALBUFFERSIZE];
    float buf1[FASTDUALBUFFERSIZE];
    long idx;
    float *wbuf;//Buffer to write
    float *rbuf;//Buffer to read
    bool lock;
    
  public:
    FastDualBuffer();
    int add(float x);
    float *read();
    void clear();
  
};

#endif
