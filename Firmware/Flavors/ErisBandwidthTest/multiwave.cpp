// Multi-channel sine generator used to fan out NUMCHANNELS floats per sample
// for serial-bandwidth stress testing. Lives alongside (not inside) the
// shared SineWave module because its sample type is flavor-specific.

#include "Eris.h"
#include "multiwave.h"

namespace MultiWave {

ErisBuffer<multiSample_t> buffer;

static eris_thread_ref_t generateWave = NULL;
static const float FREQ = 1;

ERIS_THREAD_WA(waGenerateWave_T, 128);
ERIS_THREAD_FUNC(GenerateWave_T) {
    static long idx = 0;
    multiSample_t mSample;
    while (1) {
        idx = (idx >= 10000) ? 0 : idx + 1;
        float timestamp = ((float)(micros() - t0)) / 1.0e3f;
        float value = 3.3f * sin(2 * M_PI * FREQ * ((float)idx) / 1.0e4f);
        mSample.timestamp = timestamp;
        for (uint8_t i = 0; i < NUMCHANNELS; i++) {
            mSample.value[i] = value * (1 + i / 10);
        }
        buffer.append(mSample);
        eris_sleep_ms(10);
    }
}

void start(void) {
    buffer.init();
    generateWave = eris_thread_create(waGenerateWave_T, 128, ERIS_NORMAL_PRIORITY+1, GenerateWave_T, NULL);
}

}
