#include "Eris.h"
#include "serialcommand.h"
#include "imu.h"

namespace SerialCom {

// Static allocation to avoid large stack frames in the ReadSerial thread.
static IMUSample_t imusamples[MEMBUFFERSIZE];

void InitIMU() {
    IMU::InitIMU();
}

void ShowFailures() {
    eriscommon::println(IMU::failures);
}

void TransmitIMU() {
    // IMU [<idx>] — picks buffer 0 or 1; default is buffer 0.
    char *arg = sCmd.next();
    ErisBuffer<IMUSample_t>* imubuffer;
    if (arg != NULL) {
        Serial.print("IMU[");
        Serial.print(arg);
        Serial.print("] ");
        int imuidx = atoi(arg);
        switch (imuidx) {
            case 0:  imubuffer = &IMU::buffer0; break;
            case 1:  imubuffer = &IMU::buffer1; break;
            default: imubuffer = &IMU::buffer0; break;
        }
    } else {
        imubuffer = &IMU::buffer0;
    }

    ERIS_CRITICAL_ENTER();
    int num = imubuffer->FetchData(imusamples, (char*)"IMU", MEMBUFFERSIZE);
    long missed = imubuffer->missed();
    ERIS_CRITICAL_EXIT();

    Serial.print("(missed:");
    Serial.print(missed);
    Serial.print(") ");
    if (num > 0) {
        uint8_t i = 0;
        Serial.print("(@");
        Serial.print(imusamples[0].timestamp, 2);
        Serial.print("ms)");
        for (i = 0; i < num - 1; i++) {
            Serial.print(imusamples[i].ax, 2);
            Serial.print(":");
            Serial.print(imusamples[i].ay, 2);
            Serial.print(":");
            Serial.print(imusamples[i].az, 2);
            Serial.print(",");
        }
        Serial.print("(@");
        Serial.print(imusamples[i].timestamp, 2);
        Serial.println("ms)");
    } else {
        Serial.println();
    }
}

void registerCommands(SerialCommand& sCmd) {
    sCmd.addCommand("INITIMU", InitIMU);
    sCmd.addCommand("IMU",     TransmitIMU);
    sCmd.addCommand("FAIL",    ShowFailures);
}

} // namespace SerialCom
