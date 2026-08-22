#include "Eris.h"
#include "serialcommand.h"
#include "configuration.h"
#include "fsr.h"
#include "joints.h"
#include "loadcell.h"
#include "sync.h"

namespace SerialCom {

// Static allocation to avoid large stack frames in the ReadSerial thread.
static JointStateSample_t kneevalues[MEMBUFFERSIZE];
static JointStateSample_t anklevalues[MEMBUFFERSIZE];
static LoadcellSample_t   loadcellsamples[MEMBUFFERSIZE];

static void printFSRFields(const FSRSample_t& s)        { Serial.print(s.ch[0], 2); }
static void printSyncFields(const uint8_tSample_t& s)   { Serial.print(s.value); }

void TransmitFSR()  { transmitBuffer(FSR::buffer,  "FSR[ch0]", printFSRFields); }
void TransmitSync() { transmitBuffer(Sync::buffer, "Sync",     printSyncFields); }

void TransmitJointState() {
    ERIS_CRITICAL_ENTER();
    int numKnee  = Joints::kneebuffer.FetchData(kneevalues,  (char*)"JOINTS_RAW", MEMBUFFERSIZE);
    int numAnkle = Joints::anklebuffer.FetchData(anklevalues,(char*)"JOINTS_RAW", MEMBUFFERSIZE);
    ERIS_CRITICAL_EXIT();

    int n = (numKnee < numAnkle) ? numKnee : numAnkle;
    eriscommon::println("(Knee)theta,theta_dot,torque,(Ankle)theta,theta_dot,torque");
    for (int i = 0; i < n; i++) {
        eriscommon::print(kneevalues[i].theta, 2);     eriscommon::print(",");
        eriscommon::print(kneevalues[i].theta_dot, 2); eriscommon::print(",");
        eriscommon::print(kneevalues[i].torque, 2);    eriscommon::print(",");
        eriscommon::print(anklevalues[i].theta, 2);    eriscommon::print(",");
        eriscommon::print(anklevalues[i].theta_dot, 2); eriscommon::print(",");
        eriscommon::println(anklevalues[i].torque, 2);
    }
}

void TransmitLoadcellState() {
    ERIS_CRITICAL_ENTER();
    int num = Loadcell::buffer.FetchData(loadcellsamples, (char*)"LOADCELL_RAW", MEMBUFFERSIZE);
    ERIS_CRITICAL_EXIT();
    for (int i = 0; i < num; i++) {
        const LoadcellSample_t* d = &loadcellsamples[i];
        eriscommon::print("forceX:");  eriscommon::print(d->forceX, 2);
        eriscommon::print(", forceY:"); eriscommon::print(d->forceY, 2);
        eriscommon::print(", forceZ:"); eriscommon::print(d->forceZ, 2);
        eriscommon::print(", momentX:"); eriscommon::print(d->momentX, 2);
        eriscommon::print(", momentY:"); eriscommon::print(d->momentY, 2);
        eriscommon::print(", momentZ:"); eriscommon::println(d->momentZ, 2);
    }
}

// IP <kKnee> <bKnee> <theta_eqKnee> <kAnkle> <bAnkle> <theta_eqAnkle>
void SetIP() {
    float kK, bK, teK, kA, bA, teA;
    char* arg;
    arg = sCmd.next(); if (!arg) { eriscommon::println("No arguments"); return; } kK  = atof(arg);
    arg = sCmd.next(); if (!arg) { eriscommon::println("No second argument"); return; } bK  = atof(arg);
    arg = sCmd.next(); if (!arg) { eriscommon::println("No third argument"); return; } teK = atof(arg);
    arg = sCmd.next(); if (!arg) { eriscommon::println("No fourth argument"); return; } kA  = atof(arg);
    arg = sCmd.next(); if (!arg) { eriscommon::println("No fifth argument"); return; } bA  = atof(arg);
    arg = sCmd.next(); if (!arg) { eriscommon::println("No sixth argument"); return; } teA = atof(arg);
    Joints::SetIP(kK, bK, teK, kA, bA, teA);
    eriscommon::print("Updated impedance parameters:");
    eriscommon::print("kKnee=");        eriscommon::print(kK, 2);
    eriscommon::print(", bKnee=");      eriscommon::print(bK, 2);
    eriscommon::print(", theta_eqKnee="); eriscommon::print(teK, 2);
    eriscommon::print(" kAnkle=");      eriscommon::print(kA, 2);
    eriscommon::print(", bAnkle=");     eriscommon::print(bA, 2);
    eriscommon::print(", theta_eqAnkle="); eriscommon::println(teA, 2);
}

// IPA <k> <b> <theta_eq>  (Ankle only)
void SetIPA() {
    float k, b, te;
    char* arg;
    arg = sCmd.next(); if (!arg) { eriscommon::println("No arguments"); return; }      k  = atof(arg);
    arg = sCmd.next(); if (!arg) { eriscommon::println("No second argument"); return; } b  = atof(arg);
    arg = sCmd.next(); if (!arg) { eriscommon::println("No third argument"); return; } te = atof(arg);
    Joints::SetIPA(k, b, te);
    eriscommon::print("Updated impedance parameters:");
    eriscommon::print(" kAnkle=");      eriscommon::print(k, 2);
    eriscommon::print(", bAnkle=");     eriscommon::print(b, 2);
    eriscommon::print(", theta_eqAnkle="); eriscommon::println(te, 2);
}

// IPK <k> <b> <theta_eq>  (Knee only)
void SetIPK() {
    float k, b, te;
    char* arg;
    arg = sCmd.next(); if (!arg) { eriscommon::println("No arguments"); return; }      k  = atof(arg);
    arg = sCmd.next(); if (!arg) { eriscommon::println("No second argument"); return; } b  = atof(arg);
    arg = sCmd.next(); if (!arg) { eriscommon::println("No third argument"); return; } te = atof(arg);
    Joints::SetIPK(k, b, te);
    eriscommon::print("Updated impedance parameters:");
    eriscommon::print(" kKnee=");       eriscommon::print(k, 2);
    eriscommon::print(", bKnee=");      eriscommon::print(b, 2);
    eriscommon::print(", theta_eqKnee="); eriscommon::println(te, 2);
}

void GetIP() {
    float kK = 0, bK = 0, teK = 0, kA = 0, bA = 0, teA = 0;
    Joints::GetIP(kK, bK, teK, kA, bA, teA);
    eriscommon::print("kKnee=");        eriscommon::print(kK, 2);
    eriscommon::print(", bKnee=");      eriscommon::print(bK, 2);
    eriscommon::print(", theta_eqKnee="); eriscommon::print(teK, 2);
    eriscommon::print(" kAnkle=");      eriscommon::print(kA, 2);
    eriscommon::print(", bAnkle=");     eriscommon::print(bA, 2);
    eriscommon::print(", theta_eqAnkle="); eriscommon::println(teA, 2);
}

void registerCommands(SerialCommand& sCmd) {
    sCmd.addCommand("FSR",   TransmitFSR);
    sCmd.addCommand("SYNC",  TransmitSync);
    sCmd.addCommand("JOINT", TransmitJointState);
    sCmd.addCommand("LC",    TransmitLoadcellState);
    sCmd.addCommand("IP",    SetIP);
    sCmd.addCommand("IPA",   SetIPA);
    sCmd.addCommand("IPK",   SetIPK);
    sCmd.addCommand("IP?",   GetIP);
}

} // namespace SerialCom
