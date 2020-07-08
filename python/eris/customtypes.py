#  Custom type definitions for data

from construct import Struct,Float32l,Int8ub

NUMEMGCHANNELS=8
EMGSample_t = Struct(
    "timestamp" / Float32l,
    "ch" / Float32l[NUMEMGCHANNELS]
    )

EMG2CHSample_t = Struct(
    "timestamp" / Float32l,
    "ch" / Float32l[2]
    )

EMG3CHSample_t = Struct(
    "timestamp" / Float32l,
    "ch" / Float32l[3]
    )

EMG4CHSample_t = Struct(
    "timestamp" / Float32l,
    "ch" / Float32l[4]
    )

EMG8CHSample_t = Struct(
    "timestamp" / Float32l,
    "ch" / Float32l[8]
    )

floatSample_t = Struct(
    "timestamp" / Float32l,
    "value" / Float32l
    )

NUMFSRCHANNELS=1
FSRSample_t = Struct(
    "timestamp" / Float32l,
    "ch" / Float32l[NUMFSRCHANNELS]
    )

FSR2CHSample_t = Struct(
    "timestamp" / Float32l,
    "ch" / Float32l[2]
    )

IMUSample_t = Struct(
    "timestamp" / Float32l,
    "ax" / Float32l,
    "ay" / Float32l,
    "az" / Float32l,
    "wx" / Float32l,
    "wy" / Float32l,
    "wz" / Float32l
    )

JointStateSample_t = Struct(
    "timestamp" / Float32l,
    "theta" / Float32l,
    "theta_dot" / Float32l,
    "torque" / Float32l
    )

LoadcellSample_t = Struct(
    "timestamp" / Float32l,
    "forceX" / Float32l,
    "forceY" / Float32l,
    "forceZ" / Float32l,
    "momentX" / Float32l,
    "momentY" / Float32l,
    "momentZ" / Float32l,
   )

uint8_tSample_t = Struct(
    "timestamp" / Float32l,
    "value" / Int8ub
    )


