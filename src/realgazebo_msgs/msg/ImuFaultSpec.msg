# Full IMU fault scenario for one vehicle.
#
# This message is the EXTERNAL injection interface: an experiment script or test
# harness publishes it to /<model_name>/imu_fault/spec. Neither the vehicle nor
# PX4 knows this message exists — PX4 simply sees corrupted IMU values.
#
# QoS: publish with transient_local + KEEP_LAST(1). Note that DDS keeps the
# latched sample in the PUBLISHER process, so the publishing node must stay
# alive for the durability to mean anything ('ros2 topic pub --once' latches
# nothing). See scripts/inject_imu_fault.py.

# --- frame the channel indices/signs are expressed in ---
uint8 FRAME_FRD=0  # PX4 / paper convention (default)
uint8 FRAME_FLU=1  # raw Gazebo sensor convention

# --- what t=0 means for every channel's t_start/t_end ---
uint8 REF_ON_COMMAND=0   # first Update tick after this spec is accepted (default)
uint8 REF_ON_SPAWN=2     # plugin's first Update tick
uint8 REF_SIM_TIME_ABS=3 # absolute sim time (t_ref = 0)

# false = full passthrough, clears the latched time reference.
bool enable

# The time reference is re-latched ONLY when this value changes. Keeps duplicate
# delivery (transient_local resend, script restart) from silently restarting the
# fault clock.
uint32 spec_seq

uint8 frame
uint8 time_ref

FaultChannel[] channels
