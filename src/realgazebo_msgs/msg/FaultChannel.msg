# One fault applied to one scalar sensor channel.
# Sensor-agnostic on purpose: reused for mag/baro/GPS fault injection later.

# --- fault types ---
uint8 T_NONE=0
uint8 T_BIAS=1
uint8 T_DRIFT=2
uint8 T_OSCILLATION=3
# 4..11 reserved: STUCK / SCALE / NOISE / RANDOM_WALK / SATURATE / QUANTIZE / SPIKE / DEAD
# (those are non-additive or message-level and need the relay backend — see
#  docs/imu_fault_injection-plan.md section 8)

# --- behaviour once the window closes ---
uint8 EXIT_SNAP=0       # fault drops to 0 instantly at t_end (default)
# The ANALYTIC waveform value at t_end (fade-in included) is latched forever —
# not whatever the last executed tick happened to sample, and it applies even if
# the fault clock jumped straight past the window. Models permanent degradation.
uint8 EXIT_HOLD_LAST=1

# Channel index. For IMU: 0=ax 1=ay 2=az 3=p 4=q 5=r, in the frame named by
# the parent ImuFaultSpec (default FRD).
uint8 channel

uint8 type

# Magnitude parameters, interpreted per type:
#   T_BIAS        p0 = bias            [channel unit]
#   T_DRIFT       p0 = rate            [channel unit / s]
#   T_OSCILLATION p0 = amplitude       [channel unit]
#                 p1 = frequency       [Hz]
#                 p2 = phase           [rad]
float64 p0
float64 p1
float64 p2

# Activation window, in seconds relative to the spec's time reference.
# t_end <= t_start means "no end" (runs forever).
float64 t_start
float64 t_end

# Linear fade-in over this many seconds from t_start. 0 = hard step.
float64 ramp_in

uint8 on_exit 0
