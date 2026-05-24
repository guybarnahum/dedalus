# PX4 Hardware Target

Real PX4 hardware configuration, preflight validation, and hardware-specific scripts.

The vehicle command abstraction should match the AirSim PX4 SITL path: bounded velocity plus
optional yaw through PX4/MAVLink Offboard. Camera/gimbal pointing should use a separate
camera/gimbal sink, such as MAVLink Gimbal Protocol v2 when hardware supports it.
