# This is the msg definition for the Segway Status struct.

# Gain Schedule Constants
int8    LIGHT=1
int8    TALL=2
int8    HEAVY=3

# Operation Mode Constants
int8    BALANCE=1
int8    TRACTOR=2
int8    POWER_DOWN=3

Header  header                      # Timestamp, sequence number, and frame id

float32 pitch_angle                 # degrees
float32 pitch_rate                  # degrees/s
float32 roll_angle                  # degrees
float32 roll_rate                   # degrees/s
float32 left_wheel_velocity         # meters/s
float32 right_wheel_velocity        # meters/s
float32 yaw_rate                    # degrees/s
float32 servo_frames                # frames/second
float32 left_wheel_displacement     # meters
float32 right_wheel_displacement    # meters
float32 forward_displacement        # meters
float32 yaw_displacement            # revolutions
float32 left_motor_torque           # Newton-meters
float32 right_motor_torque          # Newton-meters
int8    operation_mode              # Balance, Tractor, or Powered Down
int8    gain_schedule               # Light, Tall, or Heavy
float32 ui_battery                  # Volts
float32 powerbase_battery           # Volts

float32 pitch_angle2                # degrees
float32 pitch_rate2                 # degrees/s
float32 roll_angle2                 # degrees
float32 roll_rate2                  # degrees/s
float32 left_wheel_velocity2        # meters/s
float32 right_wheel_velocity2       # meters/s
float32 yaw_rate2                   # degrees/s
float32 servo_frames2               # frames/second
float32 left_wheel_displacement2    # meters
float32 right_wheel_displacement2   # meters
float32 forward_displacement2       # meters
float32 yaw_displacement2           # revolutions
float32 left_motor_torque2          # Newton-meters
float32 right_motor_torque2         # Newton-meters
int8    operation_mode2             # Balance, Tractor, or Powered Down
int8    gain_schedule2              # Light, Tall, or Heavy
float32 ui_battery2                 # Volts
float32 powerbase_battery2          # Volts