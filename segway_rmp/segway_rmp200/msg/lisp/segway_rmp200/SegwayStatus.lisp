; Auto-generated. Do not edit!


(in-package segway_rmp200-msg)


;//! \htmlinclude SegwayStatus.msg.html

(defclass <SegwayStatus> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (pitch_angle
    :reader pitch_angle-val
    :initarg :pitch_angle
    :type float
    :initform 0.0)
   (pitch_rate
    :reader pitch_rate-val
    :initarg :pitch_rate
    :type float
    :initform 0.0)
   (roll_angle
    :reader roll_angle-val
    :initarg :roll_angle
    :type float
    :initform 0.0)
   (roll_rate
    :reader roll_rate-val
    :initarg :roll_rate
    :type float
    :initform 0.0)
   (left_wheel_velocity
    :reader left_wheel_velocity-val
    :initarg :left_wheel_velocity
    :type float
    :initform 0.0)
   (right_wheel_velocity
    :reader right_wheel_velocity-val
    :initarg :right_wheel_velocity
    :type float
    :initform 0.0)
   (yaw_rate
    :reader yaw_rate-val
    :initarg :yaw_rate
    :type float
    :initform 0.0)
   (servo_frames
    :reader servo_frames-val
    :initarg :servo_frames
    :type float
    :initform 0.0)
   (left_wheel_displacement
    :reader left_wheel_displacement-val
    :initarg :left_wheel_displacement
    :type float
    :initform 0.0)
   (right_wheel_displacement
    :reader right_wheel_displacement-val
    :initarg :right_wheel_displacement
    :type float
    :initform 0.0)
   (forward_displacement
    :reader forward_displacement-val
    :initarg :forward_displacement
    :type float
    :initform 0.0)
   (yaw_displacement
    :reader yaw_displacement-val
    :initarg :yaw_displacement
    :type float
    :initform 0.0)
   (left_motor_torque
    :reader left_motor_torque-val
    :initarg :left_motor_torque
    :type float
    :initform 0.0)
   (right_motor_torque
    :reader right_motor_torque-val
    :initarg :right_motor_torque
    :type float
    :initform 0.0)
   (operation_mode
    :reader operation_mode-val
    :initarg :operation_mode
    :type fixnum
    :initform 0)
   (gain_schedule
    :reader gain_schedule-val
    :initarg :gain_schedule
    :type fixnum
    :initform 0)
   (ui_battery
    :reader ui_battery-val
    :initarg :ui_battery
    :type float
    :initform 0.0)
   (powerbase_battery
    :reader powerbase_battery-val
    :initarg :powerbase_battery
    :type float
    :initform 0.0))
)
(defmethod symbol-codes ((msg-type (eql '<SegwayStatus>)))
  "Constants for message type '<SegwayStatus>"
  '((:LIGHT . 1)
    (:TALL . 2)
    (:HEAVY . 3)
    (:BALANCE . 1)
    (:TRACTOR . 2)
    (:POWER_DOWN . 3))
)
(defmethod serialize ((msg <SegwayStatus>) ostream)
  "Serializes a message object of type '<SegwayStatus>"
  (serialize (slot-value msg 'header) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'pitch_angle))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'pitch_rate))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'roll_angle))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'roll_rate))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'left_wheel_velocity))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'right_wheel_velocity))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'yaw_rate))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'servo_frames))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'left_wheel_displacement))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'right_wheel_displacement))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'forward_displacement))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'yaw_displacement))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'left_motor_torque))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'right_motor_torque))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
    (write-byte (ldb (byte 8 0) (slot-value msg 'operation_mode)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'gain_schedule)) ostream)
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'ui_battery))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'powerbase_battery))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <SegwayStatus>) istream)
  "Deserializes a message object of type '<SegwayStatus>"
  (deserialize (slot-value msg 'header) istream)
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'pitch_angle) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'pitch_rate) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'roll_angle) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'roll_rate) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'left_wheel_velocity) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'right_wheel_velocity) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'yaw_rate) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'servo_frames) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'left_wheel_displacement) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'right_wheel_displacement) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'forward_displacement) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'yaw_displacement) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'left_motor_torque) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'right_motor_torque) (roslisp-utils:decode-single-float-bits bits)))
  (setf (ldb (byte 8 0) (slot-value msg 'operation_mode)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'gain_schedule)) (read-byte istream))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'ui_battery) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'powerbase_battery) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<SegwayStatus>)))
  "Returns string type for a message object of type '<SegwayStatus>"
  "segway_rmp200/SegwayStatus")
(defmethod md5sum ((type (eql '<SegwayStatus>)))
  "Returns md5sum for a message object of type '<SegwayStatus>"
  "9e3af46c865b45fc481ea8afefffc392")
(defmethod message-definition ((type (eql '<SegwayStatus>)))
  "Returns full string definition for message of type '<SegwayStatus>"
  (format nil "# This is the msg definition for the Segway Status struct.~%~%# Gain Schedule Constants~%int8    LIGHT=1~%int8    TALL=2~%int8    HEAVY=3~%~%# Operation Mode Constants~%int8    BALANCE=1~%int8    TRACTOR=2~%int8    POWER_DOWN=3~%~%Header  header                      # Timestamp, sequence number, and frame id~%~%float32 pitch_angle                 # degrees~%float32 pitch_rate                  # degrees/s~%float32 roll_angle                  # degrees~%float32 roll_rate                   # degrees/s~%float32 left_wheel_velocity         # meters/s~%float32 right_wheel_velocity        # meters/s~%float32 yaw_rate                    # degrees/s~%float32 servo_frames                # frames/second~%float32 left_wheel_displacement     # meters~%float32 right_wheel_displacement    # meters~%float32 forward_displacement        # meters~%float32 yaw_displacement            # revolutions~%float32 left_motor_torque           # Newton-meters~%float32 right_motor_torque          # Newton-meters~%int8    operation_mode              # Balance, Tractor, or Powered Down~%int8    gain_schedule               # Light, Tall, or Heavy~%float32 ui_battery                  # Volts~%float32 powerbase_battery           # Volts~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <SegwayStatus>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     1
     1
     4
     4
))
(defmethod ros-message-to-list ((msg <SegwayStatus>))
  "Converts a ROS message object to a list"
  (list '<SegwayStatus>
    (cons ':header (header-val msg))
    (cons ':pitch_angle (pitch_angle-val msg))
    (cons ':pitch_rate (pitch_rate-val msg))
    (cons ':roll_angle (roll_angle-val msg))
    (cons ':roll_rate (roll_rate-val msg))
    (cons ':left_wheel_velocity (left_wheel_velocity-val msg))
    (cons ':right_wheel_velocity (right_wheel_velocity-val msg))
    (cons ':yaw_rate (yaw_rate-val msg))
    (cons ':servo_frames (servo_frames-val msg))
    (cons ':left_wheel_displacement (left_wheel_displacement-val msg))
    (cons ':right_wheel_displacement (right_wheel_displacement-val msg))
    (cons ':forward_displacement (forward_displacement-val msg))
    (cons ':yaw_displacement (yaw_displacement-val msg))
    (cons ':left_motor_torque (left_motor_torque-val msg))
    (cons ':right_motor_torque (right_motor_torque-val msg))
    (cons ':operation_mode (operation_mode-val msg))
    (cons ':gain_schedule (gain_schedule-val msg))
    (cons ':ui_battery (ui_battery-val msg))
    (cons ':powerbase_battery (powerbase_battery-val msg))
))
