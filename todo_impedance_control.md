# Safety features:
- Avoid reboot on fault jumping back to impedance control: prevent mode switch from IDLE to to impedance control: need to pass through POSITION. OR: enable impedance only if position close to setpoint: Add new error "juped into impedance far from setpoint" ?
- Maximum tracking error in impedance control
- Deal with torque commands safety: continuity, what happens if no new command received in a while (hardware interface side?) - covered by maximum tracking error in impedance control mode?


# New features
- steam stiffness / damping / (Kp?) (firmware + ROS2 side)
- DONE - better simulation mode?
- switch between current / torque control

# new paramters
- Max position error in impedance mode (float)
- Max CAN timeout error (INT (ms))
- Disable torque sensor - Current control instead of torque (bool)
- Disable impedance control tracking error (bool)
- Disable impedance control jumpstart safety (bool)
- impedance control jumpstart angle treshold (float)

# bug fix:
- DONE - Effort commands sent from ROS2 never received
- 

# Doc:
- impedance control scheme
- virtual end stop maths

# test scripts:

ros2 launch explorer_bringup custom_controller_joint_control.launch.py

ros2 topic pub /explorer_custom_controller/effort/commands std_msgs/msg/Float64MultiArray "{data: [0.0,0.0,0.0,0.0,0.0,0.0]}"
