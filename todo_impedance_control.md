# Safety features:
- Avoid reboot on fault jumping back to impedance control: prevent mode switch from IDLE to to impedance control: need to pass through POSITION. OR: enable impedance only if position close to setpoint: Add new error "juped into impedance far from setpoint" ?
- Maximum tracking error in impedance control

# New features
- steam stiffness / damping / (Kp?) (firmware + ROS2 side)
- better simulation mode?
- switch between current / torque control

# new paramters
- Max position error in impedance mode
- Max CAN timeout error
- 

# bug fix:
- Effort commands sent from ROS2 never received
- 

# Doc:
- empedance control scheme
- virtual end stop maths