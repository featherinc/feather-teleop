# FeatherSDK additions this package needs

This package calls parts of FeatherSDK that are not in the released SDK (`feathersdk_v1.0`) yet. The files that
hold them are copied here from the development SDK, at their paths inside the `feathersdk` package.

Not installable: these files depend on other changes in the development SDK (the URDF loader, the motor classes)
and in its motor firmware. To run the package, install an SDK that contains them.

## New files

| File | Used for |
|---|---|
| `robot/actuator_systems/arm_kinematics.py` | the arm's solver (`ArmIK`) and guards (`Guards`): hand motion to joint motion, keeping clear of the body and the other arm |
| `robot/actuator_systems/planning.py` | `OnlineFollower`, which steps follow mode toward its newest target (Ruckig's online mode) |
| `robot/actuator_systems/collision.py`, `robot/urdf_collision.py` | the arms' collision check against the torso and base, and `CollisionError` |
| `robot/actuator_systems/joints.py` | `JointDescription`: each joint's URDF name, direction, range and limits |
| `robot/actuator_systems/trajectory.py` | imported by `planning.py` |
| `utils/mailbox.py` | the command slot behind the control modes |
| `robot/actuators/damiao/i2rt_gripper.py`, `damiao_motor.py` | the I2RT gripper (`go_to()`, `get_opening()`) |
| `robot/services/camera_process.py`, `camera_outputs.py`, `camera_encoding.py`, `camera_frames.py` | recording the cameras and their live stream to the worn headset, on the Jetson's hardware encoders: frames reach the VIC as BGRx (no CPU colour conversion), and a camera whose stream is its own picture is encoded once for both |
| `backend/` | the C++ backend: `TeensyManager` (the Teensy link), telemetry rings, motor and system classes, its build (`Makefile`) and its tests |
| `robot/comms/cpp_link.py` | the Teensy link through the backend's C API (`backend/src/bindings/c_api.cpp`), so motor feedback is received and applied outside Python's interpreter lock |

## Changed files

| File | What this package uses that was added |
|---|---|
| `robot/actuator_systems/arms_system.py` | the `follow` control mode (`set_follow_target()` at any rate, `set_follow_limits()`), `go_to()` (a move that waits), `describe()`, `read_state()`, `set_collision_check()` |
| `robot/robot.py` | `FeatherRobotConfig.only(*systems)`, a config that starts only the systems named |
| `configs/load_configs.py` | `load_user_config(version)`, the robot config of a version that ships with the SDK |
| `robot/actuator_systems/torso_system.py` | `go_to(..., wait=True)`, `describe()`, `read_state()`, `urdf_position()`, `max_velocity_mm_s`, refusing to lower the torso onto an arm |
| `robot/actuator_systems/neck_system.py` | `describe()`, `read_state()`, `set_targets()` |
| `robot/actuator_systems/manipulation_system.py` | the I2RT gripper system (`left_gripper`, `right_gripper`) |
| `robot/services/camera_service.py` | `start_recording()`, `stop_recording()`, `faults`; for the worn headset `start_stream(host, names)`, `show_stream(names)` (only the cameras shown are encoded), `stop_stream()`, `calibration()`, `get_imu_data()`, all within `encode_budget` |
| `robot/comms/comms_manager.py`, `robot/actuators/telemetry.py`, `configs/user_configs/*/infra/comms_manager_conf.py` | `CommsManagerConfig.use_cpp_comms` (off by default) runs the Teensy link in C++, and `FeatherTelemetry` reads the backend's rings. This package does not need it; it keeps feedback fresh while Python is busy |
| `logging/log.py` | `log_event()`, which marks each demonstration's start and end in the robot's log |
