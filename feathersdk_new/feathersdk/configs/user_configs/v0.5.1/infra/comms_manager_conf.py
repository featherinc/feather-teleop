from feathersdk.robot.comms import CommsManagerConfig


comms_manager_config = CommsManagerConfig(
    teensy_operating_frequency=100.0,
    teensy_telemetry_update_frequency=100.0,
    max_can_rate=3500.0,
    can_leniency=100.0,
    server_run_time_nanos=100_000,
    can_read_run_time_nanos=100_000,
    result_operating_frequency=200.0,
    # use_cpp_comms=True,  # the Teensy link in C++ (robot/comms/cpp_link.py): feedback stays fresh while Python is busy
)
