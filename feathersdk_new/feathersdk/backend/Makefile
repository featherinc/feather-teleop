# Backend build.
#
#   make log_write_speed       Compile, run, then delete the log write benchmark
#   make log_write_speed_prof  Same benchmark under perf, then print the report
#   make telemetry_speed       Compile, run, then delete the telemetry benchmark
#   make monitor_write_speed   Compile, run, then delete the monitor file-writer benchmark
#   make qdefs                 Generate C++ headers from quill defs into ./qdefs
#   make unit_tests            Build and run every tests/unit_tests/**/*.cpp file
#   make teensy_echo           Connect to a Teensy and check one echo reply
#   make teensy_comms_speed    Echo as fast as the link allows without dropping a reply
#   make test_robstride        Init one Robstride motor, find its CAN bus, print temperature
#                              Optional motor id: make test_robstride MOTOR_ID=0x30
#   make drive_test            Drive the 3 base wheels forward at 0.5 rad/s for 3 seconds
#   make full_motor_update     Feedback round-trip rate for the base and both arms
#   make neck_vcan             vcan0, two virtual neck motors, and the emulated Teensy
#
# Paths are anchored to this file, so the targets work from any working directory.

MAKEFILE_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
# Source checkout: backend/ is four levels under the repo root. An installed copy lives
# under site-packages, so walk upward until dev_tools/quill is found.
FEATHEROS_ROOT := $(shell d="$(MAKEFILE_DIR)"; found=""; \
	while [ "$$d" != "/" ]; do \
		if [ -d "$$d/dev_tools/quill" ]; then found="$$d"; break; fi; \
		d=$$(dirname "$$d"); \
	done; \
	if [ -n "$$found" ]; then printf '%s\n' "$$found"; \
	else printf '%s\n' "$(abspath $(MAKEFILE_DIR)/../../../..)"; fi)
QUILL_DIR := $(FEATHEROS_ROOT)/dev_tools/quill
QUILL_SRC := $(QUILL_DIR)/src
QUILL_DEFS_DIR := $(abspath $(MAKEFILE_DIR)/../quill_defs/defs)
QDEFS_DIR := $(MAKEFILE_DIR)/qdefs
PYTHON ?= python3

CXX      := g++
CXXFLAGS := -std=c++17 -O3 -Wall -Wextra -Wshadow -pthread -I$(MAKEFILE_DIR) -I$(MAKEFILE_DIR)/src/utils -flto -march=native
PERF     := $(firstword $(wildcard /usr/lib/linux-tools/*/perf))

LOG_WRITE_SPEED_SRC := $(MAKEFILE_DIR)/tests/speed_tests/log_write_speed.cpp
TELEMETRY_SPEED_SRC := $(MAKEFILE_DIR)/tests/speed_tests/telemetry_speed.cpp
MONITOR_WRITE_SPEED_SRC := $(MAKEFILE_DIR)/tests/speed_tests/monitor_write_speed.cpp
DATA_MONITOR_SRC := $(MAKEFILE_DIR)/src/robot/services/data_monitor.cpp
TELEMETRY_SRC := $(MAKEFILE_DIR)/src/robot/telemetry.cpp
LOG_SRC := $(MAKEFILE_DIR)/src/utils/logging.cpp
SYSTEM_SRC := $(MAKEFILE_DIR)/src/utils/system.cpp
TEENSY_ECHO_SRC := $(MAKEFILE_DIR)/tests/teensy_echo.cpp
TEENSY_COMMS_SPEED_SRC := $(MAKEFILE_DIR)/tests/speed_tests/teensy_comms_speed.cpp
TEENSY_LINK_SRCS := \
	$(MAKEFILE_DIR)/src/robot/comms/teensy_manager.cpp \
	$(MAKEFILE_DIR)/src/robot/comms/teensy_socket.cpp \
	$(MAKEFILE_DIR)/src/robot/comms/teensy/host/recompile.cpp \
	$(TELEMETRY_SRC) \
	$(MAKEFILE_DIR)/src/utils/logging.cpp \
	$(MAKEFILE_DIR)/src/utils/system.cpp \
	$(MAKEFILE_DIR)/src/utils/run.cpp
TEENSY_ECHO_SRCS := $(TEENSY_ECHO_SRC) $(TEENSY_LINK_SRCS)
TEENSY_COMMS_SPEED_SRCS := $(TEENSY_COMMS_SPEED_SRC) $(TEENSY_LINK_SRCS)
TEST_ROBSTRIDE_SRC := $(MAKEFILE_DIR)/tests/test_robstride.cpp
DRIVE_TEST_SRC := $(MAKEFILE_DIR)/tests/one_off_tests/drive_test.cpp
FULL_MOTOR_UPDATE_SRC := $(MAKEFILE_DIR)/tests/speed_tests/full_motor_update.cpp
DRIVE_TEST_SRCS := \
	$(DRIVE_TEST_SRC) \
	$(TEENSY_LINK_SRCS) \
	$(MAKEFILE_DIR)/src/robot/comms/comms_manager.cpp \
	$(MAKEFILE_DIR)/src/robot/actuators/robstride.cpp \
	$(DATA_MONITOR_SRC)
TEST_ROBSTRIDE_SRCS := \
	$(TEST_ROBSTRIDE_SRC) \
	$(TEENSY_LINK_SRCS) \
	$(MAKEFILE_DIR)/src/robot/comms/comms_manager.cpp \
	$(MAKEFILE_DIR)/src/robot/actuators/robstride.cpp

# The Python SDK's C++ link (robot/comms/cpp_link.py builds it on first use, then make keeps it current). No -flto
# -march=native here: it is built on the machine that loads it, but must load in every process of that machine.
LIB_SRCS := \
	$(MAKEFILE_DIR)/src/bindings/c_api.cpp \
	$(DATA_MONITOR_SRC) \
	$(MAKEFILE_DIR)/src/robot/comms/teensy_manager.cpp \
	$(MAKEFILE_DIR)/src/robot/comms/teensy_socket.cpp \
	$(MAKEFILE_DIR)/src/robot/comms/teensy/host/recompile.cpp \
	$(TELEMETRY_SRC) \
	$(MAKEFILE_DIR)/src/utils/logging.cpp \
	$(MAKEFILE_DIR)/src/utils/system.cpp \
	$(MAKEFILE_DIR)/src/utils/run.cpp
LIB_HDRS := $(shell find $(MAKEFILE_DIR)/src $(MAKEFILE_DIR)/qdefs -name '*.hpp' -not -path '*/comms/teensy/*')
LIB := $(MAKEFILE_DIR)/build/libfeather_backend.so

lib: $(LIB)

$(LIB): $(LIB_SRCS) $(LIB_HDRS) $(MAKEFILE_DIR)/Makefile
	@mkdir -p $(dir $@)
	$(CXX) -std=c++17 -O2 -Wall -Wextra -Wshadow -pthread -fPIC -shared -I$(MAKEFILE_DIR) -I$(MAKEFILE_DIR)/src/utils \
		-o $@.tmp $(LIB_SRCS) && mv $@.tmp $@

.PHONY: lib log_write_speed log_write_speed_prof telemetry_speed monitor_write_speed qdefs relax-perms unit_tests teensy_echo teensy_comms_speed test_robstride drive_test full_motor_update neck_vcan

UNIT_TEST_SRCS := $(shell find $(MAKEFILE_DIR)/tests/unit_tests -name '*.cpp')
# comms/teensy is a separate firmware. Its tests #include the firmware .cpp files, and
# emulator/emulated_teensy.cpp has its own main, so linking that tree here defines every
# symbol twice. Host code that calls recompile_if_needed still needs host/recompile.cpp.
BACKEND_SRCS := $(shell find $(MAKEFILE_DIR)/src -name '*.cpp' -not -path '*/comms/teensy/*') \
	$(MAKEFILE_DIR)/src/robot/comms/teensy/host/recompile.cpp

# One binary per tests/unit_tests/**/*.cpp. Rebuild, run, then delete it.
# framework.hpp is a C harness; g++ warns on its designated initializers and truncation strncpy.
unit_tests:
	@fail=0; \
	for src in $(UNIT_TEST_SRCS); do \
		bin="/tmp/feather_$$(basename "$$src" .cpp)"; \
		echo "BUILD $$src"; \
		$(CXX) $(CXXFLAGS) -Wno-missing-field-initializers -Wno-stringop-truncation -o "$$bin" "$$src" $(BACKEND_SRCS) || { fail=1; continue; }; \
		"$$bin" || fail=1; \
		rm -f "$$bin"; \
	done; \
	exit $$fail

# `sudo make` creates a root-owned qdefs/. While root, make it writable and give it
# back to the invoking user so a later unsudoed make can delete it.
relax-perms:
	@dir="$(QDEFS_DIR)"; \
	if [ ! -e "$$dir" ]; then exit 0; fi; \
	if [ "$$(id -u)" -eq 0 ]; then \
		chmod -R a+rwX "$$dir"; \
		if [ -n "$${SUDO_USER:-}" ]; then \
			chown -R "$$SUDO_USER:$$(id -g "$$SUDO_USER")" "$$dir"; \
		fi; \
	fi

# Generate C++ headers from quill definition Python modules into ./qdefs/.
qdefs:
	@rm -rf "$(QDEFS_DIR)"
	@mkdir -p "$(QDEFS_DIR)"
	@echo "Building quill definitions: $(QUILL_DEFS_DIR) -> $(QDEFS_DIR)"
	PYTHONPATH="$(QUILL_SRC)" $(PYTHON) -m quill "$(QUILL_DEFS_DIR)" "$(QDEFS_DIR)" -l cpp
	@$(MAKE) --no-print-directory relax-perms

# No -flto: update_telemetry has to stay a real call. With LTO the rings are never
# read back, so the timed loop can be deleted.
telemetry_speed: $(TELEMETRY_SPEED_SRC) $(TELEMETRY_SRC) $(LOG_SRC) $(SYSTEM_SRC)
	$(CXX) $(filter-out -flto,$(CXXFLAGS)) -o /tmp/telemetry_speed $(TELEMETRY_SPEED_SRC) $(TELEMETRY_SRC) $(LOG_SRC) $(SYSTEM_SRC)
	/tmp/telemetry_speed; status=$$?; rm -f /tmp/telemetry_speed; exit $$status

# No -flto: DataMonitorFileWriter::write has to stay a real call.
monitor_write_speed: $(MONITOR_WRITE_SPEED_SRC) $(DATA_MONITOR_SRC) $(LOG_SRC) $(SYSTEM_SRC)
	$(CXX) $(filter-out -flto,$(CXXFLAGS)) -o /tmp/monitor_write_speed $(MONITOR_WRITE_SPEED_SRC) $(DATA_MONITOR_SRC) $(LOG_SRC) $(SYSTEM_SRC)
	/tmp/monitor_write_speed; status=$$?; rm -f /tmp/monitor_write_speed; exit $$status

# Starts the emulated Teensy, then checks one echo and one cansend. Not part of unit_tests.
teensy_echo: $(TEENSY_ECHO_SRCS)
	$(CXX) $(CXXFLAGS) -pthread -o /tmp/teensy_echo $(TEENSY_ECHO_SRCS)
	/tmp/teensy_echo; status=$$?; rm -f /tmp/teensy_echo; exit $$status

teensy_comms_speed: $(TEENSY_COMMS_SPEED_SRCS)
	$(CXX) $(CXXFLAGS) -pthread -o /tmp/teensy_comms_speed $(TEENSY_COMMS_SPEED_SRCS)
	/tmp/teensy_comms_speed; status=$$?; rm -f /tmp/teensy_comms_speed; exit $$status

# Talks to a Robstride motor on the connected Teensy 4.1. MOTOR_ID is optional (decimal or 0x).
test_robstride: $(TEST_ROBSTRIDE_SRCS)
	$(CXX) $(CXXFLAGS) -pthread -o /tmp/test_robstride $(TEST_ROBSTRIDE_SRCS)
	/tmp/test_robstride $(MOTOR_ID); status=$$?; rm -f /tmp/test_robstride; exit $$status

FULL_MOTOR_UPDATE_SRCS := \
	$(FULL_MOTOR_UPDATE_SRC) \
	$(TEENSY_LINK_SRCS) \
	$(MAKEFILE_DIR)/src/robot/comms/comms_manager.cpp \
	$(MAKEFILE_DIR)/src/robot/actuators/robstride.cpp \
	$(DATA_MONITOR_SRC)

# Talks to the three base drive motors on the connected Teensy and drives them forward.
drive_test: $(DRIVE_TEST_SRCS)
	$(CXX) $(CXXFLAGS) -pthread -o /tmp/drive_test $(DRIVE_TEST_SRCS)
	/tmp/drive_test; status=$$?; rm -f /tmp/drive_test; exit $$status

NECK_VCAN_SRC := $(MAKEFILE_DIR)/tests/one_off_tests/neck_vcan.cpp

# vcan0, two virtual neck motors, and the emulated Teensy. Commands a pose and checks feedback.
neck_vcan: $(NECK_VCAN_SRC)
	$(CXX) $(CXXFLAGS) -pthread -o /tmp/neck_vcan $(NECK_VCAN_SRC) $(BACKEND_SRCS)
	/tmp/neck_vcan; status=$$?; rm -f /tmp/neck_vcan; exit $$status

# Feedback round trips for every base and arm Robstride motor. Does not command motion.
full_motor_update: $(FULL_MOTOR_UPDATE_SRCS)
	$(CXX) $(CXXFLAGS) -pthread -o /tmp/full_motor_update $(FULL_MOTOR_UPDATE_SRCS)
	/tmp/full_motor_update; status=$$?; rm -f /tmp/full_motor_update; exit $$status

log_write_speed: $(LOG_WRITE_SPEED_SRC) $(LOG_SRC) $(SYSTEM_SRC)
	$(CXX) $(CXXFLAGS) -o /tmp/log_write_speed $(LOG_WRITE_SPEED_SRC) $(LOG_SRC) $(SYSTEM_SRC)
	/tmp/log_write_speed; status=$$?; rm -f /tmp/log_write_speed; exit $$status

log_write_speed_prof: $(LOG_WRITE_SPEED_SRC) $(LOG_SRC) $(SYSTEM_SRC)
	$(CXX) $(CXXFLAGS) -g -fno-omit-frame-pointer -o /tmp/log_write_speed $(LOG_WRITE_SPEED_SRC) $(LOG_SRC) $(SYSTEM_SRC)
	$(PERF) record -F 999 --call-graph fp -o /tmp/log_write_speed.perf -- /tmp/log_write_speed; \
		status=$$?; \
		$(PERF) report -i /tmp/log_write_speed.perf --stdio --inline --no-children; \
		rm -f /tmp/log_write_speed /tmp/log_write_speed.perf; \
		exit $$status
