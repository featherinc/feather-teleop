# Build/flash automation for the Teensy 4.1 (Ethernet OTA by default).
#
# Targets:
#   make install-cli  Install and configure arduino-cli + teensyduino + teensy-loader-cli
#   make compile      Build quill qdefs + stage + compile the sketch into ./bin
#   make flash        Ethernet OTA-flash the compiled .hex (host/teensy_host)
#                     Override reboot wait: make flash OTA_REBOOT_WAIT=30
#   make flash-usb    USB-flash the compiled .hex (teensy_loader_cli)
#   make build        compile + flash (OTA), regenerating backend/qdefs
#   make build-noquill  compile + flash (OTA) using the qdefs already in backend/qdefs
#   make build-usb    compile + flash-usb
#   make flash-station-setup  Install quill, arduino-cli/teensy tools, and a teensy-flash alias
#   make emulated     Build quill qdefs + host-side emulated teensy binary (SocketCAN + TCP)
#   make analyze      Same host build as emulated, plus -fanalyzer and ASan/UBSan
#   make test         make analyze, then build and run host unit tests under tests/unit_tests/
#   make setup-ethernet-server  Jetson serves DHCP for a directly connected Teensy (persists across reboot)
#   make setup-ethernet-clear   Remove that profile and leave the port as a DHCP client
#
# NOTE: Make sure you've shorted the gap on the board to use the terminating resistor!
# OTA requires the Jetson↔Teensy Ethernet link (feathercli setup-ethernet).

# Anchor paths to this Makefile's directory so the targets work from any cwd.
MAKEFILE_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
# Source checkout: teensy/ is six levels under the repo root. An installed copy lives
# under site-packages, so walk upward until dev_tools/quill is found.
FEATHEROS_ROOT := $(shell d="$(MAKEFILE_DIR)"; found=""; \
	while [ "$$d" != "/" ]; do \
		if [ -d "$$d/dev_tools/quill" ]; then found="$$d"; break; fi; \
		d=$$(dirname "$$d"); \
	done; \
	if [ -n "$$found" ]; then printf '%s\n' "$$found"; \
	else printf '%s\n' "$(abspath $(MAKEFILE_DIR)/../../../../../..)"; fi)
QUILL_DIR := $(FEATHEROS_ROOT)/dev_tools/quill
# teensy/ now lives under backend/src/robot/comms. Headers are generated into backend/qdefs.
BACKEND_DIR := $(abspath $(MAKEFILE_DIR)/../../../..)
QDEFS_DIR := $(BACKEND_DIR)/qdefs
PYTHON ?= python3
CXX ?= g++

FQBN    := teensy:avr:teensy41:opt=o3lto
MCU     := TEENSY41
BIN_DIR := $(MAKEFILE_DIR)/bin
# Arduino requires the sketch folder name to match the .ino basename (teensy/teensy.ino).
SKETCH_DIR := $(BIN_DIR)/teensy
HEX     := $(BIN_DIR)/teensy.ino.hex
# Max seconds teensy_host waits for the board to accept connections after OTA reboot.
# 0 means the tool's default (60s).
# Override: `make flash OTA_REBOOT_WAIT=30`
OTA_REBOOT_WAIT ?= 0

# Prefer stable by-id path; ttyACM numbering races with other USB serial devices (e.g. Pico).
PORT    := /dev/serial/by-id/usb-Teensyduino_USB_Serial_*
BAUD    := 115200

EMULATED_BIN    := $(BIN_DIR)/emulated_teensy
# -Wpedantic is off: the command table uses array designators, and the tests use compound literals.
# Both are GNU C++ extensions. C-only warning flags are omitted. `{0}` still zero-initializes in
# C++, but -Wmissing-field-initializers warns about it, so that warning is off.
EMULATED_EXTRA_WARNINGS := -Wall -Wextra -Werror -Wshadow -Wconversion -Wcast-qual -Wcast-align \
  -Wmissing-declarations -Wformat=2 -Wformat-security -Wnull-dereference \
  -Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wwrite-strings \
  -Wno-missing-field-initializers
# "qdefs/*.hpp" resolves through backend/qdefs. A relative ../qdefs include breaks once Arduino
# copies sources into bin/teensy, and would also still see the old teensy/qdefs tree.
EMULATED_CFLAGS := -std=c++17 -O3 -flto=auto -D_GNU_SOURCE -I "$(MAKEFILE_DIR)/src" -I "$(BACKEND_DIR)"
EMULATED_LIBS   := -lm -lpthread
# All firmware .cpp files under src/ (one level of subdirs) plus host emulator sources.
EMULATED_SRCS   := \
	$(wildcard $(MAKEFILE_DIR)/src/*.cpp) \
	$(wildcard $(MAKEFILE_DIR)/src/*/*.cpp) \
	$(wildcard $(MAKEFILE_DIR)/emulator/*.cpp)

UNIT_TEST_DIR   := $(MAKEFILE_DIR)/tests/unit_tests
# Recursively discover every *.cpp under tests/unit_tests/ at any depth.
UNIT_TEST_SRCS  := $(shell find "$(UNIT_TEST_DIR)" -type f -name '*.cpp' 2>/dev/null | sort)
UNIT_TEST_BINS  := $(patsubst $(UNIT_TEST_DIR)/%.cpp,$(BIN_DIR)/unit_tests/%,$(UNIT_TEST_SRCS))
UNIT_TEST_FRAMEWORK := $(MAKEFILE_DIR)/tests/framework.hpp
# Expose runtime_static symbols; firmware .cpp files are #include'd by each test (same TU).
# Emulator backends are linked (not emulated_teensy.cpp, which has main).
UNIT_TEST_CFLAGS := $(EMULATED_CFLAGS) -DTESTING_TEENSY -I "$(MAKEFILE_DIR)"
UNIT_TEST_SUPPORT_SRCS := $(filter-out \
	$(MAKEFILE_DIR)/emulator/emulated_teensy.cpp,\
	$(wildcard $(MAKEFILE_DIR)/emulator/*.cpp))

.PHONY: install-cli install-cli-prereqs install-cli-user install-cli-root qdefs compile compile-noquill sketch-dir sketch-dir-noquill _sketch-stage \
	_arduino-compile flash flash-usb build build-noquill build-usb buildmon \
	emulated emulated-nowarn analyze test clean clean-unit-tests monitor \
	setup-ethernet-server setup-ethernet-clear \
	flash-station-setup bounce-eth-link relax-perms

# `sudo make` creates root-owned bin/ and qdefs/. While root, make them writable and give them
# back to the invoking user so a later unsudoed make can delete them.
relax-perms:
	@for dir in "$(BIN_DIR)" "$(QDEFS_DIR)"; do \
		if [ ! -e "$$dir" ]; then continue; fi; \
		if [ "$$(id -u)" -eq 0 ]; then \
			chmod -R a+rwX "$$dir"; \
			if [ -n "$${SUDO_USER:-}" ]; then \
				chown -R "$$SUDO_USER:$$(id -g "$$SUDO_USER")" "$$dir"; \
			fi; \
		fi; \
	done

# backend/Makefile owns generation into backend/qdefs. Don't keep a second copy under teensy/.
qdefs:
	@$(MAKE) -C "$(BACKEND_DIR)" qdefs

# install.sh writes ./bin in the working directory. A temp dir keeps that off the installed package tree.
# arduino-cli stores cores in $HOME, so those steps run as the login user and must not call sudo.
# `sudo feathercli` is already root. The credential from that sudo belongs to the original
# terminal, and the user steps do not have it, so they cannot sudo again. Root steps run
# directly when uid is 0. `sudo make install-cli` drops to SUDO_USER for the arduino-cli half.
install-cli:
	@set -eu; \
	if [ "$$(id -u)" -eq 0 ] && [ -z "$${SUDO_USER:-}" ]; then \
		echo "install-cli: running as root with no SUDO_USER; arduino-cli data will be stored for root."; \
	fi; \
	stage=$$(mktemp -d); \
	trap 'rm -rf "$$stage"' EXIT; \
	$(MAKE) -C "$(MAKEFILE_DIR)" --no-print-directory install-cli-prereqs; \
	if [ "$$(id -u)" -eq 0 ] && [ -n "$${SUDO_USER:-}" ] && [ "$$SUDO_USER" != root ]; then \
		chown "$$SUDO_USER:" "$$stage"; \
		runuser -u "$$SUDO_USER" -- $(MAKE) -C "$(MAKEFILE_DIR)" --no-print-directory install-cli-user ARDUINO_CLI_STAGE="$$stage"; \
	else \
		$(MAKE) -C "$(MAKEFILE_DIR)" --no-print-directory install-cli-user ARDUINO_CLI_STAGE="$$stage"; \
	fi; \
	$(MAKE) -C "$(MAKEFILE_DIR)" --no-print-directory install-cli-root ARDUINO_CLI_STAGE="$$stage"

install-cli-prereqs:
	@set -eu; \
	if command -v curl >/dev/null 2>&1; then \
		exit 0; \
	fi; \
	if [ "$$(id -u)" -eq 0 ]; then \
		DEBIAN_FRONTEND=noninteractive apt-get install -y curl; \
	else \
		sudo DEBIAN_FRONTEND=noninteractive apt-get install -y curl; \
	fi

install-cli-user:
	@set -eu; \
	if [ -z "$(ARDUINO_CLI_STAGE)" ]; then \
		echo "install-cli-user requires ARDUINO_CLI_STAGE"; \
		exit 1; \
	fi; \
	mkdir -p "$(ARDUINO_CLI_STAGE)/bin"; \
	curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh -o "$(ARDUINO_CLI_STAGE)/install.sh"; \
	BINDIR="$(ARDUINO_CLI_STAGE)/bin" sh "$(ARDUINO_CLI_STAGE)/install.sh"; \
	cli="$(ARDUINO_CLI_STAGE)/bin/arduino-cli"; \
	"$$cli" config init || true; \
	"$$cli" core update-index; \
	"$$cli" config add board_manager.additional_urls https://www.pjrc.com/teensy/package_teensy_index.json || true; \
	"$$cli" core update-index; \
	"$$cli" core install teensy:avr; \
	"$$cli" lib install QNEthernet

install-cli-root:
	@set -eu; \
	if [ -z "$(ARDUINO_CLI_STAGE)" ]; then \
		echo "install-cli-root requires ARDUINO_CLI_STAGE"; \
		exit 1; \
	fi; \
	cli="$(ARDUINO_CLI_STAGE)/bin/arduino-cli"; \
	test -x "$$cli"; \
	if [ "$$(id -u)" -eq 0 ]; then \
		mv "$$cli" /usr/local/bin/arduino-cli; \
		DEBIAN_FRONTEND=noninteractive apt-get install -y teensy-loader-cli picocom; \
		mkdir -p /etc/udev/rules.d; \
		curl -fsSL -o /etc/udev/rules.d/00-teensy.rules https://www.pjrc.com/teensy/00-teensy.rules; \
	else \
		sudo mv "$$cli" /usr/local/bin/arduino-cli; \
		sudo DEBIAN_FRONTEND=noninteractive apt-get install -y teensy-loader-cli picocom; \
		sudo mkdir -p /etc/udev/rules.d; \
		sudo curl -fsSL -o /etc/udev/rules.d/00-teensy.rules https://www.pjrc.com/teensy/00-teensy.rules; \
	fi

# The NIC name differs by board (EG: eno1 on the Orin). Override with IFACE=<nic>. Otherwise use the machine's
# only onboard ethernet NIC.
bounce-eth-link:
	@set -eu; \
	iface="$(IFACE)"; \
	if [ -z "$$iface" ]; then \
		iface=$$($(PYTHON) -c 'from feathersdk.utils.system import ethernet_interfaces; n = ethernet_interfaces(); print(n[0] if len(n) == 1 else "")'); \
	fi; \
	if [ -z "$$iface" ]; then \
		echo "Error: could not pick the ethernet NIC. Pass IFACE=<nic>"; \
		exit 1; \
	fi; \
	echo "Bouncing $$iface"; \
	sudo ip link set "$$iface" down && sudo ip link set "$$iface" up

# Stage a teensy/ sketch under bin/ as symlinks. Sources are already C++. Vendor libs live next
# to src/ when present (optional). Link lib at sketch/lib for #include resolution, and under
# sketch/src/lib so Arduino compiles it (it only builds sketch root + src/, not lib/).
sketch-dir: clean qdefs
	@$(MAKE) --no-print-directory _sketch-stage

# Stage the sketch from sources and the qdefs already on disk. Does not regenerate them.
sketch-dir-noquill: clean
	@if [ ! -d "$(QDEFS_DIR)" ] || [ -z "$$(ls -A "$(QDEFS_DIR)" 2>/dev/null)" ]; then \
		echo "backend/qdefs is empty. Run 'make qdefs' in $(BACKEND_DIR) before 'make build-noquill'."; \
		exit 1; \
	fi
	@$(MAKE) --no-print-directory _sketch-stage

_sketch-stage:
	@mkdir -p "$(BIN_DIR)"
	@rm -rf "$(SKETCH_DIR)"
	@mkdir -p "$(SKETCH_DIR)/src"
	@ln -s "$(MAKEFILE_DIR)/src/teensy.ino" "$(SKETCH_DIR)/teensy.ino"
	@ln -s "$(QDEFS_DIR)" "$(SKETCH_DIR)/qdefs"
	@cd "$(MAKEFILE_DIR)/src" && find . -type f ! -name '*.ino' | while read -r f; do \
		dest="$(SKETCH_DIR)/src/$${f}"; \
		mkdir -p "$$(dirname "$$dest")"; \
		ln -s "$(MAKEFILE_DIR)/src/$${f#./}" "$$dest"; \
	done
	@if [ -d "$(MAKEFILE_DIR)/lib" ]; then \
		ln -s "$(MAKEFILE_DIR)/lib" "$(SKETCH_DIR)/lib"; \
		mkdir -p "$(SKETCH_DIR)/src"; \
		ln -s "$(MAKEFILE_DIR)/lib" "$(SKETCH_DIR)/src/lib"; \
	fi

# Compile the teensy firmware. Use -DNDEBUG to disable assertions in the firmware, and -DCOMPILE_FOR_TEENSY to enable 
# the compile-time features for the teensy. The FBQN enables -O3 and -flto
compile: sketch-dir
	@$(MAKE) --no-print-directory _arduino-compile

compile-noquill: sketch-dir-noquill
	@$(MAKE) --no-print-directory _arduino-compile

_arduino-compile:
	arduino-cli compile --fqbn $(FQBN) --clean --output-dir "$(BIN_DIR)" \
		--build-property "build.flags.defs=-D__IMXRT1062__ -DTEENSYDUINO=160 -DCOMPILE_FOR_TEENSY -DNDEBUG -Wno-unused-function -I$(BACKEND_DIR)" \
		"$(SKETCH_DIR)"
	@rm -rf "$(SKETCH_DIR)"
	@$(MAKE) --no-print-directory relax-perms

# Ethernet OTA via CMD_FLASH_TEENSY. Requires a running Teensy with Ethernet up.
HOST_BIN := $(BIN_DIR)/teensy_host
HOST_SRCS := $(wildcard $(MAKEFILE_DIR)/host/*.cpp)

$(HOST_BIN): $(HOST_SRCS)
	@mkdir -p "$(BIN_DIR)"
	$(CXX) -std=c++17 -O2 -Wall -Wextra -Wshadow -I "$(BACKEND_DIR)" -I "$(BACKEND_DIR)/src/utils" -DTEENSY_DIR='"$(MAKEFILE_DIR)"' -o "$@" $(HOST_SRCS) -lcrypto

flash: $(HOST_BIN)
	"$(HOST_BIN)" flash "$(HEX)" --reboot-wait $(OTA_REBOOT_WAIT)

# teensy_loader_cli's first write often fails right after a soft reboot, because the board
# re-enumerates from USB serial into the HalfKay bootloader and there's a brief window where the
# freshly-appeared device isn't ready to be written ("error writing to Teensy"). A failed attempt
# leaves the board sitting in HalfKay, so a retry programs it directly. Retry a few times to absorb that.
FLASH_RETRIES := 3
FLASH_RETRY_DELAY := 1

flash-usb:
	@for i in $$(seq 1 $(FLASH_RETRIES)); do \
		echo "USB flash attempt $$i/$(FLASH_RETRIES)..."; \
		if teensy_loader_cli --mcu=$(MCU) -w -v -s "$(HEX)"; then \
			sleep 5; \
			$(MAKE) bounce-eth-link; \
			exit 0; \
		fi; \
		echo "USB flash attempt $$i failed; retrying in $(FLASH_RETRY_DELAY)s..."; \
		sleep $(FLASH_RETRY_DELAY); \
	done; \
	echo "USB flash failed after $(FLASH_RETRIES) attempts."; \
	exit 1

# Ctrl-A then Ctrl-X to exit monitor, then Ctrl-C to quit.
# Resolve the Teensy by USB id (PJRC 16c0), not the first ttyACM* — a Pico often claims ACM0.
monitor:
	@trap 'exit 0' INT TERM; \
	while true; do \
		PORT=$$(ls $(PORT) 2>/dev/null | head -n1); \
		if [ -z "$$PORT" ]; then \
			for d in /dev/ttyACM*; do \
				[ -e "$$d" ] || continue; \
				if udevadm info -q property -n "$$d" 2>/dev/null | grep -q '^ID_VENDOR_ID=16c0$$'; then \
					PORT=$$d; \
					break; \
				fi; \
			done; \
		fi; \
		if [ -n "$$PORT" ]; then \
			echo "Connecting to $$PORT..."; \
			picocom --nolock -b $(BAUD) "$$PORT"; \
			echo "Disconnected."; \
		fi; \
		sleep 2; \
	done

# Resolve IFACE from the make variable (set by feathercli) or the first NetworkManager ethernet device.
define resolve_eth_iface
	iface="$(IFACE)"; \
	if [ -z "$$iface" ]; then \
		iface=$$(nmcli -t -f DEVICE,TYPE device status | awk -F: '$$2 == "ethernet" { print $$1; exit }'); \
	fi; \
	if [ -z "$$iface" ]; then \
		echo "Error: no ethernet NIC found"; \
		exit 1; \
	fi
endef

# Jetson is the DHCP server for a directly connected Teensy. NetworkManager picks the subnet. The profile
# autoconnects after reboot and comes up when a cable appears, including if nothing is plugged in at setup time.
# Other ethernet profiles have autoconnect turned off so they cannot win the port after reboot; their names are
# recorded so setup-ethernet-clear can turn them back on.
setup-ethernet-server:
	@set -eu; \
	$(resolve_eth_iface); \
	echo "Jetson DHCP server on $$iface"; \
	sudo rm -f /etc/NetworkManager/dnsmasq-shared.d/teensy.conf; \
	if sudo nmcli -t connection show teensy >/dev/null 2>&1; then \
		sudo nmcli con delete teensy; \
	fi; \
	sudo nmcli con add type ethernet ifname "$$iface" con-name teensy \
		ipv4.method shared ipv6.method disabled \
		connection.autoconnect yes connection.autoconnect-priority 100 connection.autoconnect-retries 0; \
	disabled=/var/lib/feather/ethernet-disabled-autoconnect; \
	sudo mkdir -p /var/lib/feather; \
	tmp=$$(mktemp); \
	trap 'rm -f "$$tmp" "$$tmp.cons"' EXIT; \
	if [ -f "$$disabled" ]; then sudo cat "$$disabled" > "$$tmp"; fi; \
	nmcli -t -f NAME,TYPE connection show > "$$tmp.cons"; \
	while IFS=: read -r name type; do \
		if [ "$$type" != "802-3-ethernet" ] || [ "$$name" = "teensy" ]; then continue; fi; \
		auto=$$(nmcli -g connection.autoconnect connection show "$$name"); \
		case "$$auto" in yes*) ;; *) continue ;; esac; \
		echo "Disabling autoconnect on '$$name' so the teensy profile is used after reboot"; \
		sudo nmcli con mod "$$name" connection.autoconnect no; \
		if ! grep -Fxq "$$name" "$$tmp"; then printf '%s\n' "$$name" >> "$$tmp"; fi; \
	done < "$$tmp.cons"; \
	sudo cp "$$tmp" "$$disabled"; \
	sudo chmod 644 "$$disabled"; \
	sudo nmcli device set "$$iface" managed yes; \
	if sudo nmcli con up teensy; then \
		echo "Jetson DHCP server is up on $$iface"; \
	else \
		echo "Saved the teensy DHCP server profile. It will start when a cable is connected."; \
	fi

# Router (or any other external DHCP server): drop the Jetson server profile and leave a DHCP client that
# autoconnects. No profile change when the server profile is already absent.
setup-ethernet-clear:
	@set -eu; \
	if ! sudo nmcli -t connection show teensy >/dev/null 2>&1; then \
		echo "No Jetson DHCP server profile. Ethernet stays a DHCP client."; \
		exit 0; \
	fi; \
	$(resolve_eth_iface); \
	echo "Removing Jetson DHCP server profile on $$iface"; \
	sudo nmcli con delete teensy; \
	disabled=/var/lib/feather/ethernet-disabled-autoconnect; \
	if [ -f "$$disabled" ]; then \
		restored=$$(mktemp); \
		sudo cat "$$disabled" > "$$restored"; \
		while IFS= read -r name; do \
			[ -n "$$name" ] || continue; \
			if sudo nmcli -t connection show "$$name" >/dev/null 2>&1; then \
				sudo nmcli con mod "$$name" connection.autoconnect yes; \
			fi; \
		done < "$$restored"; \
		rm -f "$$restored"; \
		sudo rm -f "$$disabled"; \
	fi; \
	need_client=1; \
	cons=$$(mktemp); \
	nmcli -t -f NAME,TYPE connection show > "$$cons"; \
	while IFS=: read -r name type; do \
		if [ "$$type" != "802-3-ethernet" ]; then continue; fi; \
		auto=$$(nmcli -g connection.autoconnect connection show "$$name" || true); \
		case "$$auto" in yes*) need_client=0 ;; esac; \
	done < "$$cons"; \
	rm -f "$$cons"; \
	if [ "$$need_client" -eq 1 ]; then \
		echo "Adding a DHCP client profile for $$iface"; \
		sudo nmcli con add type ethernet ifname "$$iface" con-name feather-dhcp \
			ipv4.method auto connection.autoconnect yes; \
	fi; \
	sudo nmcli device set "$$iface" managed yes; \
	if sudo nmcli device connect "$$iface"; then \
		echo "Ethernet $$iface is a DHCP client"; \
	else \
		echo "DHCP client profile saved. It will start when a cable is connected."; \
	fi

# Host-side emulated firmware (no COMPILE_FOR_TEENSY): TCP server + SocketCAN vcan*.
emulated: $(EMULATED_BIN)

$(EMULATED_BIN): $(EMULATED_SRCS) qdefs
	@mkdir -p "$(BIN_DIR)"
	$(CXX) $(EMULATED_EXTRA_WARNINGS) $(EMULATED_CFLAGS) $(EMULATED_SRCS) $(EMULATED_LIBS) -o "$@"
	chmod +x "$@"
	@$(MAKE) --no-print-directory relax-perms
	@echo "Built $@"

emulated-nowarn: $(EMULATED_BIN)-nowarn

$(EMULATED_BIN)-nowarn: $(EMULATED_SRCS) qdefs
	@mkdir -p "$(BIN_DIR)"
	$(CXX) $(EMULATED_CFLAGS) $(EMULATED_SRCS) $(EMULATED_LIBS) -o "$@"
	chmod +x "$@"
	@$(MAKE) --no-print-directory relax-perms
	@echo "Built $@"

# Same sources, warnings, and -O3 -flto as `emulated`. -fanalyzer runs at LTO time, so findings
# can cross translation units. Sanitizers instrument the binary; they report when it is run.
# address covers overflows, use-after-free, and leaks. undefined covers signed overflow, bad
# shifts, and misaligned access. The float checks are off by default in -fsanitize=undefined
# and matter for motor scaling. -fno-sanitize-recover=all aborts on the first finding.
# Separate output so this does not replace bin/emulated_teensy.
ANALYZE_CFLAGS := $(EMULATED_EXTRA_WARNINGS) $(EMULATED_CFLAGS) -fanalyzer \
  -fsanitize=address,undefined,float-divide-by-zero,float-cast-overflow \
  -fno-omit-frame-pointer -fno-sanitize-recover=all

analyze: $(EMULATED_BIN)-analyze

$(EMULATED_BIN)-analyze: $(EMULATED_SRCS) qdefs
	@mkdir -p "$(BIN_DIR)"
	$(CXX) $(ANALYZE_CFLAGS) $(EMULATED_SRCS) $(EMULATED_LIBS) -o "$@"
	chmod +x "$@"
	@$(MAKE) --no-print-directory relax-perms
	@echo "Built $@"

# Host unit tests: analyzer build first, then one binary per tests/unit_tests/**/*.cpp; rebuild then run all.
clean-unit-tests:
	@if ! rm -rf "$(BIN_DIR)/unit_tests" 2>/dev/null; then \
		echo "Fixing permissions on $(BIN_DIR) left by a previous sudo make..."; \
		sudo chown -R "$$(id -un):$$(id -gn)" "$(BIN_DIR)"; \
		sudo chmod -R a+rwX "$(BIN_DIR)"; \
		rm -rf "$(BIN_DIR)/unit_tests"; \
	fi

test: qdefs clean-unit-tests
	@$(MAKE) --no-print-directory analyze
	@if [ -z "$(UNIT_TEST_SRCS)" ]; then \
		echo "No unit tests in $(UNIT_TEST_DIR)"; \
		exit 0; \
	fi
	@echo "Building $$(echo $(UNIT_TEST_SRCS) | wc -w) unit test(s)..."
	@$(MAKE) --no-print-directory -k -j$$(nproc) $(UNIT_TEST_BINS); \
	$(MAKE) --no-print-directory relax-perms; \
	echo ""; \
	echo "Running unit tests..."; \
	echo ""; \
	failed=0; \
	for t in $(UNIT_TEST_BINS); do \
		if [ ! -x "$$t" ]; then \
			echo "$$t... BUILD FAILED"; \
			failed=1; \
			continue; \
		fi; \
		if ! "$$t"; then \
			failed=1; \
		fi; \
	done; \
	echo ""; \
	if [ "$$failed" -ne 0 ]; then \
		echo "FAILED TESTS"; \
		exit 1; \
	fi; \
	echo "*** All Tests Passing! ***"

# One-time setup for a USB flash station: editable quill install, arduino-cli/teensy
# toolchain, and a `teensy-flash` alias that USB-builds from this directory.
flash-station-setup:
	$(PYTHON) -m pip install -e "$(QUILL_DIR)"
	$(MAKE) install-cli
	@if [ -f "$(HOME)/.bashrc" ]; then sed -i '/^alias teensy-flash=/d' "$(HOME)/.bashrc"; fi
	@echo "alias teensy-flash='make -C $(MAKEFILE_DIR) build-usb'" >> "$(HOME)/.bashrc"
	@echo "Added alias: teensy-flash -> make -C $(MAKEFILE_DIR) build-usb"
	@echo "Run 'source ~/.bashrc' or open a new shell to use it."

$(BIN_DIR)/unit_tests/%: $(UNIT_TEST_DIR)/%.cpp $(UNIT_TEST_SUPPORT_SRCS) $(UNIT_TEST_FRAMEWORK) qdefs
	@mkdir -p "$(dir $@)"
	$(CXX) $(UNIT_TEST_CFLAGS) $< $(UNIT_TEST_SUPPORT_SRCS) $(EMULATED_LIBS) -o "$@"

build: compile flash
build-noquill: compile-noquill flash
build-usb: compile flash-usb
buildmon: compile flash monitor
buildmon-usb: compile flash-usb monitor

clean:
	@if ! rm -rf "$(BIN_DIR)"/* 2>/dev/null; then \
		echo "Fixing permissions on $(BIN_DIR) left by a previous sudo make..."; \
		sudo chown -R "$$(id -un):$$(id -gn)" "$(BIN_DIR)"; \
		sudo chmod -R a+rwX "$(BIN_DIR)"; \
		rm -rf "$(BIN_DIR)"/*; \
	fi
