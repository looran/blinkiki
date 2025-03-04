ZEPHYR_BASE := $(or $(ZEPHYR_BASE),$(HOME)/src/zephyr/zephyr)
ZEPHYR_SDK := $(or $(ZEPHYR_SDK),$(HOME)/zephyr-sdk-0.17.0)
TRANS := $(or $(TRANS),wifi)
DEBUG := $(or $(DEBUG),0)
$(info ZEPHYR_BASE=$(ZEPHYR_BASE))
$(info TRANS=$(TRANS) (bt, wifi or none))
$(info DEBUG=$(DEBUG) (0 or 1))

EXTRA_CONF_FILE = 
DTC_OVERLAY_FILE = 
ifeq ($(TRANS),bt)
	EXTRA_CONF_FILE=conf/bt.conf
	DTC_OVERLAY_FILE=conf/bt.overlay
else ifeq ($(TRANS),wifi)
	EXTRA_CONF_FILE=conf/wifi.conf
	DTC_OVERLAY_FILE=conf/wifi.overlay
else ifneq ($(TRANS),none)
$(error TRANS has invalid value)
endif
ifeq ($(DEBUG),1)
	EXTRA_CONF_FILE:=$(EXTRA_CONF_FILE);conf/debug.conf
endif
export ZEPHYR_BASE EXTRA_CONF_FILE DTC_OVERLAY_FILE

help:
	# --- targets list ---
	@grep -E "^([a-zA-Z0-9_-]+:|#+)([^\=]|$$)" Makefile |grep -v help |tr -d ':'

# general

conf:
	west build -t menuconfig
hard:
	west build -t hardenconfig
clean:
	rm -rf build/
rom:
	west build -t rom_report
cons:
	while :; do picocom /dev/ttyACM0 --logfile /tmp/picocom.log && break; sleep 0.2; done
flash:
	west flash
bt-flash:
	# uses MAC variable
	smpmgr --timeout 5 --ble $(MAC) upgrade --slot 1 build/zephyr/zephyr.signed.bin
	smpmgr --timeout 5 --ble $(MAC) image state-read
bt-cons:
	bluetoothcli
wifi-flash:
	# uses IP variable
	smpmgr --ip $(IP) upgrade --slot 1 build/zephyr/zephyr.signed.bin
	smpmgr --ip $(IP) image state-read
cons-core:
	line=$$(grep -n "coredump: #CD:BEGIN#" /tmp/picocom.log |tail -n1 |cut -d: -f1) ; \
		tail -n +$$line /tmp/picocom.log |sed -e 's/.*coredump: \(.*\)\x1b.*/\1/g' > notes/coredump.log ; \
		tail -n +$$(($$line-15)) /tmp/picocom.log |head -n 15
	$(ZEPHYR_BASE)/scripts/coredump/coredump_serial_log_parser.py notes/coredump.log notes/coredump.bin
zephyr-update:
	# maybe also west packages pip --install
	cd $(ZEPHYR_BASE) \
		&& git pull \
		&& west update \
		&& west blobs fetch hal_espressif

# boards

sim:
	# native_sim build does not use conf/{bt,wifi}.overlay
	west build -b native_sim -- \
		-DDTC_OVERLAY_FILE='boards/native_sim.overlay'
sim-run:
	./build/zephyr/zephyr.exe
sim-run-gdb:
	gdb -ex run ./build/zephyr/zephyr.exe
sim-run-bt:
	sudo hciconfig hci0 down
	sudo ./build/zephyr/zephyr.exe --bt-dev=hci0
sim-setup-net:
	# you must have executed 'make sim-run' as root
	sudo ip a f dev zeth
	sudo ip a a 10.9.9.1/24 dev zeth
	sudo ip link set up dev zeth
	ping -c1 10.9.9.2

m0:
	west build -b adafruit_trinket_m0 -- \
		-DEXTRA_CONF_FILE='$(EXTRA_CONF_FILE);conf/ledstrip.conf' \
		-DDTC_OVERLAY_FILE='$(DTC_OVERLAY_FILE);boards/adafruit_trinket_m0.overlay'

esp:
	west build -b xiao_esp32s3/esp32s3/procpu -- \
		-DEXTRA_CONF_FILE='$(EXTRA_CONF_FILE);conf/mcuboot.conf' \
		-DDTC_OVERLAY_FILE='$(DTC_OVERLAY_FILE);boards/xiao_esp32s3_procpu.overlay'
	west sign -t imgtool -- --key $(ZEPHYR_BASE)/../bootloader/mcuboot/root-rsa-2048.pem 
esp-flash:
	west flash --esp-device /dev/ttyACM0 --bin-file build/zephyr/zephyr.signed.bin
esp-mon:
	west espressif monitor
esp-gdb-cons: cons-core
	# to be used after a coredump has been capture in terminal, for example in 'cons' target
	$(ZEPHYR_SDK)/xtensa-espressif_esp32s3_zephyr-elf/bin/xtensa-espressif_esp32s3_zephyr-elf-gdb \
		-ex "target remote | $(ZEPHYR_BASE)/scripts/coredump/coredump_gdbserver.py --pipe build/zephyr/zephyr.elf notes/coredump.bin" \
		build/zephyr/zephyr.elf
esp-mcuboot-flash:
	unset EXTRA_CONF_FILE DTC_OVERLAY_FILE \
		&& cd $(ZEPHYR_BASE)/.. \
		&& west build -b xiao_esp32s3/esp32s3/procpu -d build_mcuboot bootloader/mcuboot/boot/zephyr \
		&& west flash -d build_mcuboot

.PHONY: conf hard clean rom cons flash
