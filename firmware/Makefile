include Makeconf.local

PROJ_DIR := src/
BUILD_DIR = build
OUTPUT_DIRECTORY := $(BUILD_DIR)/sturmfront
BIN_DIRECTORY := $(BUILD_DIR)/bin
PUBLIC_KEY_SRC := dfu_public_key.c

SRC_FILES += \
  $(PROJ_DIR)/configuration.cpp \
  $(PROJ_DIR)/main.cpp \


INC_FOLDERS += \
  mk/config \
  $(PROJ_DIR) \
  boards \


BOARD = CUSTOM_BOARD_INC=sturmfront_v0

include mk/Makefile

APP_VERSION = 0
BL_VERSION = 0
BL_HEX = $(BUILD_DIR)/bootloader/nrf52832_xxaa_s332.hex

$(BUILD_DIR):
	$(MK) $@

$(OUTPUT_DIRECTORY): $(BUILD_DIR)

$(BIN_DIRECTORY): $(BUILD_DIRECTORY)
	$(MK) $@

.PHONY:	merge settings dfu bootloader

# Flash softdevice
flash_softdevice:
	@echo Flashing: ANT_s332_nrf52_7.0.1.hex
	nrfjprog -f nrf52 --program $(SDK_ROOT)/components/softdevice/s332/hex/ANT_s332_nrf52_7.0.1.hex --sectorerase
	nrfjprog -f nrf52 --reset

$(PUBLIC_KEY_SRC): $(PRIVATE_KEY)
	nrfutil keys display --key pk --format code $< > $@

bootloader: $(BUILD_DIR) dfu_public_key.c
	@echo Compile Bootloader:
	make -f mk/Makefile_bl SDK_ROOT=$(SDK_ROOT) PROJ_DIR=$(SDK_ROOT)/examples/dfu/secure_bootloader/ BOARD_FLAG=-D$(BOARD) DFU_PUBKEY_SRC=$(PUBLIC_KEY_SRC) TARGETS=nrf52832_xxaa_s332 SD=332

dfu: nrf52832_xxaa $(BIN_DIRECTORY)
	@echo Generate signed DFU zip:
	nrfutil pkg generate --hw-version 52 --sd-req 0xBA --application-version $(APP_VERSION) --application $(OUTPUT_DIRECTORY)/nrf52832_xxaa.hex --key-file $(PRIVATE_KEY) $(BIN_DIRECTORY)/sturmfront_dfu.zip

settings: nrf52832_xxaa $(BIN_DIRECTORY)
	@echo Generate settings page:
	nrfutil settings generate --family NRF52 --application $(OUTPUT_DIRECTORY)/nrf52832_xxaa.hex --softdevice $(SOFTDEVICE_S332) --application-version $(APP_VERSION) --bootloader-version $(BL_VERSION) --bl-settings-version 2 $(BIN_DIRECTORY)/settings.hex

merge: bootloader nrf52832_xxaa settings $(BIN_DIRECTORY)
	@echo Merging SD, BL, App and settings:
	mergehex --merge $(SOFTDEVICE_S332) $(BL_HEX) $(OUTPUT_DIRECTORY)/nrf52832_xxaa.hex $(BIN_DIRECTORY)/settings.hex --output $(BIN_DIRECTORY)/sturmfront.hex

clean:
	rm -rf build
