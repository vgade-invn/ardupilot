LOCAL_PATH := $(call my-dir)
###############################################################################
# ArduCopter
###############################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := arducopter
LOCAL_MODULE_FILENAME := arducopter
LOCAL_DESCRIPTION := ArduCopter is an open source autopilot
LOCAL_CATEGORY_PATH := apm

$(call load-config)

ARDUCOPTER_BUILD_DIR := $(call local-get-build-dir)
ARDUCOPTER_TOOLCHAIN_BIN_DIR := $(dir $(TARGET_CC_PATH))
ARDUCOPTER_FILENAME := $(LOCAL_MODULE_FILENAME)

# forward versosity to Ardupilot Makefile
ifneq ("$(V)","0")
	ARDUCOPTER_VERBOSE=1
else
	ARDUCOPTER_VERBOSE=
endif

# always enter in arducopter makefile
$(ARDUCOPTER_BUILD_DIR)/$(ARDUCOPTER_FILENAME): .FORCE
$(ARDUCOPTER_BUILD_DIR)/$(ARDUCOPTER_FILENAME):
	@mkdir -p $(ARDUCOPTER_BUILD_DIR)
	$(Q) PATH=$(ARDUCOPTER_TOOLCHAIN_BIN_DIR):$(PATH) BUILDROOT=$(ARDUCOPTER_BUILD_DIR) \
		$(MAKE) -C $(PRIVATE_PATH)/ArduCopter VERBOSE=$(ARDUCOPTER_VERBOSE) bebop
	$(Q) cp -af $(ARDUCOPTER_BUILD_DIR)/ArduCopter.elf $(ARDUCOPTER_BUILD_DIR)/$(ARDUCOPTER_FILENAME)
	@mkdir -p $(TARGET_OUT_STAGING)/usr/bin/
	$(Q) cp -af $(ARDUCOPTER_BUILD_DIR)/$(ARDUCOPTER_FILENAME) $(TARGET_OUT_STAGING)/usr/bin/$(ARDUCOPTER_FILENAME)

.PHONY: $(LOCAL_MODULE)-clean
$(LOCAL_MODULE)-clean:
	$(Q) PATH=$(ARDUCOPTER_TOOLCHAIN_BIN_DIR):$(PATH) BUILDROOT=$(ARDUCOPTER_BUILD_DIR) \
		$(MAKE) -C $(PRIVATE_PATH)/ArduCopter VERBOSE=$(ARDUCOPTER_VERBOSE) clean
	$(Q) rm -f $(ARDUCOPTER_BUILD_DIR)/$(ARDUCOPTER_FILENAME)
	$(Q) rm -f $(TARGET_OUT_STAGING)/usr/bin/$(ARDUCOPTER_FILENAME)

LOCAL_CONFIG_FILES := Config.in
LOCAL_COPY_FILES = 50-arducopter.rc:etc/boxinit.d/
ifeq ($(CONFIG_ARDUPILOT_MILOS),y)
LOCAL_COPY_FILES += Tools/Frame_params/Parrot_Bebop2.param:etc/arducopter/bebop.parm
else
LOCAL_COPY_FILES += Tools/Frame_params/Parrot_Bebop.param:etc/arducopter/bebop.parm
endif

include $(BUILD_CUSTOM)

###############################################################################
# APM:Plane, for disco
###############################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := apm-plane-disco
LOCAL_MODULE_FILENAME := apm-plane-disco
LOCAL_DESCRIPTION := APM:Plane is an open source autopilot
LOCAL_CATEGORY_PATH := apm

# subdirectories of libraries, containing sources
APM_PLANE_DISCO_SRC_LIBRARY_DIRS := \
	AC_AttitudeControl \
	AC_PID \
	AC_Avoidance \
	AC_Fence \
	AC_WPNav \
	AP_AccelCal \
	AP_ADC \
	AP_ADSB \
	AP_AHRS \
	AP_Airspeed \
	AP_Arming \
	AP_Baro \
	AP_BattMonitor \
	AP_BoardConfig \
	AP_Camera \
	AP_Common \
	AP_Compass \
	AP_Declination \
	AP_Frsky_Telem \
	AP_GPS \
	AP_HAL \
	AP_HAL_Empty \
	AP_HAL_Linux \
	AP_HAL/utility \
	AP_InertialNav \
	AP_InertialSensor \
	AP_L1_Control \
	AP_Math \
	APM_Control \
	AP_Menu \
	AP_Mission \
	APM_OBC \
	AP_Motors \
	AP_Mount \
	AP_NavEKF \
	AP_NavEKF2 \
	AP_Notify \
	AP_OpticalFlow \
	AP_Parachute \
	AP_Param \
	AP_Rally \
	AP_RangeFinder \
	AP_RCMapper \
	AP_Relay \
	AP_RPM \
	AP_RSSI \
	AP_Scheduler \
	AP_SerialManager \
	AP_ServoRelayEvents \
	AP_TECS \
	AP_Terrain \
	AP_Tuning \
	DataFlash \
	Filter \
	GCS_MAVLink \
	RC_Channel \
	AP_Module \
	StorageManager

APM_PLANE_DISCO_SRC_DIRS := \
	ArduPlane \
	$(addprefix libraries/,$(APM_PLANE_DISCO_SRC_LIBRARY_DIRS))

LOCAL_SRC_FILES := \
	$(foreach dir,$(APM_PLANE_DISCO_SRC_DIRS),$(call all-cpp-files-in,$(dir)))

LOCAL_CXXFLAGS := \
	-D_GNU_SOURCE \
	-DF_CPU= \
	-DSKETCH=\"ArduPlane\" \
	-DSKETCHNAME="\"ArduPlane\"" \
	-DSKETCHBOOK="\"$(LOCAL_PATH)\"" \
	-DAPM_BUILD_DIRECTORY=APM_BUILD_ArduPlane \
	-DGIT_VERSION=\"$(shell cd $(LOCAL_PATH); git describe --always --dirty)\" \
	-DMAVLINK_PROTOCOL_VERSION=2 \
	-DHAVE_CMATH_ISFINITE \
	-DNEED_CMATH_ISFINITE_STD_NAMESPACE \
	-DCONFIG_HAL_BOARD=HAL_BOARD_LINUX \
	-DCONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_LINUX_DISCO \
	-std=gnu++11 \
	-Wno-unused-parameter \
	-Wno-unknown-pragmas \
	-Wno-missing-field-initializers \
	-Wno-reorder \
	-Wno-overloaded-virtual \
	-Wno-unknown-warning-option \
	-ffunction-sections \
	-fdata-sections \
	-fno-exceptions \
	-fsigned-char

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/libraries/AP_Common/missing \
	$(LOCAL_PATH)/libraries/

LOCAL_LIBRARIES := \
	apm-mavlink-ardupilotmega \
	libiio

LOCAL_COPY_FILES = 50-apm-plane-disco.rc:etc/boxinit.d/

include $(BUILD_EXECUTABLE)

