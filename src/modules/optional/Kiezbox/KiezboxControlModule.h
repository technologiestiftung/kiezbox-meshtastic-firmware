#pragma once
#include "configuration.h"
#if MESHTASTIC_INCLUDE_KIEZBOXCONTROL
#include "ProtobufModule.h"
#include "concurrency/OSThread.h"
#include "mesh/generated/meshtastic/kiezbox_control.pb.h"

// Sensor stuff
#include <OneWire.h>
#include <DallasTemperature.h>

// MPPT
#include "ve-direct.h"

// RTC
#include "RTClib.h"

// Dust sensor
#include "SdsDustSensor.h"

// BME 680
#include <Adafruit_BME680.h>

#ifndef KB_CHANNEL_NAME
#define KB_CHANNEL_NAME "kiezbox"
#endif
// kept outside of /prefs, so a factory reset doesn't remove it
#ifndef KB_CONFIG_DIR
#define KB_CONFIG_DIR "/kiezbox"
#endif
#define KB_CONFIG_FILE KB_CONFIG_DIR "/config.proto"

enum class sens_state_t {
    sds_bootup = 0,
    sds_done = 1,
    sds_warmup = 2,
};

/**
 * A module that reports kiezbox sensor values and controls the kiezbox hardware.
 */
class KiezboxControlModule : public ProtobufModule<meshtastic_KiezboxMessage>, private concurrency::OSThread
{
    meshtastic_KiezboxMessage_Config kb_config;
    OneWire onewire;
    DallasTemperature dallas;
    bool router_power_state;
    ve::VEDirect vedirect;
    // NOTE: RTClib uses the default i2c from the Wire library.
    // Pins are defined by I2C_SCL and I2C_SDA and 42 and 41 for the heltec v3
    RTC_DS3231 rtc;
    sens_state_t sens_state;
    SdsDustSensor sds;
    Adafruit_BME680 bme680;

  public:
    /** Constructor
     * name is for debugging output
     */
    KiezboxControlModule();

  protected:
    /** Called to handle a particular incoming message

    @return true if you've guaranteed you've handled this message and no other handlers should be considered for it
    */
    virtual bool handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_KiezboxMessage *p) override;

    /**
     * Periodically read the gpios we have been asked to WATCH, if they have changed,
     * broadcast a message with the change information.
     *
     * The method that will be called each time our thread gets a chance to run
     *
     * Returns desired period for next invocation (or RUN_SAME for no change)
     */
    virtual int32_t runOnce() override;
    void updateRouterPower();
    void setRouterPower(bool state);
    void initCore();
    void initSensor();
    void reboot(int32_t seconds);
    void loadConfig();
    bool saveConfig();
    bool isTarget(const meshtastic_KiezboxMessage_Meta &m);
};

extern KiezboxControlModule kiezboxControlModule;
#endif