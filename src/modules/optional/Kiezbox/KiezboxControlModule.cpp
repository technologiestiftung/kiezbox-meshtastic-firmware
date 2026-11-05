#include "KiezboxControlModule.h"
#if MESHTASTIC_INCLUDE_KIEZBOXCONTROL
#include "FSCommon.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "RTC.h"
#include "Router.h"
#include "SPILock.h"
#include "UptimeClock.h"
#include "configuration.h"
#include "main.h"
#include "ve-direct.h"
#include <cmath>

// The climate sensors are read with the upstream telemetry drivers
#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && __has_include(<Adafruit_BME280.h>)
#define KB_HAS_CLIMATE_SENSORS 1
#include "detect/ScanI2CTwoWire.h"
#include "modules/Telemetry/Sensor/BME280Sensor.h"
#include "modules/Telemetry/Sensor/TelemetrySensor.h"
#include <Wire.h>
#include <forward_list>

namespace
{
std::forward_list<TelemetrySensor *> climateSensors;

template <typename T> void addClimateSensor(const ScanI2C *i2cScanner, ScanI2C::DeviceType type)
{
    ScanI2C::FoundDevice dev = i2cScanner->find(type);
    if (dev.type == ScanI2C::DeviceType::NONE) {
        return;
    }
#if WIRE_INTERFACES_COUNT > 1
    TwoWire *bus = ScanI2CTwoWire::fetchI2CBus(dev.address);
#else
    TwoWire *bus = &Wire;
#endif
    TelemetrySensor *sensor = new T();
    if (sensor->initDevice(bus, &dev)) {
        climateSensors.push_front(sensor);
    } else {
        delete sensor;
    }
}
} // namespace
#else
#define KB_HAS_CLIMATE_SENSORS 0
#endif

KiezboxControlModule::KiezboxControlModule()
    : ProtobufModule("kiezboxcontrol", static_cast<meshtastic_PortNum>(meshtastic_KiezboxMessage_PortNum_KIEZBOX_CONTROL_APP),
                     &meshtastic_KiezboxMessage_msg),
      concurrency::OSThread("KiezboxControlModule"),
      kb_config(meshtastic_KiezboxMessage_Config_init_zero),
      router_power_state(true),
      rtc_found(false)
{
    loadConfig();
    // Generic peripherals, connected regardless of dev_type
    rtc_found = rtc.begin();
    if (!rtc_found) {
        LOG_WARN("Kiezboxcontrol: no DS3231 RTC found, using the system time");
    }
    // Core specific devices
    if ( kb_config.dev_type == meshtastic_KiezboxMessage_DeviceType_core ) {
        LOG_DEBUG("INITIALIZE: Core module");
        initCore();
    }
}

void KiezboxControlModule::initCore() {
    pinMode(KB_POWER_PIN_RESET,OUTPUT);
    pinMode(KB_POWER_PIN_SET,OUTPUT);
    //TODO: This fixes a small power break on ESP bootup, we should really find a better/permanent solution
    digitalWrite(KB_POWER_PIN_RESET, HIGH);
    digitalWrite(KB_POWER_PIN_SET, HIGH);
    // TODO: check if forcing initial low is a good idea? But should be fine, as KiezboxControlModule constructor is only called once
    updateRouterPower();
}

void KiezboxControlModule::i2cScanFinished(ScanI2C *i2cScanner)
{
#if KB_HAS_CLIMATE_SENSORS
    if (moduleConfig.telemetry.environment_measurement_enabled) {
        LOG_WARN("Kiezboxcontrol: environment telemetry is enabled, it reads the same sensors as this module");
    }
    addClimateSensor<BME280Sensor>(i2cScanner, ScanI2C::DeviceType::BME_280);
    if (climateSensors.empty()) {
        LOG_INFO("Kiezboxcontrol: no climate sensor found");
    }
#endif
}

void KiezboxControlModule::readClimate(meshtastic_KiezboxMessage_CoreValues &values)
{
#if KB_HAS_CLIMATE_SENSORS
    if (climateSensors.empty()) {
        return;
    }
    meshtastic_Telemetry t = meshtastic_Telemetry_init_zero;
    t.which_variant = meshtastic_Telemetry_environment_metrics_tag;
    bool valid = false;
    for (TelemetrySensor *sensor : climateSensors) {
        // evaluate getMetrics() for every sensor, even after the first valid one
        bool sensorValid = sensor->getMetrics(&t);
        valid = valid || sensorValid;
    }
    if (!valid) {
        LOG_WARN("Kiezboxcontrol: failed to read the climate sensors");
        return;
    }
    // scaled by 1000 like the other values, the gateway divides by 1000 again
    const meshtastic_EnvironmentMetrics &e = t.variant.environment_metrics;
    if (e.has_temperature) {
        values.has_temp_in = true;
        values.temp_in = static_cast<int32_t>(lroundf(e.temperature * 1000.0f));
    }
    if (e.has_relative_humidity) {
        values.has_humid_in = true;
        values.humid_in = static_cast<int32_t>(lroundf(e.relative_humidity * 1000.0f));
    }
    if (e.has_barometric_pressure) {
        values.has_pressure = true;
        values.pressure = static_cast<int32_t>(lroundf(e.barometric_pressure * 1000.0f));
    }
#endif
}

#ifdef KB_MOCK_VALUES
// Test builds for a bare board: fills every value no sensor delivered with a made-up one that
// changes over time, so the whole chain to the database can be tested without the box hardware.
void KiezboxControlModule::mockValues(meshtastic_KiezboxMessage_CoreValues &values)
{
    const int32_t t = static_cast<int32_t>(millis() / 1000 % 600);

#define KB_MOCK(field, value)                                                                                                    \
    if (!values.has_##field) {                                                                                                   \
        values.has_##field = true;                                                                                               \
        values.field = (value);                                                                                                  \
    }
    KB_MOCK(solar_voltage, 18000 + t * 10)
    KB_MOCK(solar_power, 40000 + t * 50)
    KB_MOCK(solar_energy_day, 120000 + t * 100)
    KB_MOCK(solar_energy_total, 9870000)
    KB_MOCK(battery_voltage, 12600 + t)
    KB_MOCK(battery_current, 1500 - t)
    KB_MOCK(temp_rtc, 21000 + t * 5)
    KB_MOCK(temp_in, 23000 + t * 5)
    KB_MOCK(humid_in, 45000 + t * 10)
    KB_MOCK(pressure, 1013000 - t * 10)
#undef KB_MOCK
}
#endif

void KiezboxControlModule::updateRouterPower() {
    LOG_DEBUG("Current router power state is: %s", router_power_state ? "ON" : "OFF");
    if(router_power_state != kb_config.router_power) {
        router_power_state = kb_config.router_power;
        LOG_DEBUG("New router power state is: %s", router_power_state ? "ON" : "OFF");
    }
    setRouterPower(router_power_state);
}

void KiezboxControlModule::setRouterPower(bool state) {
    if ( kb_config.dev_type == meshtastic_KiezboxMessage_DeviceType_core ) {
        digitalWrite(KB_POWER_PIN_RESET, state ? HIGH : LOW );
        digitalWrite(KB_POWER_PIN_SET, state ? LOW : HIGH );
    } else {
        LOG_DEBUG("Rejected setting router power, as dev_type is not core! mode: %sd", kb_config.dev_type);
    }
}

void KiezboxControlModule::reboot(int32_t seconds)
{
    LOG_INFO("Reboot in %d seconds", seconds);
    if (screen)
        screen->showSimpleBanner("Rebooting...", 0);
    rebootAtMsec = (seconds < 0) ? 0 : Time::timerEndsAtMillis(seconds * 1000);
}

void KiezboxControlModule::loadConfig()
{
    if (nodeDB->loadProto(KB_CONFIG_FILE, meshtastic_KiezboxMessage_Config_size, sizeof(meshtastic_KiezboxMessage_Config),
                          &meshtastic_KiezboxMessage_Config_msg, &kb_config) != LoadFileResult::LOAD_SUCCESS) {
        kb_config = meshtastic_KiezboxMessage_Config_init_zero;
    }
}

bool KiezboxControlModule::saveConfig()
{
#ifdef FSCom
    spiLock->lock();
    FSCom.mkdir(KB_CONFIG_DIR);
    spiLock->unlock();
#endif
    return nodeDB->saveProto(KB_CONFIG_FILE, meshtastic_KiezboxMessage_Config_size, &meshtastic_KiezboxMessage_Config_msg,
                             &kb_config);
}

bool KiezboxControlModule::wantPacket(const meshtastic_MeshPacket *p)
{
    if (!ProtobufModule::wantPacket(p))
        return false;
    if (isFromUs(p))
        return true;
    // direct messages are encrypted with the node keys and arrive without the kiezbox channel
    if (p->pki_encrypted)
        return isAdminKey(p->public_key);
    return strcasecmp(channels.getName(p->channel), KB_CHANNEL_NAME) == 0;
}

bool KiezboxControlModule::isAdminKey(const meshtastic_MeshPacket_public_key_t &key)
{
    for (const auto &admin : config.security.admin_key) {
        if (admin.size == 32 && key.size == 32 && memcmp(admin.bytes, key.bytes, 32) == 0)
            return true;
    }
    LOG_DEBUG("Kiezboxcontrol: ignoring direct message, sender key isn't an admin key");
    return false;
}

bool KiezboxControlModule::isTarget(const meshtastic_KiezboxMessage_Meta &m)
{
    if (m.has_dev_type && (m.dev_type != kb_config.dev_type)) {
        LOG_DEBUG("Kiezboxcontrol: dev_type does not match");
        return false;
    }
    if (m.has_box_id && (m.box_id != kb_config.box_id)) {
        LOG_DEBUG("Kiezboxcontrol: box_id does not match");
        return false;
    }
    if (m.has_dist_id && (m.dist_id != kb_config.dist_id)) {
        LOG_DEBUG("Kiezboxcontrol: dist_id does not match");
        return false;
    }
    return true;
}

bool KiezboxControlModule::handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_KiezboxMessage *kb)
{
    if (kb == NULL) {
        LOG_DEBUG("Kiezboxcontrol: no kiezbox message to handle");
        return false;
    } else {
        LOG_DEBUG("Kiezboxcontrol: handling kiezbox message");
    }
    bool fromOthers = mp.from != 0 && mp.from != nodeDB->getNodeNum();
    // TODO: find out if this check is really needed?
    if (mp.which_payload_variant != meshtastic_MeshPacket_decoded_tag) {
        return false;
    }
    if (kb->has_request && isTarget(kb->request.meta)) {
        if (kb->request.has_type && kb->request.type == meshtastic_KiezboxMessage_Request_Type_GetConfig) {
            meshtastic_KiezboxMessage r = meshtastic_KiezboxMessage_init_default;
            r.has_config = true;
            r.config = kb_config;
            myReply = allocDataProtobuf(r);
        } else {
            LOG_DEBUG("Kiezboxcontrol: unhandled request of type %d", kb->request.type);
        }
    }
    if (kb->has_control) {
        LOG_DEBUG("Kiezboxcontrol: handling kiezbox control message");
        meshtastic_KiezboxMessage_Control &c = kb->control;
        if (isTarget(c.meta)) {
            LOG_DEBUG("Kiezboxcontrol: accepted control message of type %d",c.which_set);
            switch(c.which_set) {
                case meshtastic_KiezboxMessage_Control_unix_time_tag:
                    // Only accept time updates from local/serial connection
                    //TODO: check if it is a good idea to accept time updates fromt the LoRa mesh? (better than nothing?)
                    if (fromOthers) {
                        LOG_DEBUG("Kiezboxcontrol: ignoring remote update of type %d",c.which_set);
                    } else if (rtc_found) {
                        rtc.adjust(DateTime(c.set.unix_time));
                    } else {
                        struct timeval tv = {.tv_sec = static_cast<time_t>(c.set.unix_time), .tv_usec = 0};
                        if (perhapsSetRTC(RTCQualityNTP, &tv, true) != RTCSetResultSuccess) {
                            LOG_WARN("Kiezboxcontrol: can't set the system time");
                        }
                    }
                    break;
                case meshtastic_KiezboxMessage_Control_mode_tag:
                    kb_config.mode = c.set.mode;
                    saveConfig();
                    break;
                case meshtastic_KiezboxMessage_Control_router_power_tag:
                    kb_config.router_power = c.set.router_power;
                    // apply power update immediately
                    saveConfig();
                    updateRouterPower();
                    break;
                case meshtastic_KiezboxMessage_Control_status_interval_tag:
                    kb_config.status_interval = c.set.status_interval;
                    saveConfig();
                    // will be applied at next runOnce loop
                    break;
                case meshtastic_KiezboxMessage_Control_box_id_tag:
                    if (!fromOthers) {
                        kb_config.box_id = c.set.box_id;
                        saveConfig();
                    } else {
                        LOG_DEBUG("Kiezboxcontrol: ignoring remote update of type %d",c.which_set);
                    }
                    break;
                case meshtastic_KiezboxMessage_Control_dist_id_tag:
                    if (!fromOthers) {
                        kb_config.dist_id = c.set.dist_id;
                        saveConfig();
                    } else {
                        LOG_DEBUG("Kiezboxcontrol: ignoring remote update of type %d",c.which_set);
                    }
                    break;
                case meshtastic_KiezboxMessage_Control_dev_type_tag:
                    if (!fromOthers) {
                        kb_config.dev_type = c.set.dev_type;
                        saveConfig();
                        // rebooting as device type changes requires proper initialization
                        reboot(3);
                    } else {
                        LOG_DEBUG("Kiezboxcontrol: ignoring remote update of type %d",c.which_set);
                    }
                    break;
                case meshtastic_KiezboxMessage_Control_enabled_tag:
                    kb_config.enabled = c.set.enabled;
                    saveConfig();
                    // will be applied at next runOnce loop
                    break;
                case meshtastic_KiezboxMessage_Control_button_id_tag:
                    if (!fromOthers) {
                        kb_config.button_id = c.set.button_id;
                        saveConfig();
                    } else {
                        LOG_DEBUG("Kiezboxcontrol: ignoring remote update of type %d",c.which_set);
                    }
                    break;
                case meshtastic_KiezboxMessage_Control_config_tag:
                    if (!fromOthers) {
                        bool dev_type_changed = c.set.config.dev_type != kb_config.dev_type;
                        kb_config = c.set.config;
                        saveConfig();
                        if (dev_type_changed) {
                            // rebooting as device type changes requires proper initialization
                            reboot(3);
                        } else {
                            updateRouterPower();
                        }
                    } else {
                        LOG_DEBUG("Kiezboxcontrol: ignoring remote update of type %d",c.which_set);
                    }
                    break;
                default:
                    LOG_DEBUG("unhandled kiezbox control message of type %d",c.which_set);
                    break;
            }
        }
    }
    // let the following modules run, so the message is still forwarded to the client
    return false;
}

int32_t KiezboxControlModule::runOnce()
{
    // Update router power if it should be changed
    // TODO: maybe change this to be immediate on setting change?
    // Broadcast sensor values
    if (kb_config.enabled) {
        LOG_DEBUG("Module enabled");
        LOG_DEBUG("Operation mode: %d",kb_config.dev_type);
        meshtastic_KiezboxMessage r = meshtastic_KiezboxMessage_init_default;
        if ( kb_config.dev_type == meshtastic_KiezboxMessage_DeviceType_core ) {
            r.has_update = true;
            r.update.unix_time = rtc_found ? rtc.now().unixtime() : getTime();
            r.update.has_meta = true;
            r.update.meta.has_box_id = true;
            r.update.meta.box_id = kb_config.box_id;
            r.update.meta.has_dist_id = true;
            r.update.meta.dist_id = kb_config.dist_id;
            r.update.meta.has_dev_type = true;
            r.update.meta.dev_type = kb_config.dev_type;
        }
        if ( kb_config.dev_type == meshtastic_KiezboxMessage_DeviceType_core ) {
            updateRouterPower();
            r.update.has_core = true;
            r.update.core.mode = kb_config.mode;
            r.update.core.has_values = true;
            // mppt measurements
            if(vedirect.get_value(ve::id::panel_voltage ,r.update.core.values.solar_voltage)){
                r.update.core.values.solar_voltage *= 10;
                r.update.core.values.has_solar_voltage = true;
            }
            if(vedirect.get_value(ve::id::panel_power ,r.update.core.values.solar_power)){
                r.update.core.values.solar_power *= 10;
                r.update.core.values.has_solar_power = true;
            }
            if(vedirect.get_value(ve::id::yield_today ,r.update.core.values.solar_energy_day)){
                r.update.core.values.solar_energy_day *= 10;
                r.update.core.values.has_solar_energy_day = true;
            }
            if(vedirect.get_value(ve::id::yield_system ,r.update.core.values.solar_energy_total)){
                r.update.core.values.solar_energy_total *= 10;
                r.update.core.values.has_solar_energy_total = true;
            }
            //TODO: check if charger_current and voltage are really mapped to battery_current and voltage from ve.direct text protocol
            if(vedirect.get_value(ve::id::charger_voltage ,r.update.core.values.battery_voltage)){
                r.update.core.values.battery_voltage *= 10;
                r.update.core.values.has_battery_voltage = true;
            }
            //TODO: check not 3 from the docs about battery current and load current
            if(vedirect.get_value(ve::id::charger_current ,r.update.core.values.battery_current)){
                r.update.core.values.battery_current *= 100;
                r.update.core.values.has_battery_current = true;
            }
            // Checking router power state by reading pin state
            r.update.core.has_router = true;
            r.update.core.router.powered = router_power_state;
            // RTC Time and Temperature
            if (rtc_found) {
                r.update.core.values.has_temp_rtc = true;
                r.update.core.values.temp_rtc = static_cast<int32_t>(rtc.getTemperature() * 1000.0);
            }
            // Temperature, humidity and pressure inside the box
            readClimate(r.update.core.values);
#ifdef KB_MOCK_VALUES
            mockValues(r.update.core.values);
#endif
        }
        if ( kb_config.dev_type == meshtastic_KiezboxMessage_DeviceType_core ) {
            meshtastic_MeshPacket *p = allocDataProtobuf(r);
            meshtastic_Channel &sendChannel = channels.getByName(KB_CHANNEL_NAME);
            if ( strncmp(sendChannel.settings.name,KB_CHANNEL_NAME,12) == 0 ) {
                p->channel = sendChannel.index;
                LOG_DEBUG("Broadcasting Kiezbox Message (to channel %d)", p->channel);
                service->sendToMesh(p, RX_SRC_LOCAL, true);
            } else {
                LOG_DEBUG("There seems to be no channel named \"kiezbox\" (on index %d). skip sending.", p->channel);
                service->releaseToPool(p);
            }
        }
    }
    // Wait before next status update use KB_STATUS_MIN as default and capped by KB_STATUS_MAX
    // TODO: maybe sync this with rtc somehow, to have all updates in sync? but this could also lead to more lora collisions?
    return std::min<int32_t>(std::max<int32_t>(KB_STATUS_MIN, kb_config.status_interval), KB_STATUS_MAX);
}
#endif
