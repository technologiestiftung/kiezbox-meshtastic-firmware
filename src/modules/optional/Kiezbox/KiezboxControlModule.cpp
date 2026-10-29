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

KiezboxControlModule::KiezboxControlModule()
    : ProtobufModule("kiezboxcontrol", static_cast<meshtastic_PortNum>(meshtastic_KiezboxMessage_PortNum_KIEZBOX_CONTROL_APP),
                     &meshtastic_KiezboxMessage_msg),
      concurrency::OSThread("KiezboxControlModule"),
      kb_config(meshtastic_KiezboxMessage_Config_init_zero),
      // dht has to be constructed here, but the KB_DHTPIN isn't actually used/initialized
      // until dht.begin() so it is fine to reuse
      dht(KB_DHTPIN, KB_DHTTYPE),
      onewire(),
      dallas(),
      router_power_state(true),
      sens_state(sens_state_t::sds_bootup),
      // sds has to be constructed here, but Serial2 isn't actually used/initialized
      // until initSernsor() is called so KB_DUST_RXPIN and KB_DUST_TXPIN are fine to reuse
      sds(Serial2)
{
    // restrict to the gpio channel for rx
    boundChannel = KB_CHANNEL_NAME;
    loadConfig();
    // Generic peripherals, connected regardless of dev_type
    rtc.begin();
    // Core specific devices
    if ( kb_config.dev_type == meshtastic_KiezboxMessage_DeviceType_core ) {
        LOG_DEBUG("INITIALIZE: Core module\n");
        initCore();
    }
    if ( kb_config.dev_type == meshtastic_KiezboxMessage_DeviceType_sensor ) {
        LOG_DEBUG("INITIALIZE: Sensor module\n");
        initSensor();
    }
}

void KiezboxControlModule::initCore() {
    dht.begin();
    onewire.begin(KB_ONEWIRE_PIN);
    dallas.setOneWire(&onewire);
    pinMode(KB_POWER_PIN_RESET,OUTPUT);
    pinMode(KB_POWER_PIN_SET,OUTPUT);
    //TODO: This fixes a small power break on ESP bootup, we should really find a better/permanent solution
    digitalWrite(KB_POWER_PIN_RESET, HIGH);
    digitalWrite(KB_POWER_PIN_SET, HIGH);
    // TODO: check if forcing initial low is a good idea? But should be fine, as KiezboxControlModule constructor is only called once
    updateRouterPower();
}

void KiezboxControlModule::initSensor() {
    Serial2.begin(KB_DUST_BAUD, SERIAL_8N1, KB_DUST_RXPIN, KB_DUST_TXPIN);
    if (!bme680.begin()) {
        LOG_DEBUG("Failed to initialize BME680\n");
    }
    // Set up oversampling and filter initialization
    // TODO: review parameter settings
    bme680.setTemperatureOversampling(BME680_OS_8X);
    bme680.setHumidityOversampling(BME680_OS_2X);
    bme680.setPressureOversampling(BME680_OS_4X);
    bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme680.setGasHeater(320, 150); // 320*C for 150 ms
}

void KiezboxControlModule::updateRouterPower() {
    LOG_DEBUG("Current router power state is: %s\n", router_power_state ? "ON" : "OFF");
    if(router_power_state != kb_config.router_power) {
        router_power_state = kb_config.router_power;
        LOG_DEBUG("New router power state is: %s\n", router_power_state ? "ON" : "OFF");
    }
    setRouterPower(router_power_state);
}

void KiezboxControlModule::setRouterPower(bool state) {
    if ( kb_config.dev_type == meshtastic_KiezboxMessage_DeviceType_core ) {
        digitalWrite(KB_POWER_PIN_RESET, state ? HIGH : LOW );
        digitalWrite(KB_POWER_PIN_SET, state ? LOW : HIGH );
    } else {
        LOG_DEBUG("Rejected setting router power, as dev_type is not core! mode: %sd\n", kb_config.dev_type);
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
    if (m.has_sens_id && (m.sens_id != kb_config.sens_id)) {
        LOG_DEBUG("Kiezboxcontrol: sens_id does not match");
        return false;
    }
    return true;
}

bool KiezboxControlModule::handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_KiezboxMessage *kb)
{
    if (kb == NULL) {
        LOG_DEBUG("Kiezboxcontrol: no kiezbox message to handle");
        return true;
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
                    if (!fromOthers) {
                        rtc.adjust(DateTime(c.set.unix_time));
                    } else {
                        LOG_DEBUG("Kiezboxcontrol: ignoring remote update of type %d",c.which_set);
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
                case meshtastic_KiezboxMessage_Control_sds_warmup_time_tag:
                    kb_config.sds_warmup_time = c.set.sds_warmup_time;
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
                case meshtastic_KiezboxMessage_Control_sens_id_tag:
                    if (!fromOthers) {
                        kb_config.sens_id = c.set.sens_id;
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
                    LOG_DEBUG("unhandled kiezbox control message of type %d\n",c.which_set);
                    return true;
            }
        }
    }
    // we handled the kiezbox messages successfully, so we stop message parsing by returning true
    return true;
}

int32_t KiezboxControlModule::runOnce()
{
    // Update router power if it should be changed
    // TODO: maybe change this to be immediate on setting change?
    // Broadcast sensor values
    if (kb_config.enabled) {
        LOG_DEBUG("Module enabled\n");
        LOG_DEBUG("Operation mode: %d\n",kb_config.dev_type);
        meshtastic_KiezboxMessage r = meshtastic_KiezboxMessage_init_default;
        if ( kb_config.dev_type == meshtastic_KiezboxMessage_DeviceType_core ||
             kb_config.dev_type == meshtastic_KiezboxMessage_DeviceType_sensor ) {
            r.has_update = true;
            r.update.unix_time = rtc.now().unixtime();
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
            // Internal sensors
            r.update.has_core = true;
            r.update.core.has_values = true;
            r.update.core.values.has_temp_in = true;
            r.update.core.values.temp_in = static_cast<int32_t>(dht.readTemperature() * 1000.0);
            r.update.core.values.has_humid_in = true;
            r.update.core.values.humid_in = static_cast<int32_t>(dht.readHumidity() * 1000.0);
            // external sensors
            dallas.requestTemperatures(); 
            r.update.core.values.has_temp_out = true;
            r.update.core.values.temp_out = static_cast<int32_t>(dallas.getTempCByIndex(0) * 1000.0);
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
            r.update.core.values.has_temp_rtc = true;
            r.update.core.values.temp_rtc = static_cast<int32_t>(rtc.getTemperature() * 1000.0);
        }
        if ( kb_config.dev_type == meshtastic_KiezboxMessage_DeviceType_sensor ) {
            switch (sens_state) {
                case sens_state_t::sds_bootup:
                    LOG_DEBUG("Sensor bootup. nothing to do ...\n");
                    break;
                case sens_state_t::sds_done:
                    LOG_DEBUG("Wakeup sds sensor.\n");
                    sds.wakeup();
                    sens_state = sens_state_t::sds_warmup;
                    return 30000; //TODO: set from module config
                    break;
                case sens_state_t::sds_warmup:
                    bme680.beginReading();
                    PmResult pm = sds.queryPm();
                    r.update.meta.has_sens_id = true;
                    r.update.meta.sens_id = kb_config.sens_id;
                    r.update.has_sensor = true;
                    r.update.sensor.has_values = true;
                    r.update.sensor.values.has_part_pm25 = true;
                    r.update.sensor.values.part_pm25 = static_cast<int32_t>(pm.pm25 * 1000.0);
                    r.update.sensor.values.has_part_pm10 = true;
                    r.update.sensor.values.part_pm10 = static_cast<int32_t>(pm.pm10 * 1000.0);
                    WorkingStateResult state = sds.sleep();
                    if (state.isWorking()) {
                        LOG_DEBUG("sds sensor sleep failed.\n");
                    } else {
                        LOG_DEBUG("sds sensor is sleeping now.\n");
                    }
                    // RTC Time and Temperature
                    r.update.sensor.values.has_temp_rtc = true;
                    r.update.sensor.values.temp_rtc = static_cast<int32_t>(rtc.getTemperature() * 1000.0);
                    // Battery voltage
                    r.update.sensor.values.has_battery_voltage = true;
                    r.update.sensor.values.battery_voltage = static_cast<int32_t>(((analogReadMilliVolts(KB_BAT_PIN) * 1507)/1000));
                    if (!bme680.endReading()) {
                        LOG_DEBUG("bme680 failed to read.\n");
                    } else {
                        r.update.sensor.values.has_temp_main = true;
                        r.update.sensor.values.temp_main = static_cast<int32_t>(bme680.temperature * 1000.0);
                        r.update.sensor.values.has_humid_main = true;
                        r.update.sensor.values.humid_main = static_cast<int32_t>(bme680.humidity * 1000.0);
                        r.update.sensor.values.has_pressure = true;
                        r.update.sensor.values.pressure = static_cast<int32_t>(bme680.pressure * 10.0);
                        r.update.sensor.values.has_air_quality = true;
                        r.update.sensor.values.air_quality = static_cast<int32_t>(bme680.gas_resistance * 1.0);
                    }
                    sens_state = sens_state_t::sds_done;
                    break;
            }
        }
        if ( kb_config.dev_type == meshtastic_KiezboxMessage_DeviceType_core ||
             ( kb_config.dev_type == meshtastic_KiezboxMessage_DeviceType_sensor && sens_state == sens_state_t::sds_done ) ) {
            meshtastic_MeshPacket *p = allocDataProtobuf(r);
            meshtastic_Channel &sendChannel = channels.getByName(KB_CHANNEL_NAME);
            if ( strncmp(sendChannel.settings.name,KB_CHANNEL_NAME,12) == 0 ) {
                p->channel = sendChannel.index;
                LOG_DEBUG("Broadcasting Kiezbox Message (to channel %d)\n", p->channel);
                service->sendToMesh(p, RX_SRC_LOCAL, true);
            } else {
                LOG_DEBUG("There seems to be no channel named \"kiezbox\" (on index %d). skip sending.\n", p->channel);
                service->releaseToPool(p);
            }
        }
        if ( kb_config.dev_type == meshtastic_KiezboxMessage_DeviceType_sensor && sens_state == sens_state_t::sds_bootup ) {
            sens_state = sens_state_t::sds_done;
        }
    }
    // Wait before next status update use KB_STATUS_MIN as default and capped by KB_STATUS_MAX
    // TODO: maybe sync this with rtc somehow, to have all updates in sync? but this could also lead to more lora collisions?
    return std::min<int32_t>(std::max<int32_t>(KB_STATUS_MIN, kb_config.status_interval), KB_STATUS_MAX);
}
#endif
