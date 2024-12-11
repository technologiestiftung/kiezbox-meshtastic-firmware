#include "KiezboxControlModule.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "RTC.h"
#include "Router.h"
#include "configuration.h"
#include "main.h"

KiezboxControlModule::KiezboxControlModule()
    : ProtobufModule("kiezboxcontrol", meshtastic_PortNum_KIEZBOX_CONTROL_APP, &meshtastic_KiezboxMessage_msg),
      concurrency::OSThread("KiezboxControlModule"),
      dht(KB_DHTPIN, KB_DHTTYPE),
      onewire(KB_ONEWIRE_PIN),
      dallas(&onewire),
      router_power_state(false),
      sens_state(sens_state_t::sds_bootup),
      sds(Serial2)
{
    // restrict to the gpio channel for rx
    boundChannel = Channels::kiezboxChannel;
    rtc.begin();
    if ( moduleConfig.kiezbox_control.dev_type == meshtastic_KiezboxMessage_DeviceType_core ) {
        LOG_DEBUG("INITIALIZE: Core module\n");
        dht.begin();
        pinMode(KB_POWER_PIN_DEFAULT,OUTPUT);
        // TODO: check if forcing initial low is a good idea? But should be fine, as KiezboxControlModule constructor is only called once
        digitalWrite(KB_POWER_PIN_DEFAULT, router_power_state ? HIGH : LOW );
    }
    if ( moduleConfig.kiezbox_control.dev_type == meshtastic_KiezboxMessage_DeviceType_sensor ) {
        LOG_DEBUG("INITIALIZE: Sensor module\n");
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
}

bool KiezboxControlModule::handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_KiezboxMessage *kb)
{
    assert(kb);
    bool fromOthers = mp.from != 0 && mp.from != nodeDB->getNodeNum();
    if (mp.which_payload_variant != meshtastic_MeshPacket_decoded_tag) {
        return false;
    }
    // Currently only handle messages recieved locally
    if (!fromOthers) {
        if (kb->has_control) {
            switch(kb->control.which_set) {
                case meshtastic_KiezboxMessage_Control_unix_time_tag:
                    rtc.adjust(DateTime(kb->control.set.unix_time));
                    break;
                default:
                    return false;
            }
        } else {
            return false;
        }
    }
    // we handle Kiezbox Messages, so always return true
    return true;
}

int32_t KiezboxControlModule::runOnce()
{
    // Update router power if it should be changed
    // TODO: maybe change this to be immediate on setting change?
    // Broadcast sensor values
    if (moduleConfig.kiezbox_control.enabled) {
        LOG_DEBUG("Module enabled\n");
        LOG_DEBUG("Operation mode: %d\n",moduleConfig.kiezbox_control.dev_type);
        meshtastic_KiezboxMessage r = meshtastic_KiezboxMessage_init_default;
        if ( moduleConfig.kiezbox_control.dev_type == meshtastic_KiezboxMessage_DeviceType_core ||
             moduleConfig.kiezbox_control.dev_type == meshtastic_KiezboxMessage_DeviceType_sensor ) {
            r.has_update = true;
            r.update.unix_time = rtc.now().unixtime();
            r.update.has_meta = true;
            r.update.meta.has_box_id = true;
            r.update.meta.box_id = moduleConfig.kiezbox_control.box_id;
            r.update.meta.has_dist_id = true;
            r.update.meta.dist_id = moduleConfig.kiezbox_control.dist_id;
            r.update.meta.has_dev_type = true;
            r.update.meta.dev_type = moduleConfig.kiezbox_control.dev_type;
        }
        if ( moduleConfig.kiezbox_control.dev_type == meshtastic_KiezboxMessage_DeviceType_core ) {
            if(router_power_state != moduleConfig.kiezbox_control.router_power) {
                LOG_DEBUG("Changing router power state to: %s\n", router_power_state ? "ON" : "OFF");
                router_power_state = moduleConfig.kiezbox_control.router_power;
                digitalWrite(KB_POWER_PIN_DEFAULT, router_power_state ? HIGH : LOW );
            }
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
            // TODO: and maybe convert to hex protocol to recude delay and ressource usage
            // Checking router power state by reading pin state
            r.update.core.has_router = true;
            r.update.core.router.powered = digitalRead(KB_POWER_PIN_DEFAULT);
            // RTC Time and Temperature
            r.update.core.values.has_temp_rtc = true;
            r.update.core.values.temp_rtc = static_cast<int32_t>(rtc.getTemperature() * 1000.0);
        }
        if ( moduleConfig.kiezbox_control.dev_type == meshtastic_KiezboxMessage_DeviceType_sensor ) {
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
                    r.update.meta.sens_id = moduleConfig.kiezbox_control.sens_id;
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
        if ( moduleConfig.kiezbox_control.dev_type == meshtastic_KiezboxMessage_DeviceType_core ||
             ( moduleConfig.kiezbox_control.dev_type == meshtastic_KiezboxMessage_DeviceType_sensor && sens_state == sens_state_t::sds_done ) ) {
            meshtastic_MeshPacket *p = allocDataProtobuf(r);
            LOG_DEBUG("Broadcasting Kiezbox Message\n");
            service->sendToMesh(p, RX_SRC_LOCAL, true);
        }
        if ( moduleConfig.kiezbox_control.dev_type == meshtastic_KiezboxMessage_DeviceType_sensor && sens_state == sens_state_t::sds_bootup ) {
            sens_state = sens_state_t::sds_done;
        }
    }
    // Wait before next status update use KB_STATUS_MIN as default and capped by KB_STATUS_MAX
    // TODO: maybe synt this with rtc somehow?
    // LOG_DEBUG("VEDirect debug\n");
    // ve::VEMessage msg;
    // msg.msg_generate();
    // LOG_DEBUG("VEMessage debug: %s\n", msg.get_hex_command().c_str());
    // vedirect.send(msg);
    // vedirect.debug();
    return std::min(std::max(KB_STATUS_MIN,moduleConfig.kiezbox_control.status_interval),KB_STATUS_MAX);
}

