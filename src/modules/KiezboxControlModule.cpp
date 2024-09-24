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
      router_power_state(false)
{
    // restrict to the gpio channel for rx
    boundChannel = Channels::kiezboxChannel;
    dht.begin();
    pinMode(KB_POWER_PIN_DEFAULT,OUTPUT);
    // TODO: check if forcing initial low is a good idea? But should be fine, as KiezboxControlModule constructor is only called once
    digitalWrite(KB_POWER_PIN_DEFAULT, router_power_state ? HIGH : LOW );
}

bool KiezboxControlModule::handleReceivedProtobuf(const meshtastic_MeshPacket &req, meshtastic_KiezboxMessage *pptr)
{
    return false;
}

int32_t KiezboxControlModule::runOnce()
{
    // Update router power if it should be changed
    // TODO: maybe change this to be immediate on setting change?
    if(router_power_state != moduleConfig.kiezbox_control.router_power) {
        LOG_DEBUG("Changing router power state to: %s\n", router_power_state ? "ON" : "OFF");
        router_power_state = moduleConfig.kiezbox_control.router_power;
        digitalWrite(KB_POWER_PIN_DEFAULT, router_power_state ? HIGH : LOW );
    }
    // Broadcast sensor values
    if (moduleConfig.kiezbox_control.enabled) {
        LOG_DEBUG("Broadcasting Kiezbox Message\n");
        meshtastic_KiezboxMessage r = meshtastic_KiezboxMessage_init_default;
        r.has_status = true;
        r.status.box_id = moduleConfig.kiezbox_control.box_id;
        r.status.dist_id = moduleConfig.kiezbox_control.dist_id;
        // Internal sensors
        r.status.temperature_in = static_cast<int32_t>(dht.readTemperature() * 1000.0);
        r.status.humidity_in = static_cast<int32_t>(dht.readHumidity() * 1000.0);
        // external sensors
        dallas.requestTemperatures(); 
        r.status.temperature_out = static_cast<int32_t>(dallas.getTempCByIndex(0) * 1000.0);
        // mppt measurements
        // TODO: and maybe convert to hex protocol to recude delay and ressource usage
        // RTC
        // TODO: add support and time setting handling
        // Checking router power state by reading pin state
        r.status.router_powered = digitalRead(KB_POWER_PIN_DEFAULT);
        meshtastic_MeshPacket *p = allocDataProtobuf(r);
        service->sendToMesh(p);
    }
    // Wait before next status update use KB_STATUS_MIN as default and capped by KB_STATUS_MAX
    // TODO: maybe synt this with rtc somehow?
    return std::min(std::max(KB_STATUS_MIN,moduleConfig.kiezbox_control.status_interval),KB_STATUS_MAX);
}
