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
      dallas(&onewire)
{
    // restrict to the gpio channel for rx
    boundChannel = Channels::kiezboxChannel;
    dht.begin();
}

bool KiezboxControlModule::handleReceivedProtobuf(const meshtastic_MeshPacket &req, meshtastic_KiezboxMessage *pptr)
{
    return false;
}

int32_t KiezboxControlModule::runOnce()
{
    // Broadcast sensor values
    if (moduleConfig.kiezbox_control.enabled) {
        LOG_DEBUG("Broadcasting Kiezbox Message\n");
        meshtastic_KiezboxMessage r = meshtastic_KiezboxMessage_init_default;
        r.has_status = true;
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
        meshtastic_MeshPacket *p = allocDataProtobuf(r);
        service->sendToMesh(p);
    }
    // Wait before next status update use KB_STATUS_MIN as default and capped by KB_STATUS_MAX
    // TODO: maybe synt this with rtc somehow?
    return std::min(std::max(KB_STATUS_MIN,moduleConfig.kiezbox_control.status_interval),KB_STATUS_MAX);
}
