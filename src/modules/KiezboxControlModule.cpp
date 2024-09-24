#include "KiezboxControlModule.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "RTC.h"
#include "Router.h"
#include "configuration.h"
#include "main.h"

#define NUM_GPIOS 64

// Because (FIXME) we currently don't tell API clients status on sent messages
// we need to throttle our sending, so that if a gpio is bouncing up and down we
// don't generate more messages than the net can send. So we limit watch messages to
// a max of one change per 30 seconds
#define WATCH_INTERVAL_MSEC (30 * 1000)

/// Set pin modes for every set bit in a mask
static void pinModes(uint64_t mask, uint8_t mode)
{
    for (uint64_t i = 0; i < NUM_GPIOS; i++) {
        if (mask & (1ULL << i)) {
            pinMode(i, mode);
        }
    }
}

/// Read all the pins mentioned in a mask
static uint64_t digitalReads(uint64_t mask)
{
    uint64_t res = 0;

    pinModes(mask, INPUT_PULLUP);

    for (uint64_t i = 0; i < NUM_GPIOS; i++) {
        uint64_t m = 1ULL << i;
        if (mask & m) {
            if (digitalRead(i)) {
                res |= m;
            }
        }
    }

    return res;
}

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
    if (false) {
        auto p = *pptr;
        LOG_INFO("Received KiezboxControl type=%d\n", p.type);

        switch (p.type) {
        case meshtastic_KiezboxMessage_Type_WRITE_GPIOS: {
            // Print notification to LCD screen
            screen->print("Write GPIOs\n");

            pinModes(p.gpio_mask, OUTPUT);
            for (uint8_t i = 0; i < NUM_GPIOS; i++) {
                uint64_t mask = 1ULL << i;
                if (p.gpio_mask & mask) {
                    digitalWrite(i, (p.gpio_value & mask) ? 1 : 0);
                }
            }

            break;
        }

        case meshtastic_KiezboxMessage_Type_READ_GPIOS: {
            // Print notification to LCD screen
            if (screen)
                screen->print("Read GPIOs\n");

            uint64_t res = digitalReads(p.gpio_mask);

            // Send the reply
            meshtastic_KiezboxMessage r = meshtastic_KiezboxMessage_init_default;
            r.type = meshtastic_KiezboxMessage_Type_READ_GPIOS_REPLY;
            r.gpio_value = res;
            r.gpio_mask = p.gpio_mask;
            meshtastic_MeshPacket *p2 = allocDataProtobuf(r);
            setReplyTo(p2, req);
            myReply = p2;
            break;
        }

        case meshtastic_KiezboxMessage_Type_WATCH_GPIOS: {
            watchGpios = p.gpio_mask;
            lastWatchMsec = 0; // Force a new publish soon
            previousWatch =
                ~watchGpios;   // generate a 'previous' value which is guaranteed to not match (to force an initial publish)
            enabled = true;    // Let our thread run at least once
            setInterval(2000); // Set a new interval so we'll run soon
            LOG_INFO("Now watching GPIOs 0x%llx\n", watchGpios);
            break;
        }

        case meshtastic_KiezboxMessage_Type_READ_GPIOS_REPLY:
        case meshtastic_KiezboxMessage_Type_GPIOS_CHANGED:
            break; // Ignore - we might see our own replies

        default:
            LOG_ERROR("Hardware/Kiezbox operation %d not yet implemented! FIXME\n", p.type);
            break;
        }
    }

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
    return 20000; // Poll our GPIOs every 20s
}
