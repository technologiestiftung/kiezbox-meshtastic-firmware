#pragma once
#include "ProtobufModule.h"
#include "concurrency/OSThread.h"
#include "mesh/generated/meshtastic/kiezbox_control.pb.h"

// Sensor stuff
#include "DHT.h"
#include <OneWire.h>
#include <DallasTemperature.h>

/**
 * A module that provides easy low-level remote access to device hardware.
 */
class KiezboxControlModule : public ProtobufModule<meshtastic_KiezboxMessage>, private concurrency::OSThread
{
    /// The current set of GPIOs we've been asked to watch for changes
    uint64_t watchGpios = 0;

    /// The previously read value of watched pins
    uint64_t previousWatch = 0;

    /// The timestamp of our last watch event (we throttle watches to 1 change every 30 seconds)
    uint32_t lastWatchMsec = 0;
    DHT dht;
    OneWire onewire;
    DallasTemperature dallas;

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
};

extern KiezboxControlModule kiezboxControlModule;