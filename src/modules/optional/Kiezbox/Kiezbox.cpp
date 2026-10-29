#include "Kiezbox.h"
#include "configuration.h"
#if MESHTASTIC_INCLUDE_KIEZBOXCONTROL
#include "KiezboxControlModule.h"
#endif

void setupKiezbox()
{
#if MESHTASTIC_INCLUDE_KIEZBOXCONTROL
    new KiezboxControlModule();
#endif
}
