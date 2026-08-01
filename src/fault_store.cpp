//faults

#include <Preferences.h>
#include "fault_store.h"

static Preferences s_prefs;
static uint32_t s_writeCount = 0;

bool faultStoreInit() {
    return s_prefs.begin("faults", /* readOnly = */ false);
}

uint16_t faultStoreLoad() {
    return s_prefs.getUShort("latched", 0);
}

void faultStoreSave(uint16_t latchedFaults) {
      s_prefs.putUShort("latched", latchedFaults);
      s_writeCount++;
}

uint32_t faultStoreWriteCount() {
    return s_writeCount;
}
