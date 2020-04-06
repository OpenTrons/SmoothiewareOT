#include "checksumm.h"
#include "Config.h"
#include "ConfigValue.h"
#include "libs/Pin.h"
#include "libs/Kernel.h"
#include "libs/utils.h"
#include "Revision.h"


#define pcb_revision_bit_0           CHECKSUM("pcb_revision_bit_0")
#define pcb_revision_bit_1           CHECKSUM("pcb_revision_bit_1")

static const char* REV12_NAME = "12";
static const char* REVA_NAME = "A";
static const char* REVB_NAME = "B";
static const char* REVC_NAME = "C";
static const char* REVINVALID_NAME = "INVALID";

const char *string_from_rev(const Rev& r) {
    switch (r) {
        case REV_12:
            return REV12_NAME;
        case REV_A:
            return REVA_NAME;
        case REV_B:
            return REVB_NAME;
        case REV_C:
            return REVC_NAME;
        case REV_INVALID:
        default:
            return REVINVALID_NAME;
    }
}

void RevisionManager::on_module_loaded() {
    pcb_revision = get_pcb_revision();
}

// read GPIO to determine the major version number of the OT2 stepper driver hardware
Rev RevisionManager::get_pcb_revision() const
{
    Pin *rev_bit_0 = new Pin();
    Pin *rev_bit_1 = new Pin();
    rev_bit_0->from_string(THEKERNEL->config->value(pcb_revision_bit_0)->as_string())->as_input();
    rev_bit_1->from_string(THEKERNEL->config->value(pcb_revision_bit_1)->as_string())->as_input();
    safe_delay_us(200);  // make sure the pins have settled
    const uint8_t rev_byte = (((uint8_t)rev_bit_1->get()) << 1) | (uint8_t)rev_bit_0->get();
    switch (rev_byte) {
        case 0:
            return REV_12;
        case 1:
            return REV_A;
        case 2:
            return REV_B;
        case 3:
            return REV_C;
        default:
            // this can never happen because we're only setting 2 bits but why
            // not be careful
            return REV_INVALID;
    }
}
