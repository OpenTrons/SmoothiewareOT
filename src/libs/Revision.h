#pragma once

#include "Module.h"

enum Rev: uint8_t {
    REV_INVALID = 0,
    REV_12 = 1,
    REV_A = 2,
    REV_B = 3,
    REV_C = 4
};

const char *string_from_rev(const Rev& r);

class RevisionManager : public Module
{
public:
    RevisionManager() {}
    void on_module_loaded();
    Rev pcb_revision = Rev::REV_12;
private:
     Rev get_pcb_revision() const;
};
