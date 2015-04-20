/*
 * OnTheFlyData.cpp
 *
 *  Created on: Apr 11, 2015
 *      Author: hotslice
 */

#include "libs/Kernel.h"
#include "OnTheFlyData.h"
#include "PublicDataRequest.h"

bool OnTheFlyData::get_value(uint16_t csa, uint16_t csb, uint16_t csc, void **data) {
    PublicDataRequest pdr(csa, csb, csc);
    THEKERNEL->call_event(ON_THE_FLY_GET, &pdr );
    *data= pdr.get_data_ptr();
    return pdr.is_taken();
}

bool OnTheFlyData::set_value(uint16_t csa, uint16_t csb, uint16_t csc, void *data) {
    PublicDataRequest pdr(csa, csb, csc);
    pdr.set_data_ptr(data);
    THEKERNEL->call_event(ON_THE_FLY_SET, &pdr );
    return pdr.is_taken();
}


