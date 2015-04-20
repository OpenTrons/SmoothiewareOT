/*
 * OnTheFlyData.h
 *
 *  Created on: Apr 11, 2015
 *      Author: hotslice
 */

#ifndef SRC_MODULES_COMMUNICATION_ONTHEFLYDATA_H_
#define SRC_MODULES_COMMUNICATION_ONTHEFLYDATA_H_


class OnTheFlyData {
    public:
        static bool get_value(uint16_t csa, void **data) { return get_value(csa, 0, 0, data); }
        static bool get_value(uint16_t csa, uint16_t csb, void **data) { return get_value(csa, csb, 0, data); }
        static bool get_value(uint16_t cs[3], void **data) { return get_value(cs[0], cs[1], cs[2], data); };
        static bool get_value(uint16_t csa, uint16_t csb, uint16_t csc, void **data);

        static bool set_value(uint16_t csa, void *data) { return set_value(csa, 0, 0, data); }
        static bool set_value(uint16_t csa, uint16_t csb, void *data) { return set_value(csa, csb, 0, data); }
        static bool set_value(uint16_t cs[3], void *data) { return set_value(cs[0], cs[1], cs[2], data); }
        static bool set_value(uint16_t csa, uint16_t csb, uint16_t csc, void *data);
};



#endif /* SRC_MODULES_COMMUNICATION_ONTHEFLYDATA_H_ */
