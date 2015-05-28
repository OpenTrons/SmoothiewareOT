/*
 * Feedback.h
 *
 *  Created on: Mar 15, 2015
 *      Author: nicholas
 */

#ifndef FEEDBACK_H_
#define FEEDBACK_H_

#include "libs/Module.h"

class Gcode;
class StreamOutput;

class Feedback : public Module {

public:
	Feedback();
	void feedback_switch(bool is_on);
	uint32_t feedback_tick( uint32_t dummy );

	uint32_t feedback_pos( uint32_t value );

	void on_module_loaded();
	void on_config_reload(void *argument);
	void on_gcode_received(void *argument);
	void flash_stat(int state);
	void finished_homing(char axes_to_move, char abc_axes_to_move);
	bool include_stat();

	int machine_state;
	int machine_state_ticker;
	int machine_state_zero_trip;
	int machine_state_one_trip;
	bool auto_stat;

	float positions[6];
	float positions_last[6];
	int32_t cumulative_steps[6];
	int32_t cumulative_steps_last[6];
	float steps_per_mm[6];

	bool send_feedback;
	std::string config_version;

	typedef struct {
		int			f_id;
		int32_t		f_current_position_steps;
	} ID_POS;

private:
	bool attached;

};

#endif /* FEEDBACK_H_ */
