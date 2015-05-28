/*
 * Feedback.cpp
 *
 *  Created on: Mar 15, 2015
 *      Author: nicholas
 */

#include "libs/Module.h"
#include "libs/Kernel.h"

#include "Feedback.h"
#include "Gcode.h"
#include "libs/StreamOutput.h"
#include "StreamOutputPool.h"
#include "libs/SlowTicker.h"
#include "libs/nuts_bolts.h"
#include "modules/robot/robot.h"
#include "libs/StepperMotor.h"
#include "checksumm.h"
#include "Config.h"
#include "ConfigValue.h"
#include <math.h>

#define DEBUG 0

#define config_version_checksum						CHECKSUM("config_version")


Feedback::Feedback()
{
	send_feedback = false;
	attached = false;
	machine_state = 0;
	machine_state_ticker = 0;
	machine_state_zero_trip = 0;
	machine_state_one_trip = 0;
	auto_stat = false;

	for(int i=0; i<6; i++){
		cumulative_steps[i] = cumulative_steps_last[i] = 0;
		positions[i] = positions_last[i] = 0.0F;

	}
}

void Feedback::on_module_loaded()
{
	this->register_for_event(ON_GCODE_RECEIVED);
}

void Feedback::on_config_reload(void *argument)
{
	//is this even necessary for showing config version?... just issue "config-get sd version" command
	this->config_version = THEKERNEL->config->value(config_version_checksum)->by_default("nc")->as_string();
}

void Feedback::on_gcode_received(void *argument)
{
	Gcode *gcode = static_cast<Gcode *>(argument);

	if( gcode-> has_m ) {
		switch( gcode->m ) {
		case 62: {
			if(gcode->has_letter('P'))
				this->machine_state_zero_trip = gcode->get_int('P');

			if(gcode->has_letter('S'))
				this->machine_state_one_trip = gcode->get_int('S');

			if(this->machine_state_zero_trip < 0 && this->machine_state_one_trip < 0) {
				this->auto_stat = false;
			} else if(this->machine_state_zero_trip == 0 && this->machine_state_one_trip == 0) {
				this->auto_stat = true;
				this->machine_state_zero_trip = 3;
				this->machine_state_one_trip = 3;
			} else {
				this->auto_stat = true;
			}

			this->feedback_switch(true);
			THEKERNEL->streams->printf("feedback engaged\r\n");
			for(int i=0; i<6; i++) {
				if(!THEKERNEL->robot->actuators[i]->moving){
					float position = THEKERNEL->robot->actuators[i]->get_current_position();
					this->cumulative_steps[i] = this->steps_per_mm[i] * position;
					this->cumulative_steps_last[i] = this->cumulative_steps[i]-1;
				}
			}
			gcode->mark_as_taken();
		}
		return;
		case 63: {
			this->feedback_switch(false);
			THEKERNEL->streams->printf("feedback disengaged\r\n");
			gcode->mark_as_taken();
		}
		return;
		}
	}
}

void Feedback::feedback_switch(bool is_on)
{
	if(is_on) {
		send_feedback = true;
		for(int i=0; i<6; i++) {
			THEKERNEL->robot->actuators[i]->feedback_step_switch(true);
		}
	}else{
		send_feedback = false;
		for(int i=0; i<6; i++) {
			THEKERNEL->robot->actuators[i]->feedback_step_switch(false);
			cumulative_steps_last[i] = cumulative_steps[i];
		}
	}

	if(!attached && send_feedback) {
		attached = true;
		this->machine_state_ticker = 0;

		for(int i=0; i<6; i++) {
			THEKERNEL->robot->actuators[i]->attach_feedback(this, &Feedback::feedback_pos );
		}

		THEKERNEL->slow_ticker->attach( 3, this, &Feedback::feedback_tick );
	}
}

uint32_t Feedback::feedback_pos( uint32_t value )
{
	ID_POS *id_pos = (ID_POS*)value;
	this->cumulative_steps[id_pos->f_id] = id_pos->f_current_position_steps;

	return 0;
}


uint32_t Feedback::feedback_tick( uint32_t dummy )
{
	machine_state_ticker++;

	/*if(DEBUG && machine_state_ticker < 2) {
		THEKERNEL->streams->printf("**********************\r\n");
		THEKERNEL->streams->printf("machine_state_ticker: %i\r\n", machine_state_ticker);
		THEKERNEL->streams->printf("send_feedback: %i\r\n", send_feedback);
		THEKERNEL->streams->printf("auto_stat: %i\r\n", auto_stat);
		THEKERNEL->streams->printf("machine_state_zero_trip: %i\r\n", machine_state_zero_trip);
		THEKERNEL->streams->printf("machine_state_one_trip: %i\r\n", machine_state_one_trip);
		THEKERNEL->streams->printf("**********************\r\n\r\n");
	}*/

	if(send_feedback) {

		char xyz = 0;
		char abc = 0;
		for(int i=0; i<3; i++) {
			if(this->cumulative_steps[i] != this->cumulative_steps_last[i]){
				xyz += (1 << i);
				this->cumulative_steps_last[i] = this->cumulative_steps[i];
			}
		}
		for(int i=3; i<6; i++) {
			if(this->cumulative_steps[i] != this->cumulative_steps_last[i]){
				abc += (1 << (i-3));
				this->cumulative_steps_last[i] = this->cumulative_steps[i];
			}
		}
		if (abc==0 && xyz==0){
			if(include_stat()){
				THEKERNEL->streams->printf("{\"stat\":%i}\r\n", this->machine_state);
			}
		}else{
			if (xyz == 0x1) {
				positions[0] = ((float)this->cumulative_steps[0]/this->steps_per_mm[0]);
				if (include_stat()) THEKERNEL->streams->printf("{\"x\":%i.%u,\"stat\":%i}\r\n",(int)positions[0],(int)(abs(positions[0])*1000)%1000,this->machine_state);
				else 				THEKERNEL->streams->printf("{\"x\":%i.%u}\r\n",            (int)positions[0],(int)(abs(positions[0])*1000)%1000);
			}
			else if (xyz == 0x2) {
				positions[1] = ((float)this->cumulative_steps[1]/this->steps_per_mm[1]);
				if (include_stat()) THEKERNEL->streams->printf("{\"y\":%i.%u,\"stat\":%i}\r\n",(int)positions[1],(int)(abs(positions[1])*1000)%1000,this->machine_state);
				else 				THEKERNEL->streams->printf("{\"y\":%i.%u}\r\n"            ,(int)positions[1],(int)(abs(positions[1])*1000)%1000);
			}
			else if (xyz == 0x4) {
				positions[2] = ((float)this->cumulative_steps[2]/this->steps_per_mm[2]);
				if (include_stat()) THEKERNEL->streams->printf("{\"z\":%i.%u,\"stat\":%i}\r\n",(int)positions[2],(int)(abs(positions[2])*1000)%1000,this->machine_state);
				else 				THEKERNEL->streams->printf("{\"z\":%i.%u}\r\n"            ,(int)positions[2],(int)(abs(positions[2])*1000)%1000);
			}
			else if (xyz == 0x3) {
				positions[0] = ((float)this->cumulative_steps[0]/this->steps_per_mm[0]);
				positions[1] = ((float)this->cumulative_steps[1]/this->steps_per_mm[1]);
				if (include_stat()) THEKERNEL->streams->printf("{\"x\":%i.%u,\"y\":%i.%u,\"stat\":%i}\r\n",(int)positions[0],(int)(abs(positions[0])*1000)%1000,(int)positions[1],(int)(abs(positions[1])*1000)%1000,this->machine_state);
				else 				THEKERNEL->streams->printf("{\"x\":%i.%u,\"y\":%i.%u}\r\n"            ,(int)positions[0],(int)(abs(positions[0])*1000)%1000,(int)positions[1],(int)(abs(positions[1])*1000)%1000);
			}
			else if (xyz == 0x5) {
				positions[0] = ((float)this->cumulative_steps[0]/this->steps_per_mm[0]);
				positions[2] = ((float)this->cumulative_steps[2]/this->steps_per_mm[2]);
				if (include_stat()) THEKERNEL->streams->printf("{\"x\":%i.%u,\"z\":%i.%u,\"stat\":%i}\r\n",(int)positions[0],(int)(abs(positions[0])*1000)%1000,(int)positions[2],(int)(abs(positions[2])*1000)%1000,this->machine_state);
				else 				THEKERNEL->streams->printf("{\"x\":%i.%u,\"z\":%i.%u}\r\n"            ,(int)positions[0],(int)(abs(positions[0])*1000)%1000,(int)positions[2],(int)(abs(positions[2])*1000)%1000);
			}
			else if (xyz == 0x6){
				positions[2] = ((float)this->cumulative_steps[2]/this->steps_per_mm[2]);
				positions[1] = ((float)this->cumulative_steps[1]/this->steps_per_mm[1]);
				if (include_stat()) THEKERNEL->streams->printf("{\"y\":%i.%u,\"z\":%i.%u,\"stat\":%i}\r\n",(int)positions[1],(int)(abs(positions[1])*1000)%1000,(int)positions[2],(int)(abs(positions[2])*1000)%1000,this->machine_state);
				else 				THEKERNEL->streams->printf("{\"y\":%i.%u,\"z\":%i.%u}\r\n"            ,(int)positions[1],(int)(abs(positions[1])*1000)%1000,(int)positions[2],(int)(abs(positions[2])*1000)%1000);
			}
			else if (xyz == 0x7){
				positions[0] = ((float)this->cumulative_steps[0]/this->steps_per_mm[0]);
				positions[1] = ((float)this->cumulative_steps[1]/this->steps_per_mm[1]);
				positions[2] = ((float)this->cumulative_steps[2]/this->steps_per_mm[2]);
				if (include_stat()) THEKERNEL->streams->printf("{\"x\":%i.%u,\"y\":%i.%u,\"z\":%i.%u,\"stat\":%i}\r\n",(int)positions[0],(int)(abs(positions[0])*1000)%1000,(int)positions[1],(int)(abs(positions[1])*1000)%1000,(int)positions[2],(int)(abs(positions[2])*1000)%1000,this->machine_state);
				else 				THEKERNEL->streams->printf("{\"x\":%i.%u,\"y\":%i.%u,\"z\":%i.%u}\r\n"            ,(int)positions[0],(int)(abs(positions[0])*1000)%1000,(int)positions[1],(int)(abs(positions[1])*1000)%1000,(int)positions[2],(int)(abs(positions[2])*1000)%1000);
			}
			if (abc == 0x1) {
				positions[3] = ((float)this->cumulative_steps[3]/this->steps_per_mm[3]);
				if (include_stat()) THEKERNEL->streams->printf("{\"a\":%i.%u,\"stat\":%i}\r\n",(int)positions[3],(int)(abs(positions[3])*1000)%1000,this->machine_state);
				else 				THEKERNEL->streams->printf("{\"a\":%i.%u}\r\n",            (int)positions[3],(int)(abs(positions[3])*1000)%1000);
			}
			else if (abc == 0x2) {
				positions[4] = ((float)this->cumulative_steps[4]/this->steps_per_mm[4]);
				if (include_stat()) THEKERNEL->streams->printf("{\"b\":%i.%u,\"stat\":%i}\r\n",(int)positions[4],(int)(abs(positions[4])*1000)%1000,this->machine_state);
				else 				THEKERNEL->streams->printf("{\"b\":%i.%u}\r\n"            ,(int)positions[4],(int)(abs(positions[4])*1000)%1000);
			}
			else if (abc == 0x4) {
				positions[5] = ((float)this->cumulative_steps[5]/this->steps_per_mm[5]);
				if (include_stat()) THEKERNEL->streams->printf("{\"c\":%i.%u,\"stat\":%i}\r\n",(int)positions[5],(int)(abs(positions[5])*1000)%1000,this->machine_state);
				else 				THEKERNEL->streams->printf("{\"c\":%i.%u}\r\n"            ,(int)positions[5],(int)(abs(positions[5])*1000)%1000);
			}
			else if (abc == 0x3) {
				positions[3] = ((float)this->cumulative_steps[3]/this->steps_per_mm[3]);
				positions[4] = ((float)this->cumulative_steps[4]/this->steps_per_mm[4]);
				if (include_stat()) THEKERNEL->streams->printf("{\"a\":%i.%u,\"b\":%i.%u,\"stat\":%i}\r\n",(int)positions[3],(int)(abs(positions[3])*1000)%1000,(int)positions[4],(int)(abs(positions[4])*1000)%1000,this->machine_state);
				else 				THEKERNEL->streams->printf("{\"a\":%i.%u,\"b\":%i.%u}\r\n"            ,(int)positions[3],(int)(abs(positions[3])*1000)%1000,(int)positions[4],(int)(abs(positions[4])*1000)%1000);
			}
			else if (abc == 0x5) {
				positions[3] = ((float)this->cumulative_steps[3]/this->steps_per_mm[3]);
				positions[5] = ((float)this->cumulative_steps[5]/this->steps_per_mm[5]);
				if (include_stat()) THEKERNEL->streams->printf("{\"a\":%i.%u,\"c\":%i.%u,\"stat\":%i}\r\n",(int)positions[3],(int)(abs(positions[3])*1000)%1000,(int)positions[5],(int)(abs(positions[5])*1000)%1000,this->machine_state);
				else 				THEKERNEL->streams->printf("{\"a\":%i.%u,\"c\":%i.%u}\r\n"            ,(int)positions[3],(int)(abs(positions[3])*1000)%1000,(int)positions[5],(int)(abs(positions[5])*1000)%1000);
			}
			else if (abc == 0x6){
				positions[5] = ((float)this->cumulative_steps[5]/this->steps_per_mm[5]);
				positions[4] = ((float)this->cumulative_steps[4]/this->steps_per_mm[4]);
				if (include_stat()) THEKERNEL->streams->printf("{\"b\":%i.%u,\"c\":%i.%u,\"stat\":%i}\r\n",(int)positions[4],(int)(abs(positions[4])*1000)%1000,(int)positions[5],(int)(abs(positions[5])*1000)%1000,this->machine_state);
				else 				THEKERNEL->streams->printf("{\"b\":%i.%u,\"c\":%i.%u}\r\n"            ,(int)positions[4],(int)(abs(positions[4])*1000)%1000,(int)positions[5],(int)(abs(positions[5])*1000)%1000);
			}
			else if (abc == 0x7){
				positions[3] = ((float)this->cumulative_steps[3]/this->steps_per_mm[3]);
				positions[4] = ((float)this->cumulative_steps[4]/this->steps_per_mm[4]);
				positions[5] = ((float)this->cumulative_steps[5]/this->steps_per_mm[5]);
				if (include_stat()) THEKERNEL->streams->printf("{\"a\":%i.%u,\"b\":%i.%u,\"c\":%i.%u,\"stat\":%i}\r\n",(int)positions[3],(int)(abs(positions[3])*1000)%1000,(int)positions[4],(int)(abs(positions[4])*1000)%1000,(int)positions[5],(int)(abs(positions[5])*1000)%1000,this->machine_state);
				else 				THEKERNEL->streams->printf("{\"a\":%i.%u,\"b\":%i.%u,\"c\":%i.%u}\r\n"            ,(int)positions[3],(int)(abs(positions[3])*1000)%1000,(int)positions[4],(int)(abs(positions[4])*1000)%1000,(int)positions[5],(int)(abs(positions[5])*1000)%1000);
			}
		}
	}
	return 0;
}

void Feedback::flash_stat(int state)
{
	if(this->send_feedback)
		THEKERNEL->streams->printf("{\"stat\":%i}\r\n", state);
}

void Feedback::finished_homing(char axes_to_move, char abc_axes_to_move)
{
	for ( int c = X_AXIS; c <= Z_AXIS; c++ ) {
		if (  ( axes_to_move >> c ) & 1 ) {
			this->cumulative_steps_last[c] = -9999;
		}
	}
	for ( int c = A_AXIS; c <= C_AXIS; c++ ) {
		if (  ( abc_axes_to_move >> (c-3) ) & 1 ) {
			this->cumulative_steps_last[c] = -9999;
		}
	}
	if(DEBUG) {
		THEKERNEL->streams->printf("Feedback::finished_homing()=>");
		THEKERNEL->streams->printf("machine_state: %i\r\n", this->machine_state);
	}
	//this->machine_state = 0;
}

bool Feedback::include_stat()
{
	bool inc_stat = false;

	if(auto_stat){
		if(this-> machine_state == 0 && this->machine_state_zero_trip > 0 ) {
			if(this->machine_state_ticker%this->machine_state_zero_trip==0 && this->auto_stat) {
				this->machine_state_ticker = 0;
				//THEKERNEL->streams->printf(",\"stat\":0}\r\n");
				inc_stat = true;
			} /*else {
				THEKERNEL->streams->printf("}\r\n");
			}*/
		}else if(this->machine_state == 1 && this->machine_state_one_trip > 0 ) {
			if(this->machine_state_ticker%this->machine_state_one_trip==0 && this->auto_stat) {
				this->machine_state_ticker = 0;
				//THEKERNEL->streams->printf(",\"stat\":1}\r\n");
				inc_stat = true;
			} /*else {
				THEKERNEL->streams->printf("}\r\n");
			}*/
		}else {
			if(this->machine_state_ticker%3==0 && this->auto_stat) {
				this->machine_state_ticker = 0;
				//THEKERNEL->streams->printf(",\"stat\":%i}\r\n", this->machine_state);
				inc_stat = true;
			} /*else {
				THEKERNEL->streams->printf("}\r\n");
			}*/
		}
	}

	return inc_stat;
}
