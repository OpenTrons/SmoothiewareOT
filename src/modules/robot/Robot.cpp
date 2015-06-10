/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl) with additions from Sungeun K. Jeon (https://github.com/chamnit/grbl)
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"

#include <math.h>
#include <string>
using std::string;

#include "Planner.h"
#include "Conveyor.h"
#include "Robot.h"
#include "nuts_bolts.h"
#include "Pin.h"
#include "StepperMotor.h"
#include "Gcode.h"
#include "PublicDataRequest.h"
#include "RobotPublicAccess.h"
#include "arm_solutions/BaseSolution.h"
#include "arm_solutions/CartesianSolution.h"
#include "arm_solutions/RotatableCartesianSolution.h"
#include "arm_solutions/LinearDeltaSolution.h"
#include "arm_solutions/HBotSolution.h"
#include "arm_solutions/MorganSCARASolution.h"
#include "StepTicker.h"
#include "checksumm.h"
#include "utils.h"
#include "ConfigValue.h"
#include "libs/StreamOutput.h"
#include "StreamOutputPool.h"

#include "Feedback.h"

#define  default_seek_rate_checksum          CHECKSUM("default_seek_rate")

#define  a_default_seek_rate_checksum          CHECKSUM("a_default_seek_rate")
#define  b_default_seek_rate_checksum          CHECKSUM("b_default_seek_rate")
#define  c_default_seek_rate_checksum          CHECKSUM("c_default_seek_rate")

#define  default_feed_rate_checksum          CHECKSUM("default_feed_rate")

#define  a_default_feed_rate_checksum          CHECKSUM("a_default_feed_rate")
#define  b_default_feed_rate_checksum          CHECKSUM("b_default_feed_rate")
#define  c_default_feed_rate_checksum          CHECKSUM("c_default_feed_rate")

#define  mm_per_line_segment_checksum        CHECKSUM("mm_per_line_segment")
#define  delta_segments_per_second_checksum  CHECKSUM("delta_segments_per_second")
#define  mm_per_arc_segment_checksum         CHECKSUM("mm_per_arc_segment")
#define  arc_correction_checksum             CHECKSUM("arc_correction")
#define  x_axis_max_speed_checksum           CHECKSUM("x_axis_max_speed")
#define  y_axis_max_speed_checksum           CHECKSUM("y_axis_max_speed")
#define  z_axis_max_speed_checksum           CHECKSUM("z_axis_max_speed")

// arm solutions
#define  arm_solution_checksum               CHECKSUM("arm_solution")
#define  cartesian_checksum                  CHECKSUM("cartesian")
#define  rotatable_cartesian_checksum        CHECKSUM("rotatable_cartesian")
#define  rostock_checksum                    CHECKSUM("rostock")
#define  linear_delta_checksum               CHECKSUM("linear_delta")
#define  delta_checksum                      CHECKSUM("delta")
#define  hbot_checksum                       CHECKSUM("hbot")
#define  corexy_checksum                     CHECKSUM("corexy")
#define  kossel_checksum                     CHECKSUM("kossel")
#define  morgan_checksum                     CHECKSUM("morgan")

// stepper motor stuff
#define  alpha_step_pin_checksum             CHECKSUM("alpha_step_pin")
#define  beta_step_pin_checksum              CHECKSUM("beta_step_pin")
#define  gamma_step_pin_checksum             CHECKSUM("gamma_step_pin")
#define  alpha_dir_pin_checksum              CHECKSUM("alpha_dir_pin")
#define  beta_dir_pin_checksum               CHECKSUM("beta_dir_pin")
#define  gamma_dir_pin_checksum              CHECKSUM("gamma_dir_pin")
#define  alpha_en_pin_checksum               CHECKSUM("alpha_en_pin")
#define  beta_en_pin_checksum                CHECKSUM("beta_en_pin")
#define  gamma_en_pin_checksum               CHECKSUM("gamma_en_pin")

#define  chi_step_pin_checksum             	CHECKSUM("chi_step_pin")
#define  psi_step_pin_checksum              CHECKSUM("psi_step_pin")
#define  omega_step_pin_checksum            CHECKSUM("omega_step_pin")
#define  chi_dir_pin_checksum              	CHECKSUM("chi_dir_pin")
#define  psi_dir_pin_checksum               CHECKSUM("psi_dir_pin")
#define  omega_dir_pin_checksum             CHECKSUM("omega_dir_pin")
#define  chi_en_pin_checksum               	CHECKSUM("chi_en_pin")
#define  psi_en_pin_checksum                CHECKSUM("psi_en_pin")
#define  omega_en_pin_checksum              CHECKSUM("omega_en_pin")


#define  alpha_steps_per_mm_checksum         CHECKSUM("alpha_steps_per_mm")
#define  beta_steps_per_mm_checksum          CHECKSUM("beta_steps_per_mm")
#define  gamma_steps_per_mm_checksum         CHECKSUM("gamma_steps_per_mm")

#define  chi_steps_per_mm_checksum         	CHECKSUM("chi_steps_per_mm")
#define  psi_steps_per_mm_checksum          CHECKSUM("psi_steps_per_mm")
#define  omega_steps_per_mm_checksum        CHECKSUM("omega_steps_per_mm")

#define  alpha_max_rate_checksum             CHECKSUM("alpha_max_rate")
#define  beta_max_rate_checksum              CHECKSUM("beta_max_rate")
#define  gamma_max_rate_checksum             CHECKSUM("gamma_max_rate")

#define  chi_max_rate_checksum             	CHECKSUM("chi_max_rate")
#define  psi_max_rate_checksum              CHECKSUM("psi_max_rate")
#define  omega_max_rate_checksum            CHECKSUM("omega_max_rate")

// new-style actuator stuff
#define  actuator_checksum                   CHEKCSUM("actuator")

#define  step_pin_checksum                   CHECKSUM("step_pin")
#define  dir_pin_checksum                    CHEKCSUM("dir_pin")
#define  en_pin_checksum                     CHECKSUM("en_pin")

#define  steps_per_mm_checksum               CHECKSUM("steps_per_mm")
#define  max_rate_checksum                   CHECKSUM("max_rate")

#define  alpha_checksum                      CHECKSUM("alpha")
#define  beta_checksum                       CHECKSUM("beta")
#define  gamma_checksum                      CHECKSUM("gamma")

#define  chi_checksum                      	CHECKSUM("chi")
#define  psi_checksum                       CHECKSUM("psi")
#define  omega_checksum                     CHECKSUM("omega")


#define NEXT_ACTION_DEFAULT 0
#define NEXT_ACTION_DWELL 1
#define NEXT_ACTION_GO_HOME 2

#define MOTION_MODE_SEEK 0 // G0
#define MOTION_MODE_LINEAR 1 // G1
#define MOTION_MODE_CW_ARC 2 // G2
#define MOTION_MODE_CCW_ARC 3 // G3
#define MOTION_MODE_CANCEL 4 // G80

#define PATH_CONTROL_MODE_EXACT_PATH 0
#define PATH_CONTROL_MODE_EXACT_STOP 1
#define PATH_CONTROL_MODE_CONTINOUS 2

#define PROGRAM_FLOW_RUNNING 0
#define PROGRAM_FLOW_PAUSED 1
#define PROGRAM_FLOW_COMPLETED 2

#define SPINDLE_DIRECTION_CW 0
#define SPINDLE_DIRECTION_CCW 1

// The Robot converts GCodes into actual movements, and then adds them to the Planner, which passes them to the Conveyor so they can be added to the queue
// It takes care of cutting arcs into segments, same thing for line that are too long
#define max(a,b) (((a) > (b)) ? (a) : (b))

#define DEBUG	0

Robot::Robot()
{
    this->inch_mode = false;
    this->absolute_mode = true;
    this->motion_mode =  MOTION_MODE_SEEK;
    this->select_plane(X_AXIS, Y_AXIS, Z_AXIS);
    clear_vector(this->last_milestone);
    clear_vector(this->transformed_last_milestone);
    this->arm_solution = NULL;
    seconds_per_minute = 60.0F;
    this->clearToolOffset();
    this->compensationTransform= nullptr;
    this->halted= false;
}

//Called when the module has just been loaded
void Robot::on_module_loaded()
{
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);
    this->register_for_event(ON_HALT);
    this->register_for_event(ON_THE_FLY_GET);
    this->register_for_event(ON_THE_FLY_SET);

    // Configuration
    this->on_config_reload(this);
}

void Robot::on_config_reload(void *argument)
{

    // Arm solutions are used to convert positions in millimeters into position in steps for each stepper motor.
    // While for a cartesian arm solution, this is a simple multiplication, in other, less simple cases, there is some serious math to be done.
    // To make adding those solution easier, they have their own, separate object.
    // Here we read the config to find out which arm solution to use
    if (this->arm_solution) delete this->arm_solution;
    int solution_checksum = get_checksum(THEKERNEL->config->value(arm_solution_checksum)->by_default("cartesian")->as_string());
    // Note checksums are not const expressions when in debug mode, so don't use switch
    if(solution_checksum == hbot_checksum || solution_checksum == corexy_checksum) {
        this->arm_solution = new HBotSolution(THEKERNEL->config);

    } else if(solution_checksum == rostock_checksum || solution_checksum == kossel_checksum || solution_checksum == delta_checksum || solution_checksum ==  linear_delta_checksum) {
        this->arm_solution = new LinearDeltaSolution(THEKERNEL->config);

    } else if(solution_checksum == rotatable_cartesian_checksum) {
        this->arm_solution = new RotatableCartesianSolution(THEKERNEL->config);

    } else if(solution_checksum == morgan_checksum) {
        this->arm_solution = new MorganSCARASolution(THEKERNEL->config);

    } else if(solution_checksum == cartesian_checksum) {
        this->arm_solution = new CartesianSolution(THEKERNEL->config);

    } else {
        this->arm_solution = new CartesianSolution(THEKERNEL->config);
    }


    this->feed_rate           = THEKERNEL->config->value(default_feed_rate_checksum   )->by_default(  3000.0F)->as_number();

    this->a_feed_rate           = THEKERNEL->config->value(a_default_feed_rate_checksum   )->by_default(  300.0F)->as_number();
    this->b_feed_rate           = THEKERNEL->config->value(b_default_feed_rate_checksum   )->by_default(  300.0F)->as_number();
    this->c_feed_rate           = THEKERNEL->config->value(c_default_feed_rate_checksum   )->by_default(  300.0F)->as_number();

    this->seek_rate           = THEKERNEL->config->value(default_seek_rate_checksum   )->by_default(  3000.0F)->as_number();

    this->a_seek_rate           = THEKERNEL->config->value(a_default_seek_rate_checksum   )->by_default(  300.0F)->as_number();
    this->b_seek_rate           = THEKERNEL->config->value(b_default_seek_rate_checksum   )->by_default(  300.0F)->as_number();
    this->c_seek_rate           = THEKERNEL->config->value(c_default_seek_rate_checksum   )->by_default(  300.0F)->as_number();

    this->default_seek_rates[0] = this->seek_rate;
    this->default_seek_rates[1] = this->a_seek_rate;
    this->default_seek_rates[2] = this->b_seek_rate;
    this->default_seek_rates[3] = this->c_seek_rate;

    this->mm_per_line_segment = THEKERNEL->config->value(mm_per_line_segment_checksum )->by_default(    5.0F)->as_number();
    this->delta_segments_per_second = THEKERNEL->config->value(delta_segments_per_second_checksum )->by_default(0.0f   )->as_number();
    this->mm_per_arc_segment  = THEKERNEL->config->value(mm_per_arc_segment_checksum  )->by_default(    0.5f)->as_number();
    this->arc_correction      = THEKERNEL->config->value(arc_correction_checksum      )->by_default(    5   )->as_number();

    this->max_speeds[X_AXIS]  = THEKERNEL->config->value(x_axis_max_speed_checksum    )->by_default(30000.0F)->as_number() / 60.0F;
    this->max_speeds[Y_AXIS]  = THEKERNEL->config->value(y_axis_max_speed_checksum    )->by_default(30000.0F)->as_number() / 60.0F;
    this->max_speeds[Z_AXIS]  = THEKERNEL->config->value(z_axis_max_speed_checksum    )->by_default( 1200.0F)->as_number() / 60.0F;

    Pin alpha_step_pin;
    Pin alpha_dir_pin;
    Pin alpha_en_pin;
    Pin beta_step_pin;
    Pin beta_dir_pin;
    Pin beta_en_pin;
    Pin gamma_step_pin;
    Pin gamma_dir_pin;
    Pin gamma_en_pin;

    Pin chi_step_pin;
	Pin chi_dir_pin;
	Pin chi_en_pin;
	Pin psi_step_pin;
	Pin psi_dir_pin;
	Pin psi_en_pin;
	Pin omega_step_pin;
	Pin omega_dir_pin;
	Pin omega_en_pin;

	string alpha_step_pin_str = THEKERNEL->config->value(alpha_step_pin_checksum )->by_default("2.0"  )->as_string();
	string alpha_dir_pin_str  = THEKERNEL->config->value(alpha_dir_pin_checksum  )->by_default("0.5!" )->as_string();
	string alpha_en_pin_str   = THEKERNEL->config->value(alpha_en_pin_checksum   )->by_default("0.4"  )->as_string();
	string beta_step_pin_str  = THEKERNEL->config->value(beta_step_pin_checksum  )->by_default("2.1"  )->as_string();
	string beta_dir_pin_str   = THEKERNEL->config->value(beta_dir_pin_checksum   )->by_default("0.11!")->as_string();
	string beta_en_pin_str    = THEKERNEL->config->value(beta_en_pin_checksum    )->by_default("0.10" )->as_string();
	string gamma_step_pin_str = THEKERNEL->config->value(gamma_step_pin_checksum )->by_default("2.2"  )->as_string();
	string gamma_dir_pin_str  = THEKERNEL->config->value(gamma_dir_pin_checksum  )->by_default("0.20" )->as_string();
	string gamma_en_pin_str   = THEKERNEL->config->value(gamma_en_pin_checksum   )->by_default("0.19" )->as_string();

	string chi_step_pin_str   = THEKERNEL->config->value(chi_step_pin_checksum   )->by_default("2.3"  )->as_string();
	string chi_dir_pin_str    = THEKERNEL->config->value(chi_dir_pin_checksum    )->by_default("0.22" )->as_string();
	string chi_en_pin_str     = THEKERNEL->config->value(chi_en_pin_checksum     )->by_default("0.21" )->as_string();
	string psi_step_pin_str   = THEKERNEL->config->value(psi_step_pin_checksum   )->by_default("2.8"  )->as_string();
	string psi_dir_pin_str    = THEKERNEL->config->value(psi_dir_pin_checksum    )->by_default("2.13" )->as_string();
	string psi_en_pin_str     = THEKERNEL->config->value(psi_en_pin_checksum     )->by_default("4.29" )->as_string();
	string omega_step_pin_str = THEKERNEL->config->value(omega_step_pin_checksum )->by_default("nc"   )->as_string();
	string omega_dir_pin_str  = THEKERNEL->config->value(omega_dir_pin_checksum  )->by_default("nc"   )->as_string();
	string omega_en_pin_str   = THEKERNEL->config->value(omega_en_pin_checksum   )->by_default("nc"   )->as_string();

    alpha_step_pin.from_string( alpha_step_pin_str)->as_output();
    alpha_dir_pin.from_string(  alpha_dir_pin_str)->as_output();
    alpha_en_pin.from_string(   alpha_en_pin_str)->as_output();
    beta_step_pin.from_string(  beta_step_pin_str)->as_output();
    beta_dir_pin.from_string(   beta_dir_pin_str)->as_output();
    beta_en_pin.from_string(    beta_en_pin_str)->as_output();
    gamma_step_pin.from_string( gamma_step_pin_str)->as_output();
    gamma_dir_pin.from_string(  gamma_dir_pin_str)->as_output();
    gamma_en_pin.from_string(   gamma_en_pin_str)->as_output();

    chi_step_pin.from_string(   chi_step_pin_str)->as_output();
    chi_dir_pin.from_string(    chi_dir_pin_str)->as_output();
    chi_en_pin.from_string(     chi_en_pin_str)->as_output();
    psi_step_pin.from_string(   psi_step_pin_str)->as_output();
    psi_dir_pin.from_string(    psi_dir_pin_str)->as_output();
    psi_en_pin.from_string(     psi_en_pin_str)->as_output();
    omega_step_pin.from_string( omega_step_pin_str)->as_output();
    omega_dir_pin.from_string(  omega_dir_pin_str)->as_output();
    omega_en_pin.from_string(   omega_en_pin_str)->as_output();


    float steps_per_mm[6] = {
        THEKERNEL->config->value(alpha_steps_per_mm_checksum)->by_default(  79.2F)->as_number(),
        THEKERNEL->config->value(beta_steps_per_mm_checksum )->by_default(  79.5F)->as_number(),
        THEKERNEL->config->value(gamma_steps_per_mm_checksum)->by_default(1068.7F)->as_number(),
        THEKERNEL->config->value(chi_steps_per_mm_checksum)->by_default(  1600.0F)->as_number(),
        THEKERNEL->config->value(psi_steps_per_mm_checksum)->by_default(  1600.0F)->as_number(),
        THEKERNEL->config->value(omega_steps_per_mm_checksum)->by_default(1600.0F)->as_number()
    };

    for(int i=0; i<6; i++) {
    	THEKERNEL->feedback->steps_per_mm[i] = steps_per_mm[i];
    }

    // TODO: delete or detect old steppermotors
    // Make our 3 StepperMotors
    this->alpha_stepper_motor  = THEKERNEL->step_ticker->add_stepper_motor( new StepperMotor(alpha_step_pin, alpha_dir_pin, alpha_en_pin) );
    this->beta_stepper_motor   = THEKERNEL->step_ticker->add_stepper_motor( new StepperMotor(beta_step_pin,  beta_dir_pin,  beta_en_pin ) );
    this->gamma_stepper_motor  = THEKERNEL->step_ticker->add_stepper_motor( new StepperMotor(gamma_step_pin, gamma_dir_pin, gamma_en_pin) );

    this->chi_stepper_motor    = THEKERNEL->step_ticker->add_stepper_motor( new StepperMotor(chi_step_pin,   chi_dir_pin,   chi_en_pin  ) );
    this->psi_stepper_motor    = THEKERNEL->step_ticker->add_stepper_motor( new StepperMotor(psi_step_pin,   psi_dir_pin,   psi_en_pin  ) );
    this->omega_stepper_motor  = THEKERNEL->step_ticker->add_stepper_motor( new StepperMotor(omega_step_pin, omega_dir_pin, omega_en_pin) );

    alpha_stepper_motor->change_steps_per_mm(steps_per_mm[0]);
    beta_stepper_motor->change_steps_per_mm(steps_per_mm[1]);
    gamma_stepper_motor->change_steps_per_mm(steps_per_mm[2]);

    chi_stepper_motor->change_steps_per_mm(steps_per_mm[3]);
	psi_stepper_motor->change_steps_per_mm(steps_per_mm[4]);
	omega_stepper_motor->change_steps_per_mm(steps_per_mm[5]);

    alpha_stepper_motor->max_rate = THEKERNEL->config->value(alpha_max_rate_checksum)->by_default(30000.0F)->as_number() / 60.0F;
    beta_stepper_motor->max_rate  = THEKERNEL->config->value(beta_max_rate_checksum )->by_default(30000.0F)->as_number() / 60.0F;
    gamma_stepper_motor->max_rate = THEKERNEL->config->value(gamma_max_rate_checksum)->by_default(1200.0F)->as_number() / 60.0F;

    chi_stepper_motor->max_rate   = THEKERNEL->config->value(chi_max_rate_checksum  )->by_default(190.0F)->as_number() / 60.0F;
    psi_stepper_motor->max_rate   = THEKERNEL->config->value(psi_max_rate_checksum  )->by_default(190.0F)->as_number() / 60.0F;
    omega_stepper_motor->max_rate = THEKERNEL->config->value(omega_max_rate_checksum)->by_default(190.0F)->as_number() / 60.0F;

    check_max_actuator_speeds(); // check the configs are sane

    actuators.clear();
    actuators.push_back(alpha_stepper_motor);
    actuators.push_back(beta_stepper_motor);
    actuators.push_back(gamma_stepper_motor);

    actuators.push_back(chi_stepper_motor);
	actuators.push_back(psi_stepper_motor);
	actuators.push_back(omega_stepper_motor);


    // initialise actuator positions to current cartesian position (X0 Y0 Z0)
    // so the first move can be correct if homing is not performed
    float actuator_pos[6];
    arm_solution->cartesian_to_actuator(last_milestone, actuator_pos);
    for (int i = 0; i < 6; i++){
        actuators[i]->change_last_milestone(actuator_pos[i]);
        actuators[i]->set_id(i);
    }

    //this->clearToolOffset();
}

// this does a sanity check that actuator speeds do not exceed steps rate capability
// we will override the actuator max_rate if the combination of max_rate and steps/sec exceeds base_stepping_frequency
void Robot::check_max_actuator_speeds()
{
    float step_freq = alpha_stepper_motor->max_rate * alpha_stepper_motor->get_steps_per_mm();
    if(step_freq > THEKERNEL->base_stepping_frequency) {
        alpha_stepper_motor->max_rate = floorf(THEKERNEL->base_stepping_frequency / alpha_stepper_motor->get_steps_per_mm());
        THEKERNEL->streams->printf("WARNING: alpha_max_rate exceeds base_stepping_frequency * alpha_steps_per_mm: %f, setting to %f\n", step_freq, alpha_stepper_motor->max_rate);
    }

    step_freq = beta_stepper_motor->max_rate * beta_stepper_motor->get_steps_per_mm();
    if(step_freq > THEKERNEL->base_stepping_frequency) {
        beta_stepper_motor->max_rate = floorf(THEKERNEL->base_stepping_frequency / beta_stepper_motor->get_steps_per_mm());
        THEKERNEL->streams->printf("WARNING: beta_max_rate exceeds base_stepping_frequency * beta_steps_per_mm: %f, setting to %f\n", step_freq, beta_stepper_motor->max_rate);
    }

    step_freq = gamma_stepper_motor->max_rate * gamma_stepper_motor->get_steps_per_mm();
    if(step_freq > THEKERNEL->base_stepping_frequency) {
        gamma_stepper_motor->max_rate = floorf(THEKERNEL->base_stepping_frequency / gamma_stepper_motor->get_steps_per_mm());
        THEKERNEL->streams->printf("WARNING: gamma_max_rate exceeds base_stepping_frequency * gamma_steps_per_mm: %f, setting to %f\n", step_freq, gamma_stepper_motor->max_rate);
    }

    step_freq = chi_stepper_motor->max_rate * chi_stepper_motor->get_steps_per_mm();
    if(step_freq > THEKERNEL->base_stepping_frequency) {
        chi_stepper_motor->max_rate = floorf(THEKERNEL->base_stepping_frequency / chi_stepper_motor->get_steps_per_mm());
        THEKERNEL->streams->printf("WARNING: chi_max_rate exceeds base_stepping_frequency * chi_steps_per_mm: %f, setting to %f\n", step_freq, chi_stepper_motor->max_rate);
    }

    step_freq = psi_stepper_motor->max_rate * psi_stepper_motor->get_steps_per_mm();
    if(step_freq > THEKERNEL->base_stepping_frequency) {
        psi_stepper_motor->max_rate = floorf(THEKERNEL->base_stepping_frequency / psi_stepper_motor->get_steps_per_mm());
        THEKERNEL->streams->printf("WARNING: psi_max_rate exceeds base_stepping_frequency * psi_steps_per_mm: %f, setting to %f\n", step_freq, psi_stepper_motor->max_rate);
    }

    step_freq = omega_stepper_motor->max_rate * omega_stepper_motor->get_steps_per_mm();
    if(step_freq > THEKERNEL->base_stepping_frequency) {
        omega_stepper_motor->max_rate = floorf(THEKERNEL->base_stepping_frequency / omega_stepper_motor->get_steps_per_mm());
        THEKERNEL->streams->printf("WARNING: omega_max_rate exceeds base_stepping_frequency * omega_steps_per_mm: %f, setting to %f\n", step_freq, omega_stepper_motor->max_rate);
    }
}

void Robot::on_halt(void *arg)
{
    halted = (arg == nullptr);
}

void Robot::on_get_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(robot_checksum)) return;

    if(pdr->second_element_is(speed_override_percent_checksum)) {
        static float return_data;
        return_data = 100.0F * 60.0F / seconds_per_minute;
        pdr->set_data_ptr(&return_data);
        pdr->set_taken();

    } else if(pdr->second_element_is(current_position_checksum)) {
        static float return_data[3];
        return_data[0] = from_millimeters(this->last_milestone[0]);
        return_data[1] = from_millimeters(this->last_milestone[1]);
        return_data[2] = from_millimeters(this->last_milestone[2]);

        pdr->set_data_ptr(&return_data);
        pdr->set_taken();
    }
}

void Robot::on_set_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(robot_checksum)) return;

    if(pdr->second_element_is(speed_override_percent_checksum)) {
        // NOTE do not use this while printing!
        float t = *static_cast<float *>(pdr->get_data_ptr());
        // enforce minimum 10% speed
        if (t < 10.0F) t = 10.0F;

        this->seconds_per_minute = t / 0.6F; // t * 60 / 100
        pdr->set_taken();
    } else if(pdr->second_element_is(current_position_checksum)) {
        float *t = static_cast<float *>(pdr->get_data_ptr());
        for (int i = 0; i < 3; i++) {
            this->last_milestone[i] = this->to_millimeters(t[i]);
        }

        float actuator_pos[6];
        arm_solution->cartesian_to_actuator(last_milestone, actuator_pos);
        for (int i = 0; i < 6; i++)
            actuators[i]->change_last_milestone(actuator_pos[i]);

        pdr->set_taken();
    }
}

void Robot::on_the_fly_get(void* argument)
{
	if(DEBUG) THEKERNEL->streams->printf("Robot::on_the_fly_get() called\r\n");

	PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

	if(pdr->starts_with(robot_checksum)){
		if(pdr->second_element_is(0) && pdr->third_element_is(0)){
			THEKERNEL->streams->printf("{\"default_feed_rate\":%f}\r\n",this->feed_rate);
			THEKERNEL->streams->printf("{\"a_default_feed_rate\":%f}\r\n",this->a_feed_rate);
			THEKERNEL->streams->printf("{\"b_default_feed_rate\":%f}\r\n",this->b_feed_rate);
			THEKERNEL->streams->printf("{\"c_default_feed_rate\":%f}\r\n",this->c_feed_rate);
			THEKERNEL->streams->printf("{\"default_seek_rate\":%f}\r\n",this->seek_rate);
			THEKERNEL->streams->printf("{\"a_default_seek_rate\":%f}\r\n",this->a_seek_rate);
			THEKERNEL->streams->printf("{\"b_default_seek_rate\":%f}\r\n",this->b_seek_rate);
			THEKERNEL->streams->printf("{\"c_default_seek_rate\":%f}\r\n",this->c_seek_rate);
			THEKERNEL->streams->printf("{\"mm_per_line_segment\":%f}\r\n",this->mm_per_line_segment);
			THEKERNEL->streams->printf("{\"delta_segments_per_second\":%f}\r\n",this->delta_segments_per_second);
			THEKERNEL->streams->printf("{\"mm_per_arc_segment\":%f}\r\n",this->mm_per_arc_segment);
			THEKERNEL->streams->printf("{\"arc_correction\":%i}\r\n",this->arc_correction);
			THEKERNEL->streams->printf("{\"x_axis_max_speed\":%f}\r\n",this->max_speeds[X_AXIS]);
			THEKERNEL->streams->printf("{\"y_axis_max_speed\":%f}\r\n",this->max_speeds[Y_AXIS]);
			THEKERNEL->streams->printf("{\"z_axis_max_speed\":%f}\r\n",this->max_speeds[Z_AXIS]);
			/* TODO: maybe... pin to string, instead of saving all the strings??? inverse of Pin::from_string()
			 * Pins are created here to create steppermotors and are then destroyed, so the real place
			 * for the change in with the steppermotors.
			 */
			THEKERNEL->streams->printf("{\"alpha_steps_per_mm\":%f}\r\n",alpha_stepper_motor->get_steps_per_mm());
			THEKERNEL->streams->printf("{\"beta_steps_per_mm\":%f}\r\n",beta_stepper_motor->get_steps_per_mm());
			THEKERNEL->streams->printf("{\"gamma_steps_per_mm\":%f}\r\n",gamma_stepper_motor->get_steps_per_mm());
			THEKERNEL->streams->printf("{\"chi_steps_per_mm\":%f}\r\n",chi_stepper_motor->get_steps_per_mm());
			THEKERNEL->streams->printf("{\"psi_steps_per_mm\":%f}\r\n",psi_stepper_motor->get_steps_per_mm());
			THEKERNEL->streams->printf("{\"omega_steps_per_mm\":%f}\r\n",omega_stepper_motor->get_steps_per_mm());
			THEKERNEL->streams->printf("{\"alpha_max_rate\":%f}\r\n",alpha_stepper_motor->max_rate*60.0F);
			THEKERNEL->streams->printf("{\"beta_max_rate\":%f}\r\n",beta_stepper_motor->max_rate*60.0F);
			THEKERNEL->streams->printf("{\"gamma_max_rate\":%f}\r\n",gamma_stepper_motor->max_rate*60.0F);
			THEKERNEL->streams->printf("{\"chi_max_rate\":%f}\r\n",chi_stepper_motor->max_rate*60.0F);
			THEKERNEL->streams->printf("{\"psi_max_rate\":%f}\r\n",psi_stepper_motor->max_rate*60.0F);
			THEKERNEL->streams->printf("{\"omega_max_rate\":%f}\r\n",omega_stepper_motor->max_rate*60.0F);
			pdr->set_taken();
		}
	} else if(pdr->starts_with(default_feed_rate_checksum)){
		THEKERNEL->streams->printf("{\"default_feed_rate\":%f}\r\n",this->feed_rate);
		pdr->set_taken();
	} else if(pdr->starts_with(a_default_feed_rate_checksum)){
		THEKERNEL->streams->printf("{\"a_default_feed_rate\":%f}\r\n",this->a_feed_rate);
		pdr->set_taken();
	} else if(pdr->starts_with(b_default_feed_rate_checksum)){
		THEKERNEL->streams->printf("{\"b_default_feed_rate\":%f}\r\n",this->b_feed_rate);
		pdr->set_taken();
	} else if(pdr->starts_with(c_default_feed_rate_checksum)){
		THEKERNEL->streams->printf("{\"c_default_feed_rate\":%f}\r\n",this->c_feed_rate);
		pdr->set_taken();

	} else if(pdr->starts_with(default_seek_rate_checksum)){
		THEKERNEL->streams->printf("{\"default_seek_rate\":%f}\r\n",this->seek_rate);
		pdr->set_taken();
	} else if(pdr->starts_with(a_default_seek_rate_checksum)){
		THEKERNEL->streams->printf("{\"a_default_seek_rate\":%f}\r\n",this->a_seek_rate);
		pdr->set_taken();
	} else if(pdr->starts_with(b_default_seek_rate_checksum)){
		THEKERNEL->streams->printf("{\"b_default_seek_rate\":%f}\r\n",this->b_seek_rate);
		pdr->set_taken();
	} else if(pdr->starts_with(c_default_seek_rate_checksum)){
		THEKERNEL->streams->printf("{\"c_default_seek_rate\":%f}\r\n",this->c_seek_rate);
		pdr->set_taken();

	} else if(pdr->starts_with(mm_per_line_segment_checksum)){
		THEKERNEL->streams->printf("{\"mm_per_line_segment\":%f}\r\n",this->mm_per_line_segment);
		pdr->set_taken();
	} else if(pdr->starts_with(delta_segments_per_second_checksum)){
		THEKERNEL->streams->printf("{\"delta_segments_per_second\":%f}\r\n",this->delta_segments_per_second);
		pdr->set_taken();
	} else if(pdr->starts_with(mm_per_arc_segment_checksum)){
		THEKERNEL->streams->printf("{\"mm_per_arc_segment\":%f}\r\n",this->mm_per_arc_segment);
		pdr->set_taken();
	} else if(pdr->starts_with(arc_correction_checksum)){
		THEKERNEL->streams->printf("{\"arc_correction\":%i}\r\n",this->arc_correction);
		pdr->set_taken();

	} else if(pdr->starts_with(x_axis_max_speed_checksum)){
		THEKERNEL->streams->printf("{\"x_axis_max_speed\":%f}\r\n",this->max_speeds[X_AXIS]);
		pdr->set_taken();
	} else if(pdr->starts_with(y_axis_max_speed_checksum)){
		THEKERNEL->streams->printf("{\"y_axis_max_speed\":%f}\r\n",this->max_speeds[Y_AXIS]);
		pdr->set_taken();
	} else if(pdr->starts_with(z_axis_max_speed_checksum)){
		THEKERNEL->streams->printf("{\"z_axis_max_speed\":%f}\r\n",this->max_speeds[Z_AXIS]);
		pdr->set_taken();

	} else if(pdr->starts_with(alpha_steps_per_mm_checksum)){
		THEKERNEL->streams->printf("{\"alpha_steps_per_mm\":%f}\r\n",alpha_stepper_motor->get_steps_per_mm());
		pdr->set_taken();
	} else if(pdr->starts_with(beta_steps_per_mm_checksum)){
		THEKERNEL->streams->printf("{\"beta_steps_per_mm\":%f}\r\n",beta_stepper_motor->get_steps_per_mm());
		pdr->set_taken();
	} else if(pdr->starts_with(gamma_steps_per_mm_checksum)){
		THEKERNEL->streams->printf("{\"gamma_steps_per_mm\":%f}\r\n",gamma_stepper_motor->get_steps_per_mm());
		pdr->set_taken();
	} else if(pdr->starts_with(chi_steps_per_mm_checksum)){
		THEKERNEL->streams->printf("{\"chi_steps_per_mm\":%f}\r\n",chi_stepper_motor->get_steps_per_mm());
		pdr->set_taken();
	} else if(pdr->starts_with(psi_steps_per_mm_checksum)){
		THEKERNEL->streams->printf("{\"psi_steps_per_mm\":%f}\r\n",psi_stepper_motor->get_steps_per_mm());
		pdr->set_taken();
	} else if(pdr->starts_with(omega_steps_per_mm_checksum)){
		THEKERNEL->streams->printf("{\"omega_steps_per_mm\":%f}\r\n",omega_stepper_motor->get_steps_per_mm());
		pdr->set_taken();

	} else if(pdr->starts_with(alpha_max_rate_checksum)){
		THEKERNEL->streams->printf("{\"alpha_max_rate\":%f}\r\n",alpha_stepper_motor->max_rate*60.0F);
		pdr->set_taken();
	} else if(pdr->starts_with(beta_max_rate_checksum)){
		THEKERNEL->streams->printf("{\"beta_max_rate\":%f}\r\n",beta_stepper_motor->max_rate*60.0F);
		pdr->set_taken();
	} else if(pdr->starts_with(gamma_max_rate_checksum)){
		THEKERNEL->streams->printf("{\"gamma_max_rate\":%f}\r\n",gamma_stepper_motor->max_rate*60.0F);
		pdr->set_taken();
	} else if(pdr->starts_with(chi_max_rate_checksum)){
		THEKERNEL->streams->printf("{\"chi_max_rate\":%f}\r\n",chi_stepper_motor->max_rate*60.0F);
		pdr->set_taken();
	} else if(pdr->starts_with(psi_max_rate_checksum)){
		THEKERNEL->streams->printf("{\"psi_max_rate\":%f}\r\n",psi_stepper_motor->max_rate*60.0F);
		pdr->set_taken();
	} else if(pdr->starts_with(omega_max_rate_checksum)){
		THEKERNEL->streams->printf("{\"omega_max_rate\":%f}\r\n",omega_stepper_motor->max_rate*60.0F);
		pdr->set_taken();
	}
}

void Robot::on_the_fly_set(void* argument)
{
	if(DEBUG) THEKERNEL->streams->printf("Robot::on_the_fly_set() called\r\n");

	PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);
	string str_value = *static_cast<string *>(pdr->get_data_ptr());

	if(pdr->starts_with(default_feed_rate_checksum)){
		this->feed_rate = strtof(str_value.c_str(), NULL);
		pdr->set_taken();
	} else if(pdr->starts_with(a_default_feed_rate_checksum)){
		this->a_feed_rate = strtof(str_value.c_str(), NULL);
		pdr->set_taken();
	} else if(pdr->starts_with(b_default_feed_rate_checksum)){
		this->b_feed_rate = strtof(str_value.c_str(), NULL);
		pdr->set_taken();
	} else if(pdr->starts_with(c_default_feed_rate_checksum)){
		this->c_feed_rate = strtof(str_value.c_str(), NULL);
		pdr->set_taken();

	} else if(pdr->starts_with(default_seek_rate_checksum)){
		this->seek_rate = strtof(str_value.c_str(), NULL);
		this->default_seek_rates[0] = this->seek_rate;
		pdr->set_taken();
	} else if(pdr->starts_with(a_default_seek_rate_checksum)){
		this->a_seek_rate = strtof(str_value.c_str(), NULL);
		this->default_seek_rates[1] = this->a_seek_rate;
		pdr->set_taken();
	} else if(pdr->starts_with(b_default_seek_rate_checksum)){
		this->b_seek_rate = strtof(str_value.c_str(), NULL);
		this->default_seek_rates[2] = this->b_seek_rate;
		pdr->set_taken();
	} else if(pdr->starts_with(c_default_seek_rate_checksum)){
		this->c_seek_rate = strtof(str_value.c_str(), NULL);
		this->default_seek_rates[3] = this->c_seek_rate;
		pdr->set_taken();

	} else if(pdr->starts_with(mm_per_line_segment_checksum)){
		this->mm_per_line_segment = strtof(str_value.c_str(), NULL);
		pdr->set_taken();
	} else if(pdr->starts_with(delta_segments_per_second_checksum)){
		this->delta_segments_per_second = strtof(str_value.c_str(), NULL);
		pdr->set_taken();
	} else if(pdr->starts_with(mm_per_line_segment_checksum)){
		this->mm_per_line_segment = strtof(str_value.c_str(), NULL);
		pdr->set_taken();
	} else if(pdr->starts_with(mm_per_arc_segment_checksum)){
		this->mm_per_arc_segment = strtof(str_value.c_str(), NULL);
		pdr->set_taken();
	} else if(pdr->starts_with(arc_correction_checksum)){
		this->arc_correction = strtof(str_value.c_str(), NULL);
		pdr->set_taken();

	} else if(pdr->starts_with(x_axis_max_speed_checksum)){
		this->max_speeds[X_AXIS] = strtof(str_value.c_str(), NULL);
		pdr->set_taken();
	} else if(pdr->starts_with(y_axis_max_speed_checksum)){
		this->max_speeds[Y_AXIS] = strtof(str_value.c_str(), NULL);
		pdr->set_taken();
	} else if(pdr->starts_with(z_axis_max_speed_checksum)){
		this->max_speeds[Z_AXIS] = strtof(str_value.c_str(), NULL);
		pdr->set_taken();

	} else if(pdr->starts_with(alpha_steps_per_mm_checksum)){
		alpha_stepper_motor->change_steps_per_mm( strtof(str_value.c_str(), NULL) );
		pdr->set_taken();
	} else if(pdr->starts_with(beta_steps_per_mm_checksum)){
		beta_stepper_motor->change_steps_per_mm( strtof(str_value.c_str(), NULL) );
		pdr->set_taken();
	} else if(pdr->starts_with(gamma_steps_per_mm_checksum)){
		gamma_stepper_motor->change_steps_per_mm( strtof(str_value.c_str(), NULL) );
		pdr->set_taken();
	} else if(pdr->starts_with(chi_steps_per_mm_checksum)){
		chi_stepper_motor->change_steps_per_mm( strtof(str_value.c_str(), NULL) );
		pdr->set_taken();
	} else if(pdr->starts_with(psi_steps_per_mm_checksum)){
		psi_stepper_motor->change_steps_per_mm( strtof(str_value.c_str(), NULL) );
		pdr->set_taken();
	} else if(pdr->starts_with(omega_steps_per_mm_checksum)){
		omega_stepper_motor->change_steps_per_mm( strtof(str_value.c_str(), NULL) );
		pdr->set_taken();

	} else if(pdr->starts_with(alpha_max_rate_checksum)){
		alpha_stepper_motor->max_rate = strtof(str_value.c_str(), NULL) / 60.0F;
		pdr->set_taken();
	} else if(pdr->starts_with(beta_max_rate_checksum)){
		beta_stepper_motor->max_rate = strtof(str_value.c_str(), NULL) / 60.0F;
		pdr->set_taken();
	} else if(pdr->starts_with(gamma_max_rate_checksum)){
		gamma_stepper_motor->max_rate = strtof(str_value.c_str(), NULL) / 60.0F;
		pdr->set_taken();
	} else if(pdr->starts_with(chi_max_rate_checksum)){
		chi_stepper_motor->max_rate = strtof(str_value.c_str(), NULL) / 60.0F;
		pdr->set_taken();
	} else if(pdr->starts_with(psi_max_rate_checksum)){
		psi_stepper_motor->max_rate = strtof(str_value.c_str(), NULL) / 60.0F;
		pdr->set_taken();
	} else if(pdr->starts_with(omega_max_rate_checksum)){
		omega_stepper_motor->max_rate = strtof(str_value.c_str(), NULL) / 60.0F;
		pdr->set_taken();
	}


}

//A GCode has been received
//See if the current Gcode line has some orders for us
void Robot::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    this->motion_mode = -1;

    //G-letter Gcodes are mostly what the Robot module is interested in, other modules also catch the gcode event and do stuff accordingly
    if( gcode->has_g) {
        switch( gcode->g ) {
            case 0:  this->motion_mode = MOTION_MODE_SEEK; gcode->mark_as_taken(); break;
            case 1:  this->motion_mode = MOTION_MODE_LINEAR; gcode->mark_as_taken();  break;
            case 2:  this->motion_mode = MOTION_MODE_CW_ARC; gcode->mark_as_taken();  break;
            case 3:  this->motion_mode = MOTION_MODE_CCW_ARC; gcode->mark_as_taken();  break;
            case 17: this->select_plane(X_AXIS, Y_AXIS, Z_AXIS); gcode->mark_as_taken();  break;
            case 18: this->select_plane(X_AXIS, Z_AXIS, Y_AXIS); gcode->mark_as_taken();  break;
            case 19: this->select_plane(Y_AXIS, Z_AXIS, X_AXIS); gcode->mark_as_taken();  break;
            case 20: this->inch_mode = true; gcode->mark_as_taken();  break;
            case 21: this->inch_mode = false; gcode->mark_as_taken();  break;
            case 90: this->absolute_mode = true; gcode->mark_as_taken();  break;
            case 91: this->absolute_mode = false; gcode->mark_as_taken();  break;
            case 92: {
                if(gcode->get_num_args() == 0) {
                    for (int i = X_AXIS; i <= Z_AXIS; ++i) {
                        reset_axis_position(0, i);
                    }

                } else {
                    for (char letter = 'X'; letter <= 'Z'; letter++) {
                        if ( gcode->has_letter(letter) ) {
                            reset_axis_position(this->to_millimeters(gcode->get_value(letter)), letter - 'X');
                        }
                    }
                    for (char letter = 'A'; letter <= 'C'; letter++) {
						if ( gcode->has_letter(letter) ) {
							reset_axis_position(this->to_millimeters(gcode->get_value(letter)), (letter - 'A')+3 );
						}
					}
                }

                gcode->mark_as_taken();

            }
            return;
        }
    } else if( gcode->has_m) {
        switch( gcode->m ) {
            case 92:{ // M92 - set steps per mm
                if (gcode->has_letter('X'))
                    actuators[0]->change_steps_per_mm(this->to_millimeters(gcode->get_value('X')));
                if (gcode->has_letter('Y'))
                    actuators[1]->change_steps_per_mm(this->to_millimeters(gcode->get_value('Y')));
                if (gcode->has_letter('Z'))
                    actuators[2]->change_steps_per_mm(this->to_millimeters(gcode->get_value('Z')));
                if (gcode->has_letter('F'))
                    seconds_per_minute = gcode->get_value('F');
                if(THEKERNEL->use_json) {
                	gcode->stream->printf("{\"M92\":{\"X\":%g,\"Y\":%g,\"Z\":%g,\"F\":%g}}", actuators[0]->steps_per_mm, actuators[1]->steps_per_mm, actuators[2]->steps_per_mm, seconds_per_minute);
                } else {
                	gcode->stream->printf("X:%g Y:%g Z:%g F:%g ", actuators[0]->steps_per_mm, actuators[1]->steps_per_mm, actuators[2]->steps_per_mm, seconds_per_minute);
                }
                gcode->add_nl = true;
                gcode->mark_as_taken();
                check_max_actuator_speeds();
            }
            return;
            case 114: {

                int n = 0;
                if(THEKERNEL->use_json){
                	char buf_j[64];
                	char buf_s[64];
                	char buf_o[64];
                	char buf_n[64];
                	n = snprintf(buf_j, sizeof(buf_j), "{\"M114\":{\"X\":%1.3f,\"Y\":%1.3f,\"Z\":%1.3f,",
									 from_millimeters(this->last_milestone[0]),
									 from_millimeters(this->last_milestone[1]),
									 from_millimeters(this->last_milestone[2]) );
                	gcode->txt_after_ok.append(buf_j, n);
                	n = snprintf(buf_s, sizeof(buf_s), "\"x\":%1.3f,\"y\":%1.3f,\"z\":%1.3f,",
									 actuators[X_AXIS]->get_current_position(),
									 actuators[Y_AXIS]->get_current_position(),
									 actuators[Z_AXIS]->get_current_position() );
                	gcode->txt_after_ok.append(buf_s, n);
                	n = snprintf(buf_o, sizeof(buf_o), "\"A\":%1.3f,\"B\":%1.3f,\"C\":%1.3f,",
                					 from_millimeters(this->last_milestone[3]),
									 from_millimeters(this->last_milestone[4]),
									 from_millimeters(this->last_milestone[5]) );
                	gcode->txt_after_ok.append(buf_o, n);
                	n = snprintf(buf_n, sizeof(buf_n), "\"a\":%1.3f,\"b\":%1.3f,\"c\":%1.3f}}\r\n",
                					 actuators[A_AXIS]->get_current_position(),
									 actuators[B_AXIS]->get_current_position(),
									 actuators[C_AXIS]->get_current_position() );
					gcode->txt_after_ok.append(buf_n, n);
                } else {
                	char buf[64];
					n = snprintf(buf, sizeof(buf), "C: X:%1.3f Y:%1.3f Z:%1.3f x:%1.3f y:%1.3f z:%1.3f\r\n",
									 from_millimeters(this->last_milestone[0]),
									 from_millimeters(this->last_milestone[1]),
									 from_millimeters(this->last_milestone[2]),
									 actuators[X_AXIS]->get_current_position(),
									 actuators[Y_AXIS]->get_current_position(),
									 actuators[Z_AXIS]->get_current_position() );
					gcode->txt_after_ok.append(buf, n);
					char abc_buf[64];
					int p = snprintf(abc_buf, sizeof(abc_buf), "C: A:%1.3f B:%1.3f C:%1.3f a:%1.3f b:%1.3f c:%1.3f\r\n",
									 from_millimeters(this->last_milestone[3]),
									 from_millimeters(this->last_milestone[4]),
									 from_millimeters(this->last_milestone[5]),
									 actuators[A_AXIS]->get_current_position(),
									 actuators[B_AXIS]->get_current_position(),
									 actuators[C_AXIS]->get_current_position() );
					gcode->txt_after_ok.append(abc_buf, p);
                }
                gcode->mark_as_taken();

            }
            return;
            case 198: {
            	if(THEKERNEL->use_json){
            		int n = 0;
            		bool use_comma = false;
            		if (gcode->has_letter('S')) n  = 1;
            		if (gcode->has_letter('A')) n += 2;
            		if (gcode->has_letter('B')) n += 4;
            		if (gcode->has_letter('C')) n += 8;

            		if(n>0) {
            			gcode->stream->printf("{\"M198\":{");
            			if(n==1 ||n==3 ||n==5 ||n==7 ||n==9 ||n==11||n==13||n==15) gcode->stream->printf("\"S\":%g", this->feed_rate); use_comma = true;
            			if(n==2 ||n==3 ||n==6 ||n==7 ||n==10||n==11||n==14||n==15){
            				if(use_comma) gcode->stream->printf(",\"A\":%g", this->a_feed_rate);
            				else 		  gcode->stream->printf( "\"A\":%g", this->a_feed_rate); use_comma = true;
            			}
            			if(n==4 ||n==5 ||n==6 ||n==7 ||n==12||n==14||n==15){
							if(use_comma) gcode->stream->printf(",\"B\":%g", this->b_feed_rate);
							else 		  gcode->stream->printf( "\"B\":%g", this->b_feed_rate); use_comma = true;
						}
            			if(n==8 ||n==9 ||n==10||n==11||n==12||n==13||n==14||n==15){
							if(use_comma) gcode->stream->printf(",\"C\":%g", this->c_feed_rate);
							else 		  gcode->stream->printf( "\"C\":%g", this->c_feed_rate);use_comma = true;
						}
            			gcode->stream->printf("}}");
            		}

            	} else {
            		if (gcode->has_letter('S'))
						gcode->stream->printf("S:%g ", this->feed_rate);
					if (gcode->has_letter('A'))
						gcode->stream->printf("A:%g ", this->a_feed_rate);
					if (gcode->has_letter('B'))
						gcode->stream->printf("B:%g ", this->b_feed_rate);
					if (gcode->has_letter('C'))
						gcode->stream->printf("C:%g ", this->c_feed_rate);
            	}
				gcode->add_nl = true;
				gcode->mark_as_taken();
			}
			break;
            case 199: {
            	if(THEKERNEL->use_json){
					int n = 0;
					bool use_comma = false;
					if (gcode->has_letter('S')) n  = 1;
					if (gcode->has_letter('A')) n += 2;
					if (gcode->has_letter('B')) n += 4;
					if (gcode->has_letter('C')) n += 8;

					if(n>0) {
						//TODO: refactor this w/"strcat"
						gcode->stream->printf("{\"M198\":{");
						if(n==1 ||n==3 ||n==5 ||n==7 ||n==9 ||n==11||n==13||n==15) gcode->stream->printf("\"S\":%g", this->seek_rate); use_comma = true;
						if(n==2 ||n==3 ||n==6 ||n==7 ||n==10||n==11||n==14||n==15){
							if(use_comma) gcode->stream->printf(",\"A\":%g", this->a_seek_rate);
							else 		  gcode->stream->printf( "\"A\":%g", this->a_seek_rate); use_comma = true;
						}
						if(n==4 ||n==5 ||n==6 ||n==7 ||n==12||n==14||n==15){
							if(use_comma) gcode->stream->printf(",\"B\":%g", this->b_seek_rate);
							else 		  gcode->stream->printf( "\"B\":%g", this->b_seek_rate); use_comma = true;
						}
						if(n==8 ||n==9 ||n==10||n==11||n==12||n==13||n==14||n==15){
							if(use_comma) gcode->stream->printf(",\"C\":%g", this->c_seek_rate);
							else 		  gcode->stream->printf( "\"C\":%g", this->c_seek_rate); use_comma = true;
						}
						gcode->stream->printf("}}");
					}

				} else {
					if (gcode->has_letter('S'))
						gcode->stream->printf("S:%g ", this->seek_rate);
					if (gcode->has_letter('A'))
						gcode->stream->printf("A:%g ", this->a_seek_rate);
					if (gcode->has_letter('B'))
						gcode->stream->printf("B:%g ", this->b_seek_rate);
					if (gcode->has_letter('C'))
						gcode->stream->printf("C:%g ", this->c_seek_rate);
				}
            	gcode->add_nl = true;
            	gcode->mark_as_taken();
            }
            break;
            case 203: { // M203 Set maximum feedrates in mm/sec
                if (gcode->has_letter('X'))
                    this->max_speeds[X_AXIS] = gcode->get_value('X');
                if (gcode->has_letter('Y'))
                    this->max_speeds[Y_AXIS] = gcode->get_value('Y');
                if (gcode->has_letter('Z'))
                    this->max_speeds[Z_AXIS] = gcode->get_value('Z');
                if (gcode->has_letter('A'))
                    alpha_stepper_motor->max_rate = gcode->get_value('A');
                if (gcode->has_letter('B'))
                    beta_stepper_motor->max_rate = gcode->get_value('B');
                if (gcode->has_letter('C'))
                    gamma_stepper_motor->max_rate = gcode->get_value('C');

                check_max_actuator_speeds();

                if(THEKERNEL->use_json){
                	gcode->stream->printf("{\"M203\":{\"X\":%g,\"Y\":%g,\"Z\":%g,\"A\":%g,\"B\":%g,\"C\":%g}}",
                						  this->max_speeds[X_AXIS], this->max_speeds[Y_AXIS], this->max_speeds[Z_AXIS],
                						  alpha_stepper_motor->max_rate, beta_stepper_motor->max_rate, gamma_stepper_motor->max_rate);
                } else {
					gcode->stream->printf("X:%g Y:%g Z:%g  A:%g B:%g C:%g ",
										  this->max_speeds[X_AXIS], this->max_speeds[Y_AXIS], this->max_speeds[Z_AXIS],
										  alpha_stepper_motor->max_rate, beta_stepper_motor->max_rate, gamma_stepper_motor->max_rate);
                }
                gcode->add_nl = true;
                gcode->mark_as_taken();

            }
            break;
            case 204:{ // M204 Snnn - set acceleration to nnn, Znnn sets z acceleration
                gcode->mark_as_taken();

                if (gcode->has_letter('S')) {
                    // TODO for safety so it applies only to following gcodes, maybe a better way to do this?
                    THEKERNEL->conveyor->wait_for_empty_queue();
                    float acc = gcode->get_value('S'); // mm/s^2
                    // enforce minimum
                    if (acc < 1.0F)
                        acc = 1.0F;
                    THEKERNEL->planner->acceleration = acc;
                }
                if (gcode->has_letter('Z')) {
                    // TODO for safety so it applies only to following gcodes, maybe a better way to do this?
                    THEKERNEL->conveyor->wait_for_empty_queue();
                    float acc = gcode->get_value('Z'); // mm/s^2
                    // enforce positive
                    if (acc < 0.0F)
                        acc = 0.0F;
                    THEKERNEL->planner->z_acceleration = acc;
                }
                if (gcode->has_letter('A')){
                	THEKERNEL->conveyor->wait_for_empty_queue();
                	float acc = gcode->get_value('A');
                	if (acc < 1.0F)
                		acc = 1.0F;
                	THEKERNEL->planner->a_acceleration = acc;
                }
                if (gcode->has_letter('B')){
					THEKERNEL->conveyor->wait_for_empty_queue();
					float acc = gcode->get_value('B');
					if (acc < 1.0F)
						acc = 1.0F;
					THEKERNEL->planner->b_acceleration = acc;
				}
                if (gcode->has_letter('C')){
					THEKERNEL->conveyor->wait_for_empty_queue();
					float acc = gcode->get_value('C');
					if (acc < 1.0F)
						acc = 1.0F;
					THEKERNEL->planner->c_acceleration = acc;
				}
                if(THEKERNEL->use_json) {
                	gcode->stream->printf("{\"M204\":{\"s\":%g,\"z\":%g,\"a\":%g,\"b\":%g,\"c\":%g}}",
                						  THEKERNEL->planner->acceleration, THEKERNEL->planner->z_acceleration, THEKERNEL->planner->a_acceleration,
                						  THEKERNEL->planner->b_acceleration, THEKERNEL->planner->c_acceleration );
                } else {
					gcode->stream->printf("s:%g z:%g  a:%g b:%g c:%g ",
										  THEKERNEL->planner->acceleration, THEKERNEL->planner->z_acceleration, THEKERNEL->planner->a_acceleration,
										  THEKERNEL->planner->b_acceleration, THEKERNEL->planner->c_acceleration );
                }
				gcode->add_nl = true;
            }
            break;
            case 205:{ // M205 Xnnn - set junction deviation, Z - set Z junction deviation, Snnn - Set minimum planner speed
                gcode->mark_as_taken();
                if (gcode->has_letter('X')) {
                    float jd = gcode->get_value('X');
                    // enforce minimum
                    if (jd < 0.0F)
                        jd = 0.0F;
                    THEKERNEL->planner->junction_deviation = jd;
                }
                if (gcode->has_letter('Z')) {
                    float jd = gcode->get_value('Z');
                    // enforce minimum, -1 disables it and uses regular junction deviation
                    if (jd < -1.0F)
                        jd = -1.0F;
                    THEKERNEL->planner->z_junction_deviation = jd;
                }
                if (gcode->has_letter('S')) {
                    float mps = gcode->get_value('S');
                    // enforce minimum
                    if (mps < 0.0F)
                        mps = 0.0F;
                    THEKERNEL->planner->minimum_planner_speed = mps;
                }
            }
            break;

            case 220:{ // M220 - speed override percentage
                gcode->mark_as_taken();
                if (gcode->has_letter('S')) {
                    float factor = gcode->get_value('S');
                    // enforce minimum 10% speed
                    if (factor < 10.0F)
                        factor = 10.0F;
                    // enforce maximum 10x speed
                    if (factor > 1000.0F)
                        factor = 1000.0F;

                    seconds_per_minute = 6000.0F / factor;
                }

            }
            break;
            case 400:{ // wait until all moves are done up to this point
                gcode->mark_as_taken();
                THEKERNEL->conveyor->wait_for_empty_queue();

            }
            break;
            case 500: // M500 saves some volatile settings to config override file
            case 503: { // M503 just prints the settings
                gcode->stream->printf(";Steps per unit:\nM92 X%1.5f Y%1.5f Z%1.5f\n", actuators[0]->steps_per_mm, actuators[1]->steps_per_mm, actuators[2]->steps_per_mm);
                gcode->stream->printf(";Acceleration mm/sec^2:\nM204 S%1.5f Z%1.5f\n", THEKERNEL->planner->acceleration, THEKERNEL->planner->z_acceleration);
                gcode->stream->printf(";X- Junction Deviation, Z- Z junction deviation, S - Minimum Planner speed:\nM205 X%1.5f Z%1.5f S%1.5f\n", THEKERNEL->planner->junction_deviation, THEKERNEL->planner->z_junction_deviation, THEKERNEL->planner->minimum_planner_speed);
                gcode->stream->printf(";Max feedrates in mm/sec, XYZ cartesian, ABC actuator:\nM203 X%1.5f Y%1.5f Z%1.5f A%1.5f B%1.5f C%1.5f\n",
                                      this->max_speeds[X_AXIS], this->max_speeds[Y_AXIS], this->max_speeds[Z_AXIS],
                                      alpha_stepper_motor->max_rate, beta_stepper_motor->max_rate, gamma_stepper_motor->max_rate);

                // get or save any arm solution specific optional values
                BaseSolution::arm_options_t options;
                if(arm_solution->get_optional(options) && !options.empty()) {
                    gcode->stream->printf(";Optional arm solution specific settings:\nM665");
                    for(auto &i : options) {
                        gcode->stream->printf(" %c%1.4f", i.first, i.second);
                    }
                    gcode->stream->printf("\n");
                }
                gcode->mark_as_taken();

            }
            break;
            case 665: { // M665 set optional arm solution variables based on arm solution.
                gcode->mark_as_taken();
                // the parameter args could be any letter except S so ask solution what options it supports
                BaseSolution::arm_options_t options;
                if(arm_solution->get_optional(options)) {
                    for(auto &i : options) {
                        // foreach optional value
                        char c = i.first;
                        if(gcode->has_letter(c)) { // set new value
                            i.second = gcode->get_value(c);
                        }
                        // print all current values of supported options
                        gcode->stream->printf("%c: %8.4f ", i.first, i.second);
                        gcode->add_nl = true;
                    }
                    // set the new options
                    arm_solution->set_optional(options);
                }

                // set delta segments per second, not saved by M500
                if(gcode->has_letter('S')) {
                    this->delta_segments_per_second = gcode->get_value('S');
                }

            }
            break;
        }
    }

    if( this->motion_mode < 0)
        return;

    //Get parameters
    float target[6], offset[3];
    clear_vector(offset);

    if(DEBUG) THEKERNEL->streams->printf("sizeof(target) = %i\r\n",sizeof(target));
    if(DEBUG) THEKERNEL->streams->printf("3.last_milestone:\r\n{");
	for(int i=0;i<5;i++){
		if(DEBUG) THEKERNEL->streams->printf("%f,",this->last_milestone[i]);
	}
	if(DEBUG) THEKERNEL->streams->printf("%f}\r\n",this->last_milestone[5]);
    memcpy(target, this->last_milestone, sizeof(target));    //default to last target
    //for(int i=0; i<6; i++) {
    //	target[i] = this->last_milestone[i];
    //}

    for(char letter = 'I'; letter <= 'K'; letter++) {
        if( gcode->has_letter(letter) ) {
            offset[letter - 'I'] = this->to_millimeters(gcode->get_value(letter));
        }
    }
    for(char letter = 'X'; letter <= 'Z'; letter++) {
        if( gcode->has_letter(letter) ) {
            target[letter - 'X'] = this->to_millimeters(gcode->get_value(letter)) + (this->absolute_mode ? this->toolOffset[letter - 'X'] : target[letter - 'X']);
        }
    }
    for(char letter = 'A'; letter <= 'C'; letter++) {
		if( gcode->has_letter(letter) ) {
			target[(letter - 'A') + 3] = this->to_millimeters(gcode->get_value(letter)) + (this->absolute_mode ? 0.0F  : target[(letter - 'A') + 3]);
		}
	}

    if( gcode->has_letter('F') ) {
        if( this->motion_mode == MOTION_MODE_SEEK )
            this->seek_rate = this->to_millimeters( gcode->get_value('F') );
        else
            this->feed_rate = this->to_millimeters( gcode->get_value('F') );
    }

    if( gcode->has_letter('a') ) {
		if( this->motion_mode == MOTION_MODE_SEEK )
			this->a_seek_rate = this->to_millimeters( gcode->get_value('a') );
		else
			this->a_feed_rate = this->to_millimeters( gcode->get_value('a') );
	}

	if( gcode->has_letter('b') ) {
		if( this->motion_mode == MOTION_MODE_SEEK )
			this->b_seek_rate = this->to_millimeters( gcode->get_value('b') );
		else
			this->b_feed_rate = this->to_millimeters( gcode->get_value('b') );
	}

	if( gcode->has_letter('c') ) {
		if( this->motion_mode == MOTION_MODE_SEEK )
			this->c_seek_rate = this->to_millimeters( gcode->get_value('c') );
		else
			this->c_feed_rate = this->to_millimeters( gcode->get_value('c') );
	}



    //Perform any physical actions
    switch(this->motion_mode) {
        case MOTION_MODE_CANCEL: break;
        case MOTION_MODE_SEEK  : this->append_line(gcode, target, this->seek_rate / seconds_per_minute, this->a_seek_rate / seconds_per_minute, this->b_seek_rate / seconds_per_minute, this->c_seek_rate / seconds_per_minute ); break;
        case MOTION_MODE_LINEAR: this->append_line(gcode, target, this->feed_rate / seconds_per_minute, this->a_feed_rate / seconds_per_minute, this->b_feed_rate / seconds_per_minute, this->c_feed_rate / seconds_per_minute ); break;
        case MOTION_MODE_CW_ARC:
        case MOTION_MODE_CCW_ARC: this->compute_arc(gcode, offset, target ); break;
    }

    // last_milestone was set to target in append_milestone, no need to do it again

}

// We received a new gcode, and one of the functions
// determined the distance for that given gcode. So now we can attach this gcode to the right block
// and continue
void Robot::distance_in_gcode_is_known(Gcode *gcode)
{
    //If the queue is empty, execute immediately, otherwise attach to the last added block
    THEKERNEL->conveyor->append_gcode(gcode);
}

// reset the position for all axis (used in homing for delta as last_milestone may be bogus)
void Robot::reset_axis_position(float x, float y, float z, float a, float b, float c)
{
    this->last_milestone[X_AXIS] = x;
    this->last_milestone[Y_AXIS] = y;
    this->last_milestone[Z_AXIS] = z;

    this->last_milestone[A_AXIS] = a;
	this->last_milestone[B_AXIS] = b;
	this->last_milestone[C_AXIS] = c;

    this->transformed_last_milestone[X_AXIS] = x;
    this->transformed_last_milestone[Y_AXIS] = y;
    this->transformed_last_milestone[Z_AXIS] = z;

    this->transformed_last_milestone[A_AXIS] = a;
	this->transformed_last_milestone[B_AXIS] = b;
	this->transformed_last_milestone[C_AXIS] = c;

    float actuator_pos[6];
    arm_solution->cartesian_to_actuator(this->last_milestone, actuator_pos);
    for (int i = 0; i < 6; i++)
        actuators[i]->change_last_milestone(actuator_pos[i]);
}

// Reset the position for an axis (used in homing and G92)
void Robot::reset_axis_position(float position, int axis)
{
    this->last_milestone[axis] = position;
    this->transformed_last_milestone[axis] = position;

    float actuator_pos[6];
    arm_solution->cartesian_to_actuator(this->last_milestone, actuator_pos);

    for (int i = 0; i < 6; i++) {
        actuators[i]->change_last_milestone(actuator_pos[i]);
        THEKERNEL->feedback->cumulative_steps[i] = actuator_pos[i] * THEKERNEL->feedback->steps_per_mm[i];
        THEKERNEL->feedback->cumulative_steps_last[i] = THEKERNEL->feedback->cumulative_steps[i]-1.0F;
    }
}

// Use FK to find out where actuator is and reset lastmilestone to match
void Robot::reset_position_from_current_actuator_position()
{
    float actuator_pos[]= {actuators[X_AXIS]->get_current_position(), actuators[Y_AXIS]->get_current_position(), actuators[Z_AXIS]->get_current_position(), actuators[A_AXIS]->get_current_position(), actuators[B_AXIS]->get_current_position(), actuators[C_AXIS]->get_current_position()};
    arm_solution->actuator_to_cartesian(actuator_pos, this->last_milestone);

    if(DEBUG) THEKERNEL->streams->printf("sizeof(this->t_l_m) = %i \r\n",sizeof(this->transformed_last_milestone));
    memcpy(this->transformed_last_milestone, this->last_milestone, sizeof(this->transformed_last_milestone));
    //for(int i=0; i<6; i++) {
    //	this->transformed_last_milestone[i] = this->last_milestone[i];
    //}
}

// Convert target from millimeters to steps, and append this to the planner
void Robot::append_milestone( float target[], float rate_mm_s, float a_rate_mm_s, float b_rate_mm_s, float c_rate_mm_s )
{
    float deltas[6];
    float unit_vec[3];
    float actuator_pos[6];
    float transformed_target[6]; // adjust target for bed compensation
    float millimeters_of_travel;

    float a_millimeters_of_travel, b_millimeters_of_travel, c_millimeters_of_travel;

    if(DEBUG) THEKERNEL->streams->printf("0.target:\r\n{");
	for(int i=0;i<5;i++){
		if(DEBUG) THEKERNEL->streams->printf("%f,",target[i]);
	}
	if(DEBUG) THEKERNEL->streams->printf("%f}\r\n",target[5]);

    // unity transform by default
    if(DEBUG) THEKERNEL->streams->printf("sizeof(transformed_target) = %i \r\n",sizeof(transformed_target));
    memcpy(transformed_target, target, sizeof(transformed_target));
    //for(int i=0; i<6; i++) {
    //	transformed_target[i] = target[i];
    //}

    // check function pointer and call if set to transform the target to compensate for bed
    if(compensationTransform) {
        // some compensation strategies can transform XYZ, some just change Z
        compensationTransform(transformed_target);
    }

    if(DEBUG) THEKERNEL->streams->printf("1.transformed_last_milestone:\r\n{");
	for(int i=0;i<5;i++){
		if(DEBUG) THEKERNEL->streams->printf("%f,",transformed_last_milestone[i]);
	}
	if(DEBUG) THEKERNEL->streams->printf("%f}\r\n",transformed_last_milestone[5]);


    // find distance moved by each axis, use transformed target from last_transformed_target
    for (int axis = X_AXIS; axis <= Z_AXIS; axis++){
        deltas[axis] = transformed_target[axis] - transformed_last_milestone[axis];
    }
    for (int axis = A_AXIS; axis <= C_AXIS; axis++){
		deltas[axis] = transformed_target[axis] - transformed_last_milestone[axis];
	}
    // store last transformed
    if(DEBUG) THEKERNEL->streams->printf("sizeof(t_l_m) = %i \r\n",sizeof(this->transformed_last_milestone));
    if(DEBUG) THEKERNEL->streams->printf("2.transformed_target:\r\n{");
    for(int i=0;i<5;i++){
    	if(DEBUG) THEKERNEL->streams->printf("%f,",transformed_target[i]);
    }
    if(DEBUG) THEKERNEL->streams->printf("%f}\r\n",transformed_target[5]);
    memcpy(this->transformed_last_milestone, transformed_target, sizeof(this->transformed_last_milestone));
    //for(int i=0; i<6; i++) {
    //	this->transformed_last_milestone[i] = transformed_target[i];
    //}

    // Compute how long this move moves, so we can attach it to the block for later use
    millimeters_of_travel = sqrtf( powf( deltas[X_AXIS], 2 ) +  powf( deltas[Y_AXIS], 2 ) +  powf( deltas[Z_AXIS], 2 ) );

    a_millimeters_of_travel = sqrtf( powf( deltas[A_AXIS], 2 ));
    b_millimeters_of_travel = sqrtf( powf( deltas[B_AXIS], 2 ));
    c_millimeters_of_travel = sqrtf( powf( deltas[C_AXIS], 2 ));

    // find distance unit vector
    for (int i = 0; i < 3; i++)
        unit_vec[i] = deltas[i] / millimeters_of_travel;

    // Do not move faster than the configured cartesian limits
    for (int axis = X_AXIS; axis <= Z_AXIS; axis++) {
        if ( max_speeds[axis] > 0 ) {
            float axis_speed = fabs(unit_vec[axis] * rate_mm_s);

            if (axis_speed > max_speeds[axis])
                rate_mm_s *= ( max_speeds[axis] / axis_speed );
        }
    }



    // find actuator position given cartesian position, use actual adjusted target
    arm_solution->cartesian_to_actuator( transformed_target, actuator_pos );
    if(DEBUG) THEKERNEL->streams->printf("actuator_pos:\r\n{");
    for(int i=0;i<5;i++){
		if(DEBUG) THEKERNEL->streams->printf("%f,",actuator_pos[i]);
	}
	if(DEBUG) THEKERNEL->streams->printf("%f}\r\n",actuator_pos[5]);
    // check per-actuator speed limits
    for (int actuator = 0; actuator <= 2; actuator++) {
        float actuator_rate  = fabs(actuator_pos[actuator] - actuators[actuator]->last_milestone_mm) * rate_mm_s / millimeters_of_travel;

        if (actuator_rate > actuators[actuator]->max_rate)
            rate_mm_s *= (actuators[actuator]->max_rate / actuator_rate);
    }

    // Append the block to the planner
    THEKERNEL->planner->append_block( actuator_pos, rate_mm_s, millimeters_of_travel, unit_vec, a_rate_mm_s, b_rate_mm_s, c_rate_mm_s, a_millimeters_of_travel, b_millimeters_of_travel, c_millimeters_of_travel );

    // Update the last_milestone to the current target for the next time we use last_milestone, use the requested target not the adjusted one

    if(DEBUG) THEKERNEL->streams->printf("sizeof(last_milestone) = %i\r\n", sizeof(this->last_milestone));
    memcpy(this->last_milestone, target, sizeof(this->last_milestone)); // this->last_milestone[] = target[];
    //for( int i=0;i<6;i++) {
    //	this->last_milestone[i] = target[i];
    //}
}

// Append a move to the queue ( cutting it into segments if needed )
void Robot::append_line(Gcode *gcode, float target[], float rate_mm_s, float a_rate_mm_s, float b_rate_mm_s, float c_rate_mm_s )
{

	if(DEBUG) THEKERNEL->streams->printf("-1.target:\r\n{");
	for(int i=0;i<5;i++){
		if(DEBUG) THEKERNEL->streams->printf("%f,",target[i]);
	}
	if(DEBUG) THEKERNEL->streams->printf("%f}\r\n",target[5]);
    // Find out the distance for this gcode
    gcode->millimeters_of_travel = powf( target[X_AXIS] - this->last_milestone[X_AXIS], 2 ) +  powf( target[Y_AXIS] - this->last_milestone[Y_AXIS], 2 ) +  powf( target[Z_AXIS] - this->last_milestone[Z_AXIS], 2 );

    float a_millimeters_of_travel, b_millimeters_of_travel, c_millimeters_of_travel;

    a_millimeters_of_travel = powf( target[A_AXIS] - this->last_milestone[A_AXIS], 2 );
    b_millimeters_of_travel = powf( target[B_AXIS] - this->last_milestone[B_AXIS], 2 );
    c_millimeters_of_travel = powf( target[C_AXIS] - this->last_milestone[C_AXIS], 2 );

    if(DEBUG)THEKERNEL->streams->printf("a_millimeters_of_travel = %f\r\n",a_millimeters_of_travel);
    if(DEBUG)THEKERNEL->streams->printf("b_millimeters_of_travel = %f\r\n",b_millimeters_of_travel);
    if(DEBUG)THEKERNEL->streams->printf("c_millimeters_of_travel = %f\r\n",c_millimeters_of_travel);

    // We ignore non-moves ( for example, extruder moves are not XYZ moves )
    if( gcode->millimeters_of_travel < 1e-8F && a_millimeters_of_travel < 1e-8F && b_millimeters_of_travel < 1e-8F && c_millimeters_of_travel < 1e-8F ) {
        if(THEKERNEL->feedback->machine_state == 0) {
        	THEKERNEL->feedback->flash_stat(1);
        	THEKERNEL->feedback->flash_stat(0);
        }
    	return;
    }
    if(THEKERNEL->feedback->machine_state == 0) {
    	//if(DEBUG) {
    	if(DEBUG) {
    		THEKERNEL->streams->printf("robot::append_line()=>");
    		THEKERNEL->streams->printf("machine_state: %i\r\n", THEKERNEL->feedback->machine_state);
    	}
    	THEKERNEL->feedback->flash_stat(1);
    	if(DEBUG) { THEKERNEL->streams->printf("after... machine_state: %i\r\n", THEKERNEL->feedback->machine_state); }
    }

    gcode->millimeters_of_travel = sqrtf(gcode->millimeters_of_travel);

    a_millimeters_of_travel = sqrtf(a_millimeters_of_travel);
	b_millimeters_of_travel = sqrtf(b_millimeters_of_travel);
	c_millimeters_of_travel = sqrtf(c_millimeters_of_travel);

    // Mark the gcode as having a known distance
    this->distance_in_gcode_is_known( gcode );

    // We cut the line into smaller segments. This is not usefull in a cartesian robot, but necessary for robots with rotational axes.
    // In cartesian robot, a high "mm_per_line_segment" setting will prevent waste.
    // In delta robots either mm_per_line_segment can be used OR delta_segments_per_second The latter is more efficient and avoids splitting fast long lines into very small segments, like initial z move to 0, it is what Johanns Marlin delta port does
    uint16_t segments;

    if(this->delta_segments_per_second > 1.0F) {
        // enabled if set to something > 1, it is set to 0.0 by default
        // segment based on current speed and requested segments per second
        // the faster the travel speed the fewer segments needed
        // NOTE rate is mm/sec and we take into account any speed override
        float seconds = gcode->millimeters_of_travel / rate_mm_s;
        segments = max(1, ceil(this->delta_segments_per_second * seconds));
        // TODO if we are only moving in Z on a delta we don't really need to segment at all

    } else {
        if(this->mm_per_line_segment == 0.0F) {
            segments = 1; // don't split it up
        } else {
            segments = ceil( gcode->millimeters_of_travel / this->mm_per_line_segment);
        }
    }

    if( a_millimeters_of_travel > 0.0F || b_millimeters_of_travel > 0.0F || c_millimeters_of_travel > 0.0F ){
		segments = 1;
	}

    if (segments > 1) {
        // A vector to keep track of the endpoint of each segment
        float segment_delta[6];
        float segment_end[6];

        // How far do we move each segment?
        for (int i = X_AXIS; i <= Z_AXIS; i++)
            segment_delta[i] = (target[i] - last_milestone[i]) / segments;

        // segment 0 is already done - it's the end point of the previous move so we start at segment 1
        // We always add another point after this loop so we stop at segments-1, ie i < segments
        for (int i = 1; i < segments; i++) {
            if(halted) return; // don;t queue any more segments
            for(int axis = X_AXIS; axis <= Z_AXIS; axis++ )
                segment_end[axis] = last_milestone[axis] + segment_delta[axis];
            for(int axis = A_AXIS; axis <= C_AXIS; axis++ )
            	segment_end[axis] = last_milestone[axis];

            // Append the end of this segment to the queue
            this->append_milestone(segment_end, rate_mm_s, a_rate_mm_s, b_rate_mm_s, c_rate_mm_s);
        }
    }
    if(halted) return;
    // Append the end of this full move to the queue
    this->append_milestone(target, rate_mm_s, a_rate_mm_s, b_rate_mm_s, c_rate_mm_s);

    // if adding these blocks didn't start executing, do that now
    THEKERNEL->conveyor->ensure_running();
}


// Append an arc to the queue ( cutting it into segments as needed )
void Robot::append_arc(Gcode *gcode, float target[], float offset[], float radius, bool is_clockwise )
{

    // Scary math
    float center_axis0 = this->last_milestone[this->plane_axis_0] + offset[this->plane_axis_0];
    float center_axis1 = this->last_milestone[this->plane_axis_1] + offset[this->plane_axis_1];
    float linear_travel = target[this->plane_axis_2] - this->last_milestone[this->plane_axis_2];
    float r_axis0 = -offset[this->plane_axis_0]; // Radius vector from center to current location
    float r_axis1 = -offset[this->plane_axis_1];
    float rt_axis0 = target[this->plane_axis_0] - center_axis0;
    float rt_axis1 = target[this->plane_axis_1] - center_axis1;

    // CCW angle between position and target from circle center. Only one atan2() trig computation required.
    float angular_travel = atan2(r_axis0 * rt_axis1 - r_axis1 * rt_axis0, r_axis0 * rt_axis0 + r_axis1 * rt_axis1);
    if (angular_travel < 0) {
        angular_travel += 2 * M_PI;
    }
    if (is_clockwise) {
        angular_travel -= 2 * M_PI;
    }

    // Find the distance for this gcode
    gcode->millimeters_of_travel = hypotf(angular_travel * radius, fabs(linear_travel));

    // We don't care about non-XYZ moves ( for example the extruder produces some of those )
    if( gcode->millimeters_of_travel < 0.0001F ) {
        return;
    }

    // Mark the gcode as having a known distance
    this->distance_in_gcode_is_known( gcode );

    // Figure out how many segments for this gcode
    uint16_t segments = floor(gcode->millimeters_of_travel / this->mm_per_arc_segment);

    float theta_per_segment = angular_travel / segments;
    float linear_per_segment = linear_travel / segments;

    /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
    and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
    r_T = [cos(phi) -sin(phi);
    sin(phi) cos(phi] * r ;
    For arc generation, the center of the circle is the axis of rotation and the radius vector is
    defined from the circle center to the initial position. Each line segment is formed by successive
    vector rotations. This requires only two cos() and sin() computations to form the rotation
    matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
    all float numbers are single precision on the Arduino. (True float precision will not have
    round off issues for CNC applications.) Single precision error can accumulate to be greater than
    tool precision in some cases. Therefore, arc path correction is implemented.

    Small angle approximation may be used to reduce computation overhead further. This approximation
    holds for everything, but very small circles and large mm_per_arc_segment values. In other words,
    theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
    to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
    numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
    issue for CNC machines with the single precision Arduino calculations.
    This approximation also allows mc_arc to immediately insert a line segment into the planner
    without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
    a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead.
    This is important when there are successive arc motions.
    */
    // Vector rotation matrix values
    float cos_T = 1 - 0.5F * theta_per_segment * theta_per_segment; // Small angle approximation
    float sin_T = theta_per_segment;

    float arc_target[3];
    float sin_Ti;
    float cos_Ti;
    float r_axisi;
    uint16_t i;
    int8_t count = 0;

    // Initialize the linear axis
    arc_target[this->plane_axis_2] = this->last_milestone[this->plane_axis_2];

    for (i = 1; i < segments; i++) { // Increment (segments-1)
        if(halted) return; // don't queue any more segments

        if (count < this->arc_correction ) {
            // Apply vector rotation matrix
            r_axisi = r_axis0 * sin_T + r_axis1 * cos_T;
            r_axis0 = r_axis0 * cos_T - r_axis1 * sin_T;
            r_axis1 = r_axisi;
            count++;
        } else {
            // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
            // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
            cos_Ti = cosf(i * theta_per_segment);
            sin_Ti = sinf(i * theta_per_segment);
            r_axis0 = -offset[this->plane_axis_0] * cos_Ti + offset[this->plane_axis_1] * sin_Ti;
            r_axis1 = -offset[this->plane_axis_0] * sin_Ti - offset[this->plane_axis_1] * cos_Ti;
            count = 0;
        }

        // Update arc_target location
        arc_target[this->plane_axis_0] = center_axis0 + r_axis0;
        arc_target[this->plane_axis_1] = center_axis1 + r_axis1;
        arc_target[this->plane_axis_2] += linear_per_segment;

        // Append this segment to the queue
        this->append_milestone(arc_target, this->feed_rate / seconds_per_minute, this->a_feed_rate / seconds_per_minute, this->b_feed_rate / seconds_per_minute, this->c_feed_rate / seconds_per_minute );

    }

    // Ensure last segment arrives at target location.
    this->append_milestone(target, this->feed_rate / seconds_per_minute, this->a_feed_rate / seconds_per_minute, this->b_feed_rate / seconds_per_minute, this->c_feed_rate / seconds_per_minute);
}

// Do the math for an arc and add it to the queue
void Robot::compute_arc(Gcode *gcode, float offset[], float target[])
{

    // Find the radius
    float radius = hypotf(offset[this->plane_axis_0], offset[this->plane_axis_1]);

    // Set clockwise/counter-clockwise sign for mc_arc computations
    bool is_clockwise = false;
    if( this->motion_mode == MOTION_MODE_CW_ARC ) {
        is_clockwise = true;
    }

    // Append arc
    this->append_arc(gcode, target, offset,  radius, is_clockwise );

}


float Robot::theta(float x, float y)
{
    float t = atanf(x / fabs(y));
    if (y > 0) {
        return(t);
    } else {
        if (t > 0) {
            return(M_PI - t);
        } else {
            return(-M_PI - t);
        }
    }
}

void Robot::select_plane(uint8_t axis_0, uint8_t axis_1, uint8_t axis_2)
{
    this->plane_axis_0 = axis_0;
    this->plane_axis_1 = axis_1;
    this->plane_axis_2 = axis_2;
}

void Robot::clearToolOffset()
{
    //memset(this->toolOffset, 0, sizeof(this->toolOffset));
	for(int i=0; i<3; i++) {
		this->toolOffset[i] = 0.0F;
	}
}

void Robot::setToolOffset(const float offset[6])
{
    //memcpy(this->toolOffset, offset, sizeof(this->toolOffset));
	for(int i=0; i<3; i++) {
		this->toolOffset[i] = offset[i];
	}
}

