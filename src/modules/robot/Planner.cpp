/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl) with additions from Sungeun K. Jeon (https://github.com/chamnit/grbl)
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

using namespace std;
#include <vector>

#include "mri.h"
#include "nuts_bolts.h"
#include "RingBuffer.h"
#include "Gcode.h"
#include "Module.h"
#include "Kernel.h"
#include "Block.h"
#include "Planner.h"
#include "Conveyor.h"
#include "StepperMotor.h"
#include "Config.h"
#include "checksumm.h"
#include "Robot.h"
#include "Stepper.h"
#include "ConfigValue.h"

#include <math.h>

#define acceleration_checksum          CHECKSUM("acceleration")
#define z_acceleration_checksum        CHECKSUM("z_acceleration")

#define a_acceleration_checksum		   CHECKSUM("a_acceleration")
#define b_acceleration_checksum		   CHECKSUM("b_acceleration")
#define c_acceleration_checksum		   CHECKSUM("c_acceleration")

#define max_jerk_checksum              CHECKSUM("max_jerk")
#define junction_deviation_checksum    CHECKSUM("junction_deviation")
#define z_junction_deviation_checksum  CHECKSUM("z_junction_deviation")
#define minimum_planner_speed_checksum CHECKSUM("minimum_planner_speed")

// The Planner does the acceleration math for the queue of Blocks ( movements ).
// It makes sure the speed stays within the configured constraints ( acceleration, junction_deviation, etc )
// It goes over the list in both direction, every time a block is added, re-doing the math to make sure everything is optimal

Planner::Planner(){
    clear_vector_float(this->previous_unit_vec);
    config_load();
}

// Configure acceleration
void Planner::config_load() {
    this->acceleration = THEKERNEL->config->value(acceleration_checksum)->by_default(500.0F )->as_number(); // Acceleration is in mm/s^2
    this->z_acceleration = THEKERNEL->config->value(z_acceleration_checksum)->by_default(300.0F )->as_number(); // disabled by default

    this->a_acceleration = THEKERNEL->config->value(a_acceleration_checksum)->by_default(16.0F )->as_number(); // disabled by default
    this->b_acceleration = THEKERNEL->config->value(b_acceleration_checksum)->by_default(16.0F )->as_number(); // disabled by default
    this->c_acceleration = THEKERNEL->config->value(c_acceleration_checksum)->by_default(16.0F )->as_number(); // disabled by default

    this->junction_deviation = THEKERNEL->config->value(junction_deviation_checksum)->by_default(0.05F)->as_number();
    this->z_junction_deviation = THEKERNEL->config->value(z_junction_deviation_checksum)->by_default(-1)->as_number(); // disabled by default
    this->minimum_planner_speed = THEKERNEL->config->value(minimum_planner_speed_checksum)->by_default(0.0f)->as_number();
}


// Append a block to the queue, compute it's speed factors
void Planner::append_block( float actuator_pos[], float rate_mm_s, float distance, float unit_vec[], float a_rate_mm_s, float b_rate_mm_s, float c_rate_mm_s, float a_distance, float b_distance, float c_distance )
{
    float acceleration, junction_deviation;

    float a_acceleration, b_acceleration, c_acceleration;

    // Create ( recycle ) a new block
    Block* block = THEKERNEL->conveyor->queue.head_ref();


    // Direction bits
    for (int i = 0; i < 6; i++)
    {
        int steps = THEKERNEL->robot->actuators[i]->steps_to_target(actuator_pos[i]);

        block->direction_bits[i] = (steps < 0) ? 1 : 0;

        // Update current position
        THEKERNEL->robot->actuators[i]->last_milestone_steps += steps;
        THEKERNEL->robot->actuators[i]->last_milestone_mm = actuator_pos[i];

        block->steps[i] = labs(steps);
    }

    acceleration = this->acceleration;

    a_acceleration = this->a_acceleration;
    b_acceleration = this->b_acceleration;
    c_acceleration = this->c_acceleration;

    junction_deviation = this->junction_deviation;

    // use either regular acceleration or a z only move accleration
    if(block->steps[ALPHA_STEPPER] == 0 && block->steps[BETA_STEPPER] == 0) {
        // z only move
        if(this->z_acceleration > 0.0F) acceleration= this->z_acceleration;
        if(this->z_junction_deviation >= 0.0F) junction_deviation= this->z_junction_deviation;
    }

    // Max number of steps, for all axes
    block->steps_event_count = max( block->steps[ALPHA_STEPPER], max( block->steps[BETA_STEPPER], block->steps[GAMMA_STEPPER] ) );

    block->millimeters = distance;

    block->a_millimeters = a_distance;
    block->b_millimeters = b_distance;
    block->c_millimeters = c_distance;

    // Calculate speed in mm/sec for each axis. No divide by zero due to previous checks.
    // NOTE: Minimum stepper speed is limited by MINIMUM_STEPS_PER_MINUTE in stepper.c
    if( distance > 0.0F ){
        block->nominal_speed = rate_mm_s;           // (mm/s) Always > 0
        block->nominal_rate = ceil(block->steps_event_count * rate_mm_s / distance); // (step/s) Always > 0
    }else{
        block->nominal_speed = 0.0F;
        block->nominal_rate  = 0;
    }

    if( a_distance > 0.0F ){
		block->a_nominal_speed = a_rate_mm_s;           // (mm/s) Always > 0
		block->a_nominal_rate = ceil(block->steps[CHI_STEPPER] * a_rate_mm_s / a_distance); // (step/s) Always > 0
	}else{
		block->a_nominal_speed = 0.0F;
		block->a_nominal_rate  = 0;
	}

    if( b_distance > 0.0F ){
		block->b_nominal_speed = b_rate_mm_s;           // (mm/s) Always > 0
		block->b_nominal_rate = ceil(block->steps[PSI_STEPPER] * b_rate_mm_s / b_distance); // (step/s) Always > 0
	}else{
		block->b_nominal_speed = 0.0F;
		block->b_nominal_rate  = 0;
	}

    if( c_distance > 0.0F ){
		block->c_nominal_speed = a_rate_mm_s;           // (mm/s) Always > 0
		block->c_nominal_rate = ceil(block->steps[OMEGA_STEPPER] * c_rate_mm_s / c_distance); // (step/s) Always > 0
	}else{
		block->c_nominal_speed = 0.0F;
		block->c_nominal_rate  = 0;
	}

    // Compute the acceleration rate for the trapezoid generator. Depending on the slope of the line
    // average travel per step event changes. For a line along one axis the travel per step event
    // is equal to the travel/step in the particular axis. For a 45 degree line the steppers of both
    // axes might step for every step event. Travel per step event is then sqrt(travel_x^2+travel_y^2).
    // To generate trapezoids with contant acceleration between blocks the rate_delta must be computed
    // specifically for each line to compensate for this phenomenon:
    // Convert universal acceleration for direction-dependent stepper rate change parameter
    block->rate_delta = (block->steps_event_count * acceleration) / (distance * THEKERNEL->stepper->get_acceleration_ticks_per_second()); // (step/min/acceleration_tick)

    block->a_rate_delta = (block->steps[CHI_STEPPER  ] * a_acceleration) / (a_distance * THEKERNEL->stepper->get_a_acceleration_ticks_per_second()); // (step/min/acceleration_tick)
    block->b_rate_delta = (block->steps[PSI_STEPPER  ] * b_acceleration) / (b_distance * THEKERNEL->stepper->get_b_acceleration_ticks_per_second()); // (step/min/acceleration_tick)
    block->c_rate_delta = (block->steps[OMEGA_STEPPER] * c_acceleration) / (c_distance * THEKERNEL->stepper->get_c_acceleration_ticks_per_second()); // (step/min/acceleration_tick)

    // Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
    // Let a circle be tangent to both previous and current path line segments, where the junction
    // deviation is defined as the distance from the junction to the closest edge of the circle,
    // colinear with the circle center. The circular segment joining the two paths represents the
    // path of centripetal acceleration. Solve for max velocity based on max acceleration about the
    // radius of the circle, defined indirectly by junction deviation. This may be also viewed as
    // path width or max_jerk in the previous grbl version. This approach does not actually deviate
    // from path, but used as a robust way to compute cornering speeds, as it takes into account the
    // nonlinearities of both the junction angle and junction velocity.
    float vmax_junction = minimum_planner_speed; // Set default max junction speed

    if (!THEKERNEL->conveyor->is_queue_empty())
    {
        float previous_nominal_speed = THEKERNEL->conveyor->queue.item_ref(THEKERNEL->conveyor->queue.prev(THEKERNEL->conveyor->queue.head_i))->nominal_speed;

        if (previous_nominal_speed > 0.0F && junction_deviation > 0.0F) {
            // Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
            // NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
            float cos_theta = - this->previous_unit_vec[X_AXIS] * unit_vec[X_AXIS]
                                - this->previous_unit_vec[Y_AXIS] * unit_vec[Y_AXIS]
                                - this->previous_unit_vec[Z_AXIS] * unit_vec[Z_AXIS] ;

            // Skip and use default max junction speed for 0 degree acute junction.
            if (cos_theta < 0.95F) {
                vmax_junction = min(previous_nominal_speed, block->nominal_speed);
                // Skip and avoid divide by zero for straight junctions at 180 degrees. Limit to min() of nominal speeds.
                if (cos_theta > -0.95F) {
                    // Compute maximum junction velocity based on maximum acceleration and junction deviation
                    float sin_theta_d2 = sqrtf(0.5F * (1.0F - cos_theta)); // Trig half angle identity. Always positive.
                    vmax_junction = min(vmax_junction, sqrtf(acceleration * junction_deviation * sin_theta_d2 / (1.0F - sin_theta_d2)));
                }
            }
        }
    }
    block->max_entry_speed = vmax_junction;

    block->a_max_entry_speed = block->a_nominal_speed;
    block->b_max_entry_speed = block->b_nominal_speed;
    block->c_max_entry_speed = block->c_nominal_speed;

    // Initialize block entry speed. Compute based on deceleration to user-defined minimum_planner_speed.
    float v_allowable = max_allowable_speed(-acceleration, minimum_planner_speed, block->millimeters); //TODO: Get from config
    block->entry_speed = min(vmax_junction, v_allowable);

    if (block->nominal_speed <= v_allowable) { block->nominal_length_flag = true; }
    else { block->nominal_length_flag = false; }

    v_allowable = max_allowable_speed(-a_acceleration, minimum_planner_speed, block->a_millimeters); //TODO: Get from config
    block->a_entry_speed = min(block->a_max_entry_speed, v_allowable);

    if (block->a_nominal_speed <= v_allowable) { block->a_nominal_length_flag = true; }
    else { block->a_nominal_length_flag = false; }

    v_allowable = max_allowable_speed(-b_acceleration, minimum_planner_speed, block->b_millimeters); //TODO: Get from config
    block->b_entry_speed = min(block->b_max_entry_speed, v_allowable);

    if (block->b_nominal_speed <= v_allowable) { block->b_nominal_length_flag = true; }
    else { block->b_nominal_length_flag = false; }

    v_allowable = max_allowable_speed(-c_acceleration, minimum_planner_speed, block->c_millimeters); //TODO: Get from config
    block->c_entry_speed = min(block->c_max_entry_speed, v_allowable);

    if (block->c_nominal_speed <= v_allowable) { block->c_nominal_length_flag = true; }
    else { block->c_nominal_length_flag = false; }

    // Initialize planner efficiency flags
    // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
    // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
    // the current block and next block junction speeds are guaranteed to always be at their maximum
    // junction speeds in deceleration and acceleration, respectively. This is due to how the current
    // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
    // the reverse and forward planners, the corresponding block junction speed will always be at the
    // the maximum junction speed and may always be ignored for any speed reduction checks.


    // Always calculate trapezoid for new block
    block->recalculate_flag = true;

    block->a_recalculate_flag = true;
    block->b_recalculate_flag = true;
    block->c_recalculate_flag = true;

    // Update previous path unit_vector and nominal speed
    memcpy(this->previous_unit_vec, unit_vec, sizeof(previous_unit_vec)); // previous_unit_vec[] = unit_vec[]

    // Math-heavy re-computing of the whole queue to take the new
    this->recalculate();

    // The block can now be used
    block->ready();

    THEKERNEL->conveyor->queue_head_block();
}

void Planner::recalculate() {
    Conveyor::Queue_t &queue = THEKERNEL->conveyor->queue;

    unsigned int block_index;

    Block* previous;
    Block* current;

    /*
     * a newly added block is decel limited
     *
     * we find its max entry speed given its exit speed
     *
     * for each block, walking backwards in the queue:
     *
     * if max entry speed == current entry speed
     * then we can set recalculate to false, since clearly adding another block didn't allow us to enter faster
     * and thus we don't need to check entry speed for this block any more
     *
     * once we find an accel limited block, we must find the max exit speed and walk the queue forwards
     *
     * for each block, walking forwards in the queue:
     *
     * given the exit speed of the previous block and our own max entry speed
     * we can tell if we're accel or decel limited (or coasting)
     *
     * if prev_exit > max_entry
     *     then we're still decel limited. update previous trapezoid with our max entry for prev exit
     * if max_entry >= prev_exit
     *     then we're accel limited. set recalculate to false, work out max exit speed
     *
     * finally, work out trapezoid for the final (and newest) block.
     */

    /*
     * Step 1:
     * For each block, given the exit speed and acceleration, find the maximum entry speed
     */

    float entry_speed = minimum_planner_speed;

    float a_entry_speed, b_entry_speed, c_entry_speed;
    a_entry_speed = b_entry_speed = c_entry_speed = entry_speed;

    block_index = queue.head_i;
    current     = queue.item_ref(block_index);

    if (!queue.is_empty())
    {
        while ((block_index != queue.tail_i) && (current->recalculate_flag || current->a_recalculate_flag || current->b_recalculate_flag || current->c_recalculate_flag) )
        {
        	if(current->recalculate_flag)
        		entry_speed = current->reverse_pass(entry_speed);

        	if(current->a_recalculate_flag)
        		a_entry_speed = current->reverse_pass(a_entry_speed);

        	if(current->b_recalculate_flag)
        		b_entry_speed = current->reverse_pass(b_entry_speed);

        	if(current->c_recalculate_flag)
        		c_entry_speed = current->reverse_pass(c_entry_speed);


        	block_index = queue.prev(block_index);
            current     = queue.item_ref(block_index);
        }

        /*
         * Step 2:
         * now current points to either tail or first non-recalculate block
         * and has not had its reverse_pass called
         * or its calc trap
         * entry_speed is set to the *exit* speed of current.
         * each block from current to head has its entry speed set to its max entry speed- limited by decel or nominal_rate
         */

        float exit_speed = current->max_exit_speed();

        float a_exit_speed = current->a_max_exit_speed();
        float b_exit_speed = current->b_max_exit_speed();
        float c_exit_speed = current->c_max_exit_speed();

        while (block_index != queue.head_i)
        {
            previous    = current;
            block_index = queue.next(block_index);
            current     = queue.item_ref(block_index);

            // we pass the exit speed of the previous block
            // so this block can decide if it's accel or decel limited and update its fields as appropriate
            exit_speed = current->forward_pass(exit_speed);

            a_exit_speed = current->a_forward_pass(a_exit_speed);
			b_exit_speed = current->b_forward_pass(b_exit_speed);
			c_exit_speed = current->c_forward_pass(c_exit_speed);

			previous->calculate_trapezoid(previous->entry_speed, current->entry_speed);

			previous->a_calculate_trapezoid(previous->a_entry_speed, current->a_entry_speed);
			previous->b_calculate_trapezoid(previous->b_entry_speed, current->b_entry_speed);
			previous->c_calculate_trapezoid(previous->c_entry_speed, current->c_entry_speed);
        }
    }

    /*
     * Step 3:
     * work out trapezoid for final (and newest) block
     */

    // now current points to the head item
    // which has not had calculate_trapezoid run yet
    current->calculate_trapezoid(current->entry_speed, minimum_planner_speed);

    current->a_calculate_trapezoid(current->a_entry_speed, minimum_planner_speed);
	current->b_calculate_trapezoid(current->b_entry_speed, minimum_planner_speed);
	current->c_calculate_trapezoid(current->c_entry_speed, minimum_planner_speed);
}


// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the
// acceleration within the allotted distance.
float Planner::max_allowable_speed(float acceleration, float target_velocity, float distance) {
  return(
    sqrtf(target_velocity*target_velocity-2.0F*acceleration*distance)  //Was acceleration*60*60*distance, in case this breaks, but here we prefer to use seconds instead of minutes
  );
}


