/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include <math.h>
#include <string>
#include "Block.h"
#include "Planner.h"
#include "Conveyor.h"
#include "Gcode.h"
#include "libs/StreamOutputPool.h"
#include "Stepper.h"

#include "mri.h"

using std::string;
#include <vector>

// A block represents a movement, it's length for each stepper motor, and the corresponding acceleration curves.
// It's stacked on a queue, and that queue is then executed in order, to move the motors.
// Most of the accel math is also done in this class
// And GCode objects for use in on_gcode_execute are also help in here

Block::Block()
{
    clear();
}

void Block::clear()
{
    //commands.clear();
    //travel_distances.clear();
    gcodes.clear();
    std::vector<Gcode>().swap(gcodes); // this resizes the vector releasing its memory

    clear_vector(this->steps);

    steps_event_count   = 0;
    nominal_rate        = 0;

    a_nominal_rate        = 0;
    b_nominal_rate        = 0;
    c_nominal_rate        = 0;

    nominal_speed       = 0.0F;

    a_nominal_speed       = 0.0F;
    b_nominal_speed       = 0.0F;
    c_nominal_speed       = 0.0F;

    millimeters         = 0.0F;

    a_millimeters         = 0.0F;
    b_millimeters         = 0.0F;
    c_millimeters         = 0.0F;

    entry_speed         = 0.0F;

    a_entry_speed         = 0.0F;
    b_entry_speed         = 0.0F;
    c_entry_speed         = 0.0F;

    exit_speed          = 0.0F;

    a_exit_speed          = 0.0F;
    b_exit_speed          = 0.0F;
    c_exit_speed          = 0.0F;

    rate_delta          = 0.0F;

    a_rate_delta          = 0.0F;
    b_rate_delta          = 0.0F;
    c_rate_delta          = 0.0F;

    initial_rate        = -1;

    a_initial_rate        = -1;
    b_initial_rate        = -1;
    c_initial_rate        = -1;

    final_rate          = -1;

    a_final_rate          = -1;
    b_final_rate          = -1;
    c_final_rate          = -1;

    accelerate_until    = 0;

    a_accelerate_until    = 0;
    b_accelerate_until    = 0;
    c_accelerate_until    = 0;

    decelerate_after    = 0;

    a_decelerate_after    = 0;
    b_decelerate_after    = 0;
    c_decelerate_after    = 0;

    direction_bits      = 0;
    recalculate_flag    = false;

    a_recalculate_flag    = false;
    b_recalculate_flag    = false;
    c_recalculate_flag    = false;

    nominal_length_flag = false;

    a_nominal_length_flag = false;
    b_nominal_length_flag = false;
    c_nominal_length_flag = false;

    max_entry_speed     = 0.0F;

    a_max_entry_speed     = 0.0F;
    b_max_entry_speed     = 0.0F;
    c_max_entry_speed     = 0.0F;

    is_ready            = false;
    times_taken         = 0;
}

void Block::debug()
{
    THEKERNEL->streams->printf("%p: steps:X%04d Y%04d Z%04d(max:%4d) nominal:r%10d/s%6.1f mm:%9.6f rdelta:%8f acc:%5d dec:%5d rates:%10d>%10d  entry/max: %10.4f/%10.4f taken:%d ready:%d recalc:%d nomlen:%d\r\n",
                               this,
                                         this->steps[0],
                                               this->steps[1],
                                                      this->steps[2],
                                                               this->steps_event_count,
                                                                             this->nominal_rate,
                                                                                   this->nominal_speed,
                                                                                            this->millimeters,
                                                                                                         this->rate_delta,
                                                                                                                 this->accelerate_until,
                                                                                                                         this->decelerate_after,
                                                                                                                                   this->initial_rate,
                                                                                                                                        this->final_rate,
                                                                                                                                                          this->entry_speed,
                                                                                                                                                                this->max_entry_speed,
                                                                                                                                                                             this->times_taken,
                                                                                                                                                                                      this->is_ready,
                                                                                                                                                                                                recalculate_flag?1:0,
                                                                                                                                                                                                          nominal_length_flag?1:0
                             );
}


/* Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.
// The factors represent a factor of braking and must be in the range 0.0-1.0.
//                                +--------+ <- nominal_rate
//                               /          \
// nominal_rate*entry_factor -> +            \
//                              |             + <- nominal_rate*exit_factor
//                              +-------------+
//                                  time -->
*/
void Block::calculate_trapezoid( float entryspeed, float exitspeed )
{
    // if block is currently executing, don't touch anything!
    if (times_taken)
        return;

    // The planner passes us factors, we need to transform them in rates
    this->initial_rate = ceil(this->nominal_rate * entryspeed / this->nominal_speed);   // (step/s)
    this->final_rate   = ceil(this->nominal_rate * exitspeed  / this->nominal_speed);   // (step/s)

    // How many steps to accelerate and decelerate
    float acceleration_per_second = this->rate_delta * THEKERNEL->stepper->get_acceleration_ticks_per_second(); // ( step/s^2)
    int accelerate_steps = ceil( this->estimate_acceleration_distance( this->initial_rate, this->nominal_rate, acceleration_per_second ) );
    int decelerate_steps = floor( this->estimate_acceleration_distance( this->nominal_rate, this->final_rate,  -acceleration_per_second ) );

    // Calculate the size of Plateau of Nominal Rate ( during which we don't accelerate nor decelerate, but just cruise )
    int plateau_steps = this->steps_event_count - accelerate_steps - decelerate_steps;

    // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
    // have to use intersection_distance() to calculate when to abort acceleration and start braking
    // in order to reach the final_rate exactly at the end of this block.
    if (plateau_steps < 0) {
        accelerate_steps = ceil(this->intersection_distance(this->initial_rate, this->final_rate, acceleration_per_second, this->steps_event_count));
        accelerate_steps = max( accelerate_steps, 0 ); // Check limits due to numerical round-off
        accelerate_steps = min( accelerate_steps, int(this->steps_event_count) );
        plateau_steps = 0;
    }
    this->accelerate_until = accelerate_steps;
    this->decelerate_after = accelerate_steps + plateau_steps;

    this->exit_speed = exitspeed;
}

void Block::a_calculate_trapezoid( float entryspeed, float exitspeed )
{
    // if block is currently executing, don't touch anything!
    if (times_taken)
        return;

    // The planner passes us factors, we need to transform them in rates
    this->a_initial_rate = ceil(this->a_nominal_rate * entryspeed / this->a_nominal_speed);   // (step/s)
    this->a_final_rate   = ceil(this->a_nominal_rate * exitspeed  / this->a_nominal_speed);   // (step/s)

    // How many steps to accelerate and decelerate
    float acceleration_per_second = this->a_rate_delta * THEKERNEL->stepper->get_a_acceleration_ticks_per_second(); // ( step/s^2)
    int accelerate_steps = ceil( this->estimate_acceleration_distance( this->a_initial_rate, this->a_nominal_rate, acceleration_per_second ) );
    int decelerate_steps = floor( this->estimate_acceleration_distance( this->a_nominal_rate, this->a_final_rate,  -acceleration_per_second ) );

    // Calculate the size of Plateau of Nominal Rate ( during which we don't accelerate nor decelerate, but just cruise )
    int plateau_steps = this->steps[A_AXIS] - accelerate_steps - decelerate_steps;

    // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
    // have to use intersection_distance() to calculate when to abort acceleration and start braking
    // in order to reach the final_rate exactly at the end of this block.
    if (plateau_steps < 0) {
        accelerate_steps = ceil(this->intersection_distance(this->a_initial_rate, this->a_final_rate, acceleration_per_second, this->steps[A_AXIS]));
        accelerate_steps = max( accelerate_steps, 0 ); // Check limits due to numerical round-off
        accelerate_steps = min( accelerate_steps, int(this->steps[A_AXIS]) );
        plateau_steps = 0;
    }
    this->a_accelerate_until = accelerate_steps;
    this->a_decelerate_after = accelerate_steps + plateau_steps;

    this->a_exit_speed = exitspeed;
}

void Block::b_calculate_trapezoid( float entryspeed, float exitspeed )
{
    // if block is currently executing, don't touch anything!
    if (times_taken)
        return;

    // The planner passes us factors, we need to transform them in rates
    this->b_initial_rate = ceil(this->b_nominal_rate * entryspeed / this->b_nominal_speed);   // (step/s)
    this->b_final_rate   = ceil(this->b_nominal_rate * exitspeed  / this->b_nominal_speed);   // (step/s)

    // How many steps to accelerate and decelerate
    float acceleration_per_second = this->b_rate_delta * THEKERNEL->stepper->get_a_acceleration_ticks_per_second(); // ( step/s^2)
    int accelerate_steps = ceil( this->estimate_acceleration_distance( this->b_initial_rate, this->b_nominal_rate, acceleration_per_second ) );
    int decelerate_steps = floor( this->estimate_acceleration_distance( this->b_nominal_rate, this->b_final_rate,  -acceleration_per_second ) );

    // Calculate the size of Plateau of Nominal Rate ( during which we don't accelerate nor decelerate, but just cruise )
    int plateau_steps = this->steps[B_AXIS] - accelerate_steps - decelerate_steps;

    // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
    // have to use intersection_distance() to calculate when to abort acceleration and start braking
    // in order to reach the final_rate exactly at the end of this block.
    if (plateau_steps < 0) {
        accelerate_steps = ceil(this->intersection_distance(this->b_initial_rate, this->b_final_rate, acceleration_per_second, this->steps[B_AXIS]));
        accelerate_steps = max( accelerate_steps, 0 ); // Check limits due to numerical round-off
        accelerate_steps = min( accelerate_steps, int(this->steps[B_AXIS]) );
        plateau_steps = 0;
    }
    this->b_accelerate_until = accelerate_steps;
    this->b_decelerate_after = accelerate_steps + plateau_steps;

    this->b_exit_speed = exitspeed;
}

void Block::c_calculate_trapezoid( float entryspeed, float exitspeed )
{
    // if block is currently executing, don't touch anything!
    if (times_taken)
        return;

    // The planner passes us factors, we need to transform them in rates
    this->c_initial_rate = ceil(this->c_nominal_rate * entryspeed / this->c_nominal_speed);   // (step/s)
    this->c_final_rate   = ceil(this->c_nominal_rate * exitspeed  / this->c_nominal_speed);   // (step/s)

    // How many steps to accelerate and decelerate
    float acceleration_per_second = this->c_rate_delta * THEKERNEL->stepper->get_acceleration_ticks_per_second(); // ( step/s^2)
    int accelerate_steps = ceil( this->estimate_acceleration_distance( this->c_initial_rate, this->c_nominal_rate, acceleration_per_second ) );
    int decelerate_steps = floor( this->estimate_acceleration_distance( this->c_nominal_rate, this->c_final_rate,  -acceleration_per_second ) );

    // Calculate the size of Plateau of Nominal Rate ( during which we don't accelerate nor decelerate, but just cruise )
    int plateau_steps = this->steps[C_AXIS] - accelerate_steps - decelerate_steps;

    // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
    // have to use intersection_distance() to calculate when to abort acceleration and start braking
    // in order to reach the final_rate exactly at the end of this block.
    if (plateau_steps < 0) {
        accelerate_steps = ceil(this->intersection_distance(this->c_initial_rate, this->c_final_rate, acceleration_per_second, this->steps[C_AXIS]));
        accelerate_steps = max( accelerate_steps, 0 ); // Check limits due to numerical round-off
        accelerate_steps = min( accelerate_steps, int(this->steps[C_AXIS]) );
        plateau_steps = 0;
    }
    this->c_accelerate_until = accelerate_steps;
    this->c_decelerate_after = accelerate_steps + plateau_steps;

    this->c_exit_speed = exitspeed;
}


// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the
// given acceleration:
float Block::estimate_acceleration_distance(float initialrate, float targetrate, float acceleration)
{
    return( ((targetrate * targetrate) - (initialrate * initialrate)) / (2.0F * acceleration));
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)
//
/*                          + <- some maximum rate we don't care about
                           /|\
                          / | \
                         /  |  + <- final_rate
                        /   |  |
       initial_rate -> +----+--+
                            ^ ^
                            | |
        intersection_distance distance */
float Block::intersection_distance(float initialrate, float finalrate, float acceleration, float distance)
{
    return((2 * acceleration * distance - initialrate * initialrate + finalrate * finalrate) / (4 * acceleration));
}

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the
// acceleration within the allotted distance.
float Block::max_allowable_speed(float acceleration, float target_velocity, float distance)
{
    return sqrtf(target_velocity * target_velocity - 2.0F * acceleration * distance);
}


// Called by Planner::recalculate() when scanning the plan from last to first entry.
float Block::reverse_pass(float exit_speed)
{
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (this->entry_speed != this->max_entry_speed)
    {
        // If nominal length true, max junction speed is guaranteed to be reached. Only compute
        // for max allowable speed if block is decelerating and nominal length is false.
        if ((!this->nominal_length_flag) && (this->max_entry_speed > exit_speed))
        {
            float max_entry_speed = max_allowable_speed(-THEKERNEL->planner->get_acceleration(), exit_speed, this->millimeters);

            this->entry_speed = min(max_entry_speed, this->max_entry_speed);

            return this->entry_speed;
        }
        else
            this->entry_speed = this->max_entry_speed;
    }

    return this->entry_speed;
}

// Called by Planner::recalculate() when scanning the plan from last to first entry.
float Block::a_reverse_pass(float exit_speed)
{
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (this->a_entry_speed != this->a_max_entry_speed)
    {
        // If nominal length true, max junction speed is guaranteed to be reached. Only compute
        // for max allowable speed if block is decelerating and nominal length is false.
        if ((!this->a_nominal_length_flag) && (this->a_max_entry_speed > exit_speed))
        {
            float max_entry_speed = max_allowable_speed(-THEKERNEL->planner->get_a_acceleration(), exit_speed, this->a_millimeters);

            this->a_entry_speed = min(max_entry_speed, this->a_max_entry_speed);

            return this->a_entry_speed;
        }
        else
            this->a_entry_speed = this->a_max_entry_speed;
    }

    return this->a_entry_speed;
}

// Called by Planner::recalculate() when scanning the plan from last to first entry.
float Block::b_reverse_pass(float exit_speed)
{
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (this->b_entry_speed != this->b_max_entry_speed)
    {
        // If nominal length true, max junction speed is guaranteed to be reached. Only compute
        // for max allowable speed if block is decelerating and nominal length is false.
        if ((!this->b_nominal_length_flag) && (this->b_max_entry_speed > exit_speed))
        {
            float max_entry_speed = max_allowable_speed(-THEKERNEL->planner->get_b_acceleration(), exit_speed, this->b_millimeters);

            this->b_entry_speed = min(max_entry_speed, this->b_max_entry_speed);

            return this->b_entry_speed;
        }
        else
            this->b_entry_speed = this->b_max_entry_speed;
    }

    return this->b_entry_speed;
}

// Called by Planner::recalculate() when scanning the plan from last to first entry.
float Block::c_reverse_pass(float exit_speed)
{
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (this->c_entry_speed != this->c_max_entry_speed)
    {
        // If nominal length true, max junction speed is guaranteed to be reached. Only compute
        // for max allowable speed if block is decelerating and nominal length is false.
        if ((!this->c_nominal_length_flag) && (this->c_max_entry_speed > exit_speed))
        {
            float max_entry_speed = max_allowable_speed(-THEKERNEL->planner->get_c_acceleration(), exit_speed, this->c_millimeters);

            this->c_entry_speed = min(max_entry_speed, this->c_max_entry_speed);

            return this->c_entry_speed;
        }
        else
            this->c_entry_speed = this->c_max_entry_speed;
    }

    return this->c_entry_speed;
}

// Called by Planner::recalculate() when scanning the plan from first to last entry.
// returns maximum exit speed of this block
float Block::forward_pass(float prev_max_exit_speed)
{
    // If the previous block is an acceleration block, but it is not long enough to complete the
    // full speed change within the block, we need to adjust the entry speed accordingly. Entry
    // speeds have already been reset, maximized, and reverse planned by reverse planner.
    // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.

    // TODO: find out if both of these checks are necessary
    if (prev_max_exit_speed > nominal_speed)
        prev_max_exit_speed = nominal_speed;
    if (prev_max_exit_speed > max_entry_speed)
        prev_max_exit_speed = max_entry_speed;

    if (prev_max_exit_speed <= entry_speed)
    {
        // accel limited
        entry_speed = prev_max_exit_speed;
        // since we're now acceleration or cruise limited
        // we don't need to recalculate our entry speed anymore
        recalculate_flag = false;
    }
    // else
    // // decel limited, do nothing

    return max_exit_speed();
}

// Called by Planner::recalculate() when scanning the plan from first to last entry.
// returns maximum exit speed of this block
float Block::a_forward_pass(float prev_max_exit_speed)
{
    // If the previous block is an acceleration block, but it is not long enough to complete the
    // full speed change within the block, we need to adjust the entry speed accordingly. Entry
    // speeds have already been reset, maximized, and reverse planned by reverse planner.
    // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.

    // TODO: find out if both of these checks are necessary
    if (prev_max_exit_speed > a_nominal_speed)
        prev_max_exit_speed = a_nominal_speed;
    if (prev_max_exit_speed > a_max_entry_speed)
        prev_max_exit_speed = a_max_entry_speed;

    if (prev_max_exit_speed <= a_entry_speed)
    {
        // accel limited
        a_entry_speed = prev_max_exit_speed;
        // since we're now acceleration or cruise limited
        // we don't need to recalculate our entry speed anymore
        a_recalculate_flag = false;
    }
    // else
    // // decel limited, do nothing

    return max_exit_speed();
}

// Called by Planner::recalculate() when scanning the plan from first to last entry.
// returns maximum exit speed of this block
float Block::b_forward_pass(float prev_max_exit_speed)
{
    // If the previous block is an acceleration block, but it is not long enough to complete the
    // full speed change within the block, we need to adjust the entry speed accordingly. Entry
    // speeds have already been reset, maximized, and reverse planned by reverse planner.
    // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.

    // TODO: find out if both of these checks are necessary
    if (prev_max_exit_speed > b_nominal_speed)
        prev_max_exit_speed = b_nominal_speed;
    if (prev_max_exit_speed > b_max_entry_speed)
        prev_max_exit_speed = b_max_entry_speed;

    if (prev_max_exit_speed <= b_entry_speed)
    {
        // accel limited
        b_entry_speed = prev_max_exit_speed;
        // since we're now acceleration or cruise limited
        // we don't need to recalculate our entry speed anymore
        b_recalculate_flag = false;
    }
    // else
    // // decel limited, do nothing

    return max_exit_speed();
}

// Called by Planner::recalculate() when scanning the plan from first to last entry.
// returns maximum exit speed of this block
float Block::c_forward_pass(float prev_max_exit_speed)
{
    // If the previous block is an acceleration block, but it is not long enough to complete the
    // full speed change within the block, we need to adjust the entry speed accordingly. Entry
    // speeds have already been reset, maximized, and reverse planned by reverse planner.
    // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.

    // TODO: find out if both of these checks are necessary
    if (prev_max_exit_speed > c_nominal_speed)
        prev_max_exit_speed = c_nominal_speed;
    if (prev_max_exit_speed > c_max_entry_speed)
        prev_max_exit_speed = c_max_entry_speed;

    if (prev_max_exit_speed <= c_entry_speed)
    {
        // accel limited
        c_entry_speed = prev_max_exit_speed;
        // since we're now acceleration or cruise limited
        // we don't need to recalculate our entry speed anymore
        c_recalculate_flag = false;
    }
    // else
    // // decel limited, do nothing

    return max_exit_speed();
}


float Block::max_exit_speed()
{
    // if block is currently executing, return cached exit speed from calculate_trapezoid
    // this ensures that a block following a currently executing block will have correct entry speed
    if (times_taken)
        return exit_speed;

    // if nominal_length_flag is asserted
    // we are guaranteed to reach nominal speed regardless of entry speed
    // thus, max exit will always be nominal
    if (nominal_length_flag)
        return nominal_speed;

    // otherwise, we have to work out max exit speed based on entry and acceleration
    float max = max_allowable_speed(-THEKERNEL->planner->get_acceleration(), this->entry_speed, this->millimeters);

    return min(max, nominal_speed);
}

float Block::a_max_exit_speed()
{
    // if block is currently executing, return cached exit speed from calculate_trapezoid
    // this ensures that a block following a currently executing block will have correct entry speed
    if (times_taken)
        return a_exit_speed;

    // if nominal_length_flag is asserted
    // we are guaranteed to reach nominal speed regardless of entry speed
    // thus, max exit will always be nominal
    if (a_nominal_length_flag)
        return a_nominal_speed;

    // otherwise, we have to work out max exit speed based on entry and acceleration
    float max = max_allowable_speed(-THEKERNEL->planner->get_a_acceleration(), this->a_entry_speed, this->a_millimeters);

    return min(max, a_nominal_speed);
}

float Block::b_max_exit_speed()
{
    // if block is currently executing, return cached exit speed from calculate_trapezoid
    // this ensures that a block following a currently executing block will have correct entry speed
    if (times_taken)
        return b_exit_speed;

    // if nominal_length_flag is asserted
    // we are guaranteed to reach nominal speed regardless of entry speed
    // thus, max exit will always be nominal
    if (b_nominal_length_flag)
        return b_nominal_speed;

    // otherwise, we have to work out max exit speed based on entry and acceleration
    float max = max_allowable_speed(-THEKERNEL->planner->get_b_acceleration(), this->b_entry_speed, this->b_millimeters);

    return min(max, b_nominal_speed);
}

float Block::c_max_exit_speed()
{
    // if block is currently executing, return cached exit speed from calculate_trapezoid
    // this ensures that a block following a currently executing block will have correct entry speed
    if (times_taken)
        return c_exit_speed;

    // if nominal_length_flag is asserted
    // we are guaranteed to reach nominal speed regardless of entry speed
    // thus, max exit will always be nominal
    if (c_nominal_length_flag)
        return c_nominal_speed;

    // otherwise, we have to work out max exit speed based on entry and acceleration
    float max = max_allowable_speed(-THEKERNEL->planner->get_c_acceleration(), this->c_entry_speed, this->c_millimeters);

    return min(max, c_nominal_speed);
}


// Gcodes are attached to their respective blocks so that on_gcode_execute can be called with it
void Block::append_gcode(Gcode* gcode)
{
    Gcode new_gcode = *gcode;
    new_gcode.strip_parameters(); // optimization to save memory we strip off the XYZIJK parameters from the saved command
    gcodes.push_back(new_gcode);
}

void Block::begin()
{
    recalculate_flag = false;

    a_recalculate_flag = false;
    b_recalculate_flag = false;
    c_recalculate_flag = false;

    if (!is_ready)
        __debugbreak();

    times_taken = -1;

    // execute all the gcodes related to this block
    for(unsigned int index = 0; index < gcodes.size(); index++)
        THEKERNEL->call_event(ON_GCODE_EXECUTE, &(gcodes[index]));

    THEKERNEL->call_event(ON_BLOCK_BEGIN, this);

    if (times_taken < 0)
        release();
}

// Signal the conveyor that this block is ready to be injected into the system
void Block::ready()
{
    this->is_ready = true;
}

// Mark the block as taken by one more module
void Block::take()
{
    if (times_taken < 0)
        times_taken = 0;
    times_taken++;
}

// Mark the block as no longer taken by one module, go to next block if this free's it
void Block::release()
{
    if (--this->times_taken <= 0)
    {
        times_taken = 0;
        if (is_ready)
        {
            is_ready = false;
            THEKERNEL->call_event(ON_BLOCK_END, this);

            // ensure conveyor gets called last
            THEKERNEL->conveyor->on_block_end(this);
        }
    }
}
