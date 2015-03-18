/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef BLOCK_H
#define BLOCK_H

#include <vector>
#include <bitset>

class Gcode;

class Block {
    public:
        Block();
        void calculate_trapezoid( float entry_speed, float exit_speed );

        void a_calculate_trapezoid( float entry_speed, float exit_speed );
        void b_calculate_trapezoid( float entry_speed, float exit_speed );
        void c_calculate_trapezoid( float entry_speed, float exit_speed );

        float estimate_acceleration_distance( float initial_rate, float target_rate, float acceleration );
        float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance);
        float get_duration_left(unsigned int already_taken_steps);
        float max_allowable_speed( float acceleration, float target_velocity, float distance);

        float reverse_pass(float exit_speed);

        float a_reverse_pass(float exit_speed);
        float b_reverse_pass(float exit_speed);
        float c_reverse_pass(float exit_speed);

        float forward_pass(float next_entry_speed);

        float a_forward_pass(float next_entry_speed);
        float b_forward_pass(float next_entry_speed);
        float c_forward_pass(float next_entry_speed);

        float max_exit_speed();

        float a_max_exit_speed();
        float b_max_exit_speed();
        float c_max_exit_speed();

        void debug();

        void append_gcode(Gcode* gcode);

        void take();
        void release();

        void ready();

        void clear();

        void begin();

        std::vector<Gcode> gcodes;

        unsigned int   steps[6];           // Number of steps for each axis for this block
        unsigned int   steps_event_count;  // Steps for the longest axis
        unsigned int   nominal_rate;       // Nominal rate in steps per second

        unsigned int   a_nominal_rate, b_nominal_rate, c_nominal_rate;

        float          nominal_speed;      // Nominal speed in mm per second

        float          a_nominal_speed, b_nominal_speed, c_nominal_speed;

        float          millimeters;        // Distance for this move

        float          a_millimeters, b_millimeters, c_millimeters;

        float          entry_speed;

        float          a_entry_speed, b_entry_speed, c_entry_speed;

        float          exit_speed;

        float          a_exit_speed, b_exit_speed, c_exit_speed;

        float          rate_delta;         // Number of steps to add to the speed for each acceleration tick

        float          a_rate_delta, b_rate_delta, c_rate_delta;

        unsigned int   initial_rate;       // Initial speed in steps per second

        unsigned int   a_initial_rate, b_initial_rate, c_initial_rate;

        unsigned int   final_rate;         // Final speed in steps per second

        unsigned int   a_final_rate, b_final_rate, c_final_rate;

        unsigned int   accelerate_until;   // Stop accelerating after this number of steps

        unsigned int   a_accelerate_until, b_accelerate_until, c_accelerate_until;

        unsigned int   decelerate_after;   // Start decelerating after this number of steps

        unsigned int   a_decelerate_after, b_decelerate_after, c_decelerate_after;

        std::bitset<6> direction_bits;     // Direction for each axis in bit form, relative to the direction port's mask

        struct {
            bool recalculate_flag:1;             // Planner flag to recalculate trapezoids on entry junction

            bool a_recalculate_flag:1;
            bool b_recalculate_flag:1;
            bool c_recalculate_flag:1;

            bool nominal_length_flag:1;          // Planner flag for nominal speed always reached

            bool a_nominal_length_flag:1;
            bool b_nominal_length_flag:1;
            bool c_nominal_length_flag:1;

            bool is_ready:1;
        };

        float max_entry_speed;

        float a_max_entry_speed;
        float b_max_entry_speed;
        float c_max_entry_speed;

        short times_taken;    // A block can be "taken" by any number of modules, and the next block is not moved to until all the modules have "released" it. This value serves as a tracker.

};


#endif
