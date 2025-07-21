// Copyright (c) 2025, Koło Naukowe Robotyków
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/*
 * Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */

#ifndef _JOINT_CONTROLLER_CORE_HPP_
#define _JOINT_CONTROLLER_CORE_HPP_

#include "joint_controller_ros2_control/pid_controller/pid_controller.hpp"
#include <algorithm>

namespace joint_controller_core
{
    struct JointParameters
    {
        double position_max_ = 0;       /* [radians] or [m]*/
        double position_min_ = 0;       /* [radians] or [m]*/
        double position_offset_ = 0;    /* [radians]  or [m]*/
        double velocity_max_ = 0;       /* [radians/s] or [m/s]*/
        double effort_max_ = 0;         /* [Nm] or [N]*/
    };

    struct JointStates
    {
        double position_ = 0;            /* [radians] or [m]*/
        double velocity_ = 0;            /* [radians/s] or [m/s]*/
        double effort_ = 0;              /* [Nm] or [N]*/
    };

    struct JointCommands
    {
        double desired_position_ = 0;   /* [radians] or [m]*/
        double desired_velocity_ = 0;   /* [radians/s] or [m/s]*/
        double kp_scale_ = 1;           
        double kd_scale_ = 1;
        double feedforward_effort_ = 0; /* [Nm] or [N]*/
    };


    class JointControllerCore
    {
        private:

        /* Joint parameters */
        JointParameters joint_params_;

        /* Internal PID controller parameters */
        pid_controller::PidController pid_controller_;

        public:

        JointControllerCore(JointParameters _joint_params,
         pid_controller::PidParameters _pid_params, double _frequency);
        JointControllerCore(const JointControllerCore& other) = default;
        JointControllerCore(JointControllerCore&& other) = default;

        /* Effort calculation */
        double calculateEffort(const JointCommands& _joint_command,const JointStates& _joint_state);
        
    };
};

#endif

