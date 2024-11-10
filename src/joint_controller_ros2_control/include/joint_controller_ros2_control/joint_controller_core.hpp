#ifndef _JOINT_CONTROLLER_CORE_HPP_
#define _JOINT_CONTROLLER_CORE_HPP_

#include "pid_controller/pid_controller.hpp"

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
        double kp_scale_ = 0;           
        double kd_scale_ = 0;
        double feedforward_effort_ = 0; /* [Nm] or [N]*/
    };


    class JointController
    {
        private:

        JointParameters joint_params_;
        pid_controller::PidController pid_controller_;

        public:

        JointController(JointParameters _joint_params,
         pid_controller::PidParameters _pid_params, double _frequency);
        JointController(const JointController& other) = default;
        JointController(JointController&& other) = default;

        double calculateEffort(const JointCommands& _joint_command,const JointStates& _joint_state);
        
    };
};

#endif

