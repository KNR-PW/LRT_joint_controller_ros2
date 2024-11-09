#ifndef _POSITION_CONTROLLER_HPP_
#define _POSITION_CONTROLLER_HPP_

namespace position_controller
{
    struct PID_Parameters
    {
        /* Classic PID coefficients */
        double proportional_coef_;
        double derivative_coef_;
        double integral_coef_;

        /* Limit for integration term */
        double integration_limit_;
    };

    struct PID_Scales
    {
        /* Proportional and derivative scales, can be changed in every control call */
        double proportional_scale_ = 1;
        double derivative_scale_ = 1;
    };

    class PositionController
    {
        private:

        PID_Parameters pid_params_;

        PID_Scales pid_scales_;

        /* Position integrator, updated in every control call */
        double position_integrator_;

        /* Time step (inverse of frequency)*/
        double time_step_;

        double calculateProportionalOutput(double _position_error); // Biorą bledy juz z wartosci wyzej
        double calculateIntegralOutput(double _position_error);
        double calculateDerivativeOutput(double _velocity_error);


        public:

        PositionController(PID_Parameters _pid_params, double _frequency);
        PositionController(const PositionController& other) = default;
        PositionController(PositionController&& other) = default;

        double calculateOutput(double _position_error, double _velocity_error,
         double _feedforward_input, PID_Scales _pid_scales); //  Tutaj liczymy wszystko
    };
};
#endif