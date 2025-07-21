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
            Stanisław Sufin      (https://github.com/Kafel1997)
 */


#ifndef _PID_CONTROLLER_HPP_
#define _PID_CONTROLLER_HPP_

namespace pid_controller
{
    struct PidParameters
    {
        /* Classic PID coefficients */
        double proportional_coef_ = 0;
        double derivative_coef_ = 0;
        double integral_coef_ = 0;

        /* Limit for integration term */
        double integration_limit_ = 0;
    };

    class PidController
    {
        private:

        PidParameters pid_params_;

        /* Position integrator, updated in every control call */
        double position_integrator_;

        /* Time step (inverse of frequency)*/
        double time_step_;

        double calculateProportionalEffort(double _position_error, double _proportional_scale); 
        double calculateIntegralEffort(double _position_error);
        double calculateDerivativeEffort(double _velocity_error, double _derivative_scale);


        public:

        PidController(PidParameters _pid_params, double _frequency);
        PidController(const PidController& other) = default;
        PidController(PidController&& other) = default;

        double calculateEffort(double _position_error, double _velocity_error,
            double _feedforward_effort, double _proportional_scale, double _derivative_scale); //  Tutaj liczymy wszystko
    };
};
#endif