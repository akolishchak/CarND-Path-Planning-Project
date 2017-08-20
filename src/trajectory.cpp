//
// Created by Andrey Kolishchak on 8/5/17.
//

#include "trajectory.h"
#include "Eigen-3.3/Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Trajectory class constructor
 *
 * @param x_i       - initial position
 * @param v_i       - initial velocity
 * @param a_i       - initial acceleration
 * @param x_f       - final position
 * @param v_f       - final velocity
 * @param a_f       - final acceleration
 * @param t         - trajectory duration
 */
Trajectory::Trajectory(double x_i, double v_i, double a_i, double x_f, double v_f, double a_f, double t)
{
    //
    // Compute jerk-minimized polynomial: a0 + a1*x + a2*x^2 + a3*x^3 + a4*x^4 + a5*x^5
    //
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;

    MatrixXd A(3, 3);
    A <<    t3, t4, t5,
            3 * t2, 4 * t3, 5 * t4,
            6 * t, 12 * t2, 20 * t3;

    VectorXd B(3);
    B <<    x_f - (x_i + v_i * t + 0.5 * a_i * t2),
            v_f - (v_i + a_i * t),
            a_f - a_i;

    auto Q = A.householderQr();
    auto x = Q.solve(B);

    poly = { x_i, v_i, 0.5 * a_i, x[0], x[1], x[2] };

    //
    // save trajectory duration and final position
    //
    final_x = x_f;
    max_time = t;
    //cout << "TRAJ: " << poly[0] << " + " << poly[1] << "*x + " << poly[2] << "*x^2 + " << poly[3] << "*x^3 + " << poly[4] << "*x^4 + " << poly[5] << "*x^5" << endl;
}

/**
 * Get position at time
 *
 * @param time      - time, (0 .. trajectory duration)
 *
 * @return          - position
 */
double Trajectory::operator ()(double time)
{
    //
    // if time exceed trajectory duration then return final position
    //
    if ( time > max_time )
        return final_x;

    //
    // Compute polynomial
    //
    double result = poly[0];
    double t = 1;

    for ( size_t i = 1; i < poly.size(); i++ ) {
        t *= time;
        result += poly[i]*t;
    }

    return result;
}

/**
 * Get maximum velocity on duration
 *
 * @param duration   - trajectory duration
 *
 * @return           - maximum velocity
 */
double Trajectory::get_max_velocity(double duration)
{
    double result = 0;

    //
    // Enumerate through the trajectory to find maximum velocity
    //
    const int intervals = 50;
    double dt = duration / intervals;
    double t_i = 0;

    for (int i = 0; i < intervals; i++ ) {
        //
        // velocity is first derivative of polynomial: a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4
        //
        t_i += dt;
        double t_i_2 = t_i * t_i;
        double t_i_3 = t_i_2 * t_i;
        double t_i_4 = t_i_3 * t_i;

        double value = fabs(poly[1] + poly[2]*2*t_i + poly[3]*3*t_i_2 + poly[4]*4*t_i_3 + poly[5]*5*t_i_4);
        if ( value > result ) {
            result = value;
        }
    }

    return result;
}