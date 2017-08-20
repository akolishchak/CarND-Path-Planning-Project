//
// Created by Andrey Kolishchak on 8/5/17.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>
using namespace std;

/**
 * Trajectory class, jerk-minimized trajectory
 *
 */
class Trajectory {
public:
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
    Trajectory(double x_i, double v_i, double a_i, double x_f, double v_f, double a_f, double t);

    /**
     * Get position at time
     *
     * @param time      - time, (0 .. trajectory duration)
     *
     * @return          - position
     */
    double operator ()(double time);

    /**
     * Get maximum velocity on duration
     *
     * @param duration   - trajectory duration
     *
     * @return           - maximum velocity
     */
    double get_max_velocity(double duration);

private:
    //
    // trajectory polynomial
    //
    vector<double> poly;
    //
    // Maximum time, trajectory duration
    //
    double max_time;
    //
    // Final position of the trajectory
    //
    double final_x;
};


#endif //PATH_PLANNING_TRAJECTORY_H
