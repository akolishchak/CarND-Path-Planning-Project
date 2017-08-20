//
// Created by Andrey Kolishchak on 8/5/17.
//

#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <vector>
#include "map.h"
#include "object.h"
#include "trajectory.h"

using namespace std;

/**
 * Planning a path trajectory
 */
class Planner {
public:
    /**
     * Planner constructor
     * @param _map      - object of Map class
     */
    Planner(Map &_map);

    /**
     * Update state of vehicle and environment
     *
     * @param s         - s coordinate of vehicle
     * @param v_s       - vehicle velocity along s axis
     * @param a_s       - vehicle acceleration along s axis
     * @param d         - d coordinate of vehicle
     * @param v_d       - vehicle velocity along d axis
     * @param a_d       - vehicle acceleration along d axis
     * @param objects   - list of surrounding objects
     */
    void update_state(double s, double v_s, double a_s, double d, double v_d,  double a_d, vector<Object> &objects);

    /**
     * Get jerk-minimized polynomial trajectory for s coordinate
     *
     * @return  - Trajectory class object
     */
    Trajectory get_s_trajectory(void);
    /**
     * Get jerk-minimized polynomial trajectory for d coordinate
     *
     * @return  - Trajectory class object
     */
    Trajectory get_d_trajectory(void);

private:
    //
    // Map object
    //
    Map *map;

    //
    // maximum velocity, m/s
    //
    const double max_velocity = 46 * 1609.344 / 3600;
    //
    // maximum acceleration, m/s^2
    //
    const double max_acceleration = 7;
    //
    // length of vehicle, m
    //
    const double vehicle_length = 5;

    //
    // Vehicle state
    //
    struct vehicle_state {
        double s;           // s coordinate
        double d;           // d coordinate
        double vs;          // velocity along s axis
        double vd;          // velocity along d axis
        double as;          // acceleration along s axis
        double ad;          // acceleration along d axis
        int lane;           // vehicle's lane
    };

    //
    // High level actions modeled by the planner
    //
    enum {
        act_keep_lane,      // keep the current lane
        act_left_lane,      // change lane to the left
        act_right_lane,     // change lane to the right
        act_max
    };
    //
    // current state set in update_state() method
    //
    vehicle_state current_state;

    //
    // last action taken by planner
    //
    int last_action;

    /**
     * Set target s, d, velocities and accelerations according to the given action
     *
     * @param action        - high level action
     * @param vehicle       - state of vehicle
     * @param time_step     - index of time step in prediction horizon
     *
     * @return              - false if action is not possible or leads to a collision, true - otherwise
     */
    bool set_action(int action, vehicle_state &vehicle, size_t time_step);

    /**
     * Find closest object in front of vehicle
     *
     * @param vehicle       - state of vehicle
     * @param time_step     - index of time step in prediction horizon
     * @param object        - parameters of found front object, position and velocity
     *
     * @return              - true if front object is found, false - otherwise
     */
    bool get_closest_object_state(vehicle_state &vehicle, size_t time_step, Object::state &object);

    /**
     * Set trajectory targets, such as s, d, velocities and accelerations.
     * The targets are used by get_s_trajectory() and get_d_trajectory() methods
     *
     * @param vehicle       - state of vehicle
     * @param time_step     - index of time step in prediction horizon
     *
     * @return              - false if a collision is expected, true - otherwise
     */
    bool set_trajectory_targets(vehicle_state &vehicle, size_t time_step);

    double target_s;        // target s coordinate
    double target_vs;       // target velocity along s axis
    double target_as;       // target acceleration along s axis
    double target_d;        // target d coordinate
    int target_lane;        // target lane

    //
    // list of predicted locations and parameters of surrounding objects
    // first vector enumerates objects, second - enumerates time steps in prediction horizon
    //
    vector<vector<Object::state>> predictions;
};


#endif //PATH_PLANNING_PLANNER_H
