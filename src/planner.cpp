//
// Created by Andrey Kolishchak on 8/5/17.
//

#include "planner.h"
#include <iostream>
#include <math.h>

/**
 * Planner constructor
 * @param _map      - object of Map class
 */
Planner::Planner(Map &_map) : map(&_map), last_action(act_keep_lane), target_lane(1)
{
}

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
void Planner::update_state(double s, double v_s, double a_s, double d, double v_d,  double a_d, vector<Object> &objects)
{
    //
    // update current state
    //
    current_state.s = s;
    current_state.vs = v_s;
    current_state.as = a_s;
    current_state.d = d;
    current_state.vd = v_d;
    current_state.ad = a_d;
    current_state.lane = map->get_lane(current_state.d);

    //
    // update predictions
    //
    predictions.clear();
    size_t prediction_horizon = 3;
    for ( size_t i = 0; i < objects.size(); i++ ) {
        predictions.push_back(objects[i].predict(prediction_horizon, 1));
    }

    //
    // planning - selecting one of the possible actions: keep lane, take left lane, take right lane
    //
    int best_action;
    if ( last_action != act_keep_lane && current_state.lane != target_lane ) {
        //
        // keep action while in transition to a new lane
        //
        best_action = last_action;
    } else {
        //
        // select an action with maximum score
        //
        double max_score = 0;
        //
        // By default - keep lane
        //
        best_action = act_keep_lane;
        //
        // Enumerate all actions
        //
        for ( int action = 0; action < act_max; action++ ) {
            //
            // initialize score and vehicle state
            //
            double score = 0;
            vehicle_state vehicle = current_state;
            //
            // set action target state, such as s, d, velocities and accelerations
            //
            if ( set_action(action, vehicle, 0) ) {
                //
                // No collision detected at the current step, proceed with next time steps up to prediction horizon
                //
                vehicle.s = target_s;
                vehicle.vs = target_vs;
                for ( size_t i = 1; i <= prediction_horizon; i++ ) {
                    //
                    // find a closest object in front
                    //
                    Object::state front_object;
                    if ( get_closest_object_state(vehicle, i, front_object) ) {
                        //
                        // object found, check for collision via distance between our vehicle and object
                        //
                        double distance = map->diff_s(front_object.s, vehicle.s);
                        if (distance >= -vehicle_length && distance < vehicle_length) {
                            //
                            // treat vehicle size distance as a collision
                            //
                            //
                            // in case of collision reset score to zero
                            //
                            score = 0;
                            break;
                        }
                    }
                    //
                    // Compute next stat of vehicle
                    //
                    vehicle.s += vehicle.vs;
                    //
                    // Increase score by distance that vehicle traveled
                    //
                    score += vehicle.vs;
                }

                //
                // penalize lane change
                //
                if ( action != act_keep_lane )
                    score -= vehicle_length;

                //
                // check if score is greater the maximum
                //
                if ( score > max_score ) {
                    max_score = score;
                    best_action = action;
                }
            } else if ( action == act_keep_lane ) {
                //
                // Potential collision is detected at the current lane
                // Do not change lane at such conditions, stop enumeration of the actions
                //
                break;
            }
        }
    }
    //
    // Set targets for the best action (maximum score)
    //
    set_action(best_action, current_state, 0);
    //
    // Update last action and target lane
    //
    last_action = best_action;
    target_lane = current_state.lane;
}

/**
 * Set target state, such as s, d, velocities and accelerations according to the given action
 *
 * @param action        - high level action
 * @param vehicle       - state of vehicle
 * @param time_step     - index of time step in prediction horizon
 *
 * @return              - false if action is not possible or leads to a collision, true - otherwise
 */
bool Planner::set_action(int action, vehicle_state &vehicle, size_t time_step)
{
    bool result = true;

    //
    // update target d coordinate according to desired lane
    //
    switch ( action ) {
        case act_keep_lane:
            target_d = map->get_lane_center(vehicle.lane);
            break;

        case act_left_lane:
            if ( vehicle.lane == 1 ) {
                result = false;
            } else {
                vehicle.lane--;
                target_d = map->get_lane_center(vehicle.lane);
            }
            break;

        case act_right_lane:
            if ( vehicle.lane == map->get_lanes_num() ) {
                result = false;
            } else {
                vehicle.lane++;
                target_d = map->get_lane_center(vehicle.lane);
            }
            break;

        default:
            result = false;
    }

    //
    // if lane change is possible then set trajectory targets
    //
    if ( result ) {
        result = set_trajectory_targets(vehicle, time_step);
    }

    return result;
}

/**
 * Find closest object in front of vehicle
 *
 * @param vehicle       - state of vehicle
 * @param time_step     - index of time step in prediction horizon
 * @param object        - parameters of found front object, position and velocity
 *
 * @return              - true if front object is found, false - otherwise
 */
bool Planner::get_closest_object_state(vehicle_state &vehicle, size_t time_step, Object::state &object)
{
    bool result = false;
    //
    // find an object with minimal distance
    //
    double min_distance = 1e10;
    int object_idx = -1;

    for ( size_t i = 0; i < predictions.size(); i++ ) {
        //
        // skip objects which are behind at the current state
        //
        if ( map->get_lane(predictions[i][0].d) == current_state.lane && predictions[i][0].s < current_state.s )
            continue;

        //
        // select objects within the borders of the target lane
        //
        const Object::state &object_state = predictions[i][time_step];

        if ( map->get_lane(object_state.d+0.5) == vehicle.lane || map->get_lane(object_state.d - 0.5) == vehicle.lane ) {
            //
            // select object with minimal distance
            //
            double distance = map->diff_s(object_state.s, vehicle.s);
            if ( distance >= -vehicle_length && distance < min_distance ) {
                min_distance = distance;
                object_idx = i;
            }
        }
    }

    if ( object_idx >= 0 ) {
        //
        // front object found, return true
        //
        object = predictions[object_idx][time_step];
        result = true;
    }

    return result;
}

/**
 * Set trajectory targets, such as s, d, velocities, and accelerations.
 * The targets are used by get_s_trajectory() and get_d_trajectory() methods
 *
 * @param vehicle       - state of vehicle
 * @param time_step     - index of time step in prediction horizon
 *
 * @return              - false if a collision is expected, true - otherwise
 */
bool Planner::set_trajectory_targets(vehicle_state &vehicle, size_t time_step)
{
    bool result = true;

    Object::state front_object;

    //
    // Adjust maximum velocity to account d axis velocity
    //
    double vd = get_d_trajectory().get_max_velocity(1) * 2;
    double max_velocity_s = sqrt(max_velocity * max_velocity - vd * vd);

    //
    // Retrieve closest object in front
    //
    if ( get_closest_object_state(vehicle, time_step, front_object) == false ) {
        //
        // no obstacles, go with maximum velocity
        //
        target_vs = min(vehicle.vs + max_acceleration, max_velocity_s);
        target_as = target_vs - vehicle.vs;
        target_s = vehicle.s + vehicle.vs + target_as / 2;
    } else {
        //
        // A front object is found, measure distance in time
        // This is time to collision if front object suddenly stops while ours moves with constant velocity
        //
        double sudden_stop_time_distance;
        double distance = front_object.s - vehicle.s;
        sudden_stop_time_distance = vehicle.vs > 1e-3 ? distance / vehicle.vs : 10;
        //
        // Update targets according to the time distance
        //
        if ( sudden_stop_time_distance < 0.75 ) {
            //
            // Keep distance by dropping speed
            //
            target_as = min(max_velocity_s, front_object.vs) - 2 - vehicle.vs;
            target_vs = vehicle.vs + target_as;
            target_s = vehicle.s + vehicle.vs + target_as / 2;
        }
        else if ( sudden_stop_time_distance < 1 ) {
            //
            // Set speed smaller than lead
            //
            target_as = ( min(max_velocity_s, front_object.vs) - 1 - vehicle.vs );
            target_vs = vehicle.vs + target_as;
            target_s = vehicle.s + vehicle.vs + target_as / 2;
        }
        else if ( sudden_stop_time_distance < 1.5 ) {
            //
            // Adjust speed to match lead
            //
            target_as = ( min(max_velocity_s, front_object.vs) - vehicle.vs );
            target_vs = vehicle.vs + target_as;
            target_s = vehicle.s + vehicle.vs + target_as / 2;
        }
        else {
            //
            // Obstacle is far enough - go as fast as can
            //
            target_vs = min(vehicle.vs + max_acceleration, max_velocity_s);
            target_as = target_vs - vehicle.vs;
            target_s = vehicle.s + vehicle.vs + target_as / 2;
        }

        if ( sudden_stop_time_distance < 1 ) {
            //
            // report collision risk to planner
            //
            result = false;
        }

    }

    return result;
}

/**
 * Get jerk-minimized polynomial trajectory for s coordinate
 *
 * @return  - Trajectory class object
 */
Trajectory Planner::get_s_trajectory(void)
{
    //
    // Create jerk-minimized s trajectory for the duration of one second
    //
    double time = 1;
    Trajectory trajectory(current_state.s, current_state.vs, current_state.as, target_s, target_vs, target_as, time);

    return trajectory;
}

/**
 * Get jerk-minimized polynomial trajectory for d coordinate
 *
 * @return  - Trajectory class object
 */
Trajectory Planner::get_d_trajectory(void)
{
    //
    // Create jerk-minimized d trajectory for the duration of one second
    //
    // set target velocity and acceleration to zero
    //
    double target_vd = 0;
    double target_ad = 0;

    //
    // get d by target lane (target lane is in current state as it gets updated in update_state())
    //
    target_d = map->get_lane_center(current_state.lane);
    //
    // set trajectory duration assuming maximum velocity of 1 m/s
    //
    double diff = target_d - current_state.d;
    double time = max(1.0, fabs(diff) / 1);

    Trajectory trajectory(current_state.d, current_state.vd, current_state.ad, target_d, target_vd, target_ad, time);

    return trajectory;
}