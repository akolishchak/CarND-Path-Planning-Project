//
// Created by Andrey Kolishchak on 8/5/17.
//

#include "object.h"

/**
 * Object class constructor
 *
 * @param postion       - object's parameters
 */
Object::Object(const params &postion) : current_position(postion)
{

}

/**
 * Predict future states of an object
 *
 * @param horizon       - prediction horizon, number of time steps
 * @param delta_t       - the duration of a time step
 *
 * @return              - list of future states, size equal to prediction horizon
 */
vector<Object::state> Object::predict(size_t horizon, double delta_t)
{
    vector<state> result;

    double s = current_position.s;
    double d = current_position.d;
    double vs = current_position.vs;
    double vd = current_position.vd;
    //
    // first element - is the current state
    //
    result.push_back({s, vs, d});

    //
    // Compute object's states up to prediction horizon
    //
    for ( size_t i = 0; i < horizon; i++ ) {
        //
        // assume constant velocity
        //
        s += vs * delta_t;
        d += vd * delta_t;
        //
        // add updated state to the list
        //
        result.push_back({s, vs, d});
    }

    return result;
}
