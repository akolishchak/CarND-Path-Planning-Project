//
// Created by Andrey Kolishchak on 8/5/17.
//

#ifndef PATH_PLANNING_OBJECT_H
#define PATH_PLANNING_OBJECT_H

#include <vector>
using namespace std;

/**
 * Object class, road object parameters and predictions
 */
class Object {
public:
    //
    // Type of object
    //
    enum {
        type_unknown,       // unknown object
        type_vehicle        // vehicle
    };

    //
    // Parameters of object
    //
    struct params {
        int id;             // object id
        int type;           // object type
        double s;           // object s coordinate
        double d;           // object d coordinate
        double vs;          // object s axis velocity
        double vd;          // object d axis velocity
    };

    //
    // Object state in prediction horizon
    //
    struct state {
        double s;           // object s coordinate
        double vs;          // object s axis velocity
        double d;           // object d coordinate
    };

    /**
     * Object class constructor
     *
     * @param postion       - object's parameters
     */
    Object(const params &postion);

    /**
     * Predict future states of an object
     *
     * @param horizon       - prediction horizon, number of time steps
     * @param delta_t       - the duration of a time step
     *
     * @return              - list of future states, size equal to prediction horizon
     */
    vector<state> predict(size_t horizon, double delta_t);

private:
    //
    // Current parameters of object
    //
    params current_position;
};


#endif //PATH_PLANNING_OBJECT_H
