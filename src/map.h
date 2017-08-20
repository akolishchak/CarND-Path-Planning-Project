//
// Created by Andrey Kolishchak on 8/5/17.
//

#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H

#include <string>
#include <vector>
#include <math.h>
#include "spline.h"

using namespace std;

/**
 * Map class.
 * The class loads map from CSV file and provides Frenet <-> Cartesian convertion, lane coordinates, and etc.
 */
class Map {
public:
    /**
     * Map class constructor
     *
     * @param map_file      - name of the CSV file with map
     */
    Map(string map_file);

    /**
     * Convert Frenet coordinates to Cartesian
     *
     * @param s     - s Frenet coordinate
     * @param d     - d Frenet coordinate
     * @param x     - output Cartesian x coordinate
     * @param y     - output Cartesian y coordinate
     */
    void get_cartesian(double s, double d, double &x, double &y);

    /**
     * Get Frenet velocity of an object
     *
     * @param s      - object s coordinate
     * @param vx     - object x axis velocity
     * @param vy     - object y axis velocity
     * @param vs     - output velocity along s axis
     * @param vd     - output velocity along d axis
     */
    void get_frenet_velocity(double s, double vx, double vy, double &vs, double &vd);

    /**
     * Check if the d coordinate belongs to the road
     *
     * @param d     - d coordinate
     *
     * @return      - true in case of road, false - otherwise
     */
    bool is_road(double d)
    {
        return 0 <= d && d <= get_lanes_num() * get_lane_width();
    }

    /**
     * Get lane width in meters
     *
     * @return      - lane width in meters
     */
    double get_lane_width(void)
    {
        return 4;
    }

    /**
     * Get number of lanes per road side
     *
     * @return      - number of lanes
     */
    double get_lanes_num(void)
    {
        return 3;
    }

    /**
     * Get d coordinate of lane center
     *
     * @param lane   - number of lane (1..3)
     *
     * @return       - d coordinate of the lane
     */
    double get_lane_center(int lane)
    {
        return lane == 3 ? 9.8 : get_lane_width() * ( lane - 0.5 );
    }

    /**
     * Get lane number
     *
     * @param d      - d coordinate
     *
     * @return       - number of lane (1..3)
     */
    int get_lane(double d)
    {
        return 1 + int(d/4.0);
    }

    /**
     * Get maximum map s coordinate
     *
     * @return
     */
    double get_max_s(void)
    {
        return max_s;
    }

    /**
     * Normalize s coordinate according to the map edge
     *
     * @param s     - map s coordinate
     *
     * @return      - normalized s coordinate
     */
    double normalize_s(double s)
    {
        return s <= max_s ? s : s - max_s;
    }

    //
    // edge detection distance
    //
    const double max_edge_distance = 100;

    /**
     * Check for edge coordinates
     *
     * @param s1     - s coordinate
     * @param s2     - s coordinate
     *
     * @return       - true if coordinates located at edge, false - otherwise
     */
    bool is_edge(double s1, double s2)
    {
        return fabs(s1 - s2) > (max_s - max_edge_distance);
    }

    /**
     * Compute difference between s coordinate. The method takes in accont edges
     *
     * @param l     - s coordinate
     * @param r     - s coordinate
     *
     * @return      -  difference between given coordinates
     */
    double diff_s(double l, double r)
    {
        if ( is_edge(l, r) ) {
            if ( l < max_edge_distance )
                l += max_s;
            if ( r < max_edge_distance )
                r += max_s;
        }

        return l - r;
    }

private:

    //
    // Middle road splines for Frenet to Cartesian convertion
    //
    tk::spline x_spline;
    tk::spline y_spline;
    tk::spline dx_spline;
    tk::spline dy_spline;

    //
    // Maximum s coordinate
    //
    double max_s;
};

#endif //PATH_PLANNING_MAP_H
