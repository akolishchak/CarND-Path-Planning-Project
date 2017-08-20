//
// Created by Andrey Kolishchak on 8/5/17.
//
#include <fstream>
#include <sstream>
#include <math.h>
#include "map.h"

/**
 * Map class constructor
 *
 * @param map_file      - name of the CSV file with map
 */
Map::Map(string map_file)
{
    //
    // Load map from CSV file, assumes that waypoints are sorted by s
    //
    // Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position,
    // the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define
    // the unit normal vector pointing outward of the highway loop.
    //
    ifstream fs(map_file.c_str(), ifstream::in);

    vector<double> waypoint_x;
    vector<double> waypoint_y;
    vector<double> waypoint_s;
    vector<double> waypoint_dx;
    vector<double> waypoint_dy;

    string line;
    while (getline(fs, line)) {
        std::istringstream iss(line);
        double x;
        double y;
        double s;
        double d_x;
        double d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;

        waypoint_x.push_back(x);
        waypoint_y.push_back(y);
        waypoint_s.push_back(s);
        waypoint_dx.push_back(d_x);
        waypoint_dy.push_back(d_y);
    }

    //
    // Set splines
    //
    x_spline.set_points(waypoint_s, waypoint_x);
    y_spline.set_points(waypoint_s, waypoint_y);
    dx_spline.set_points(waypoint_s, waypoint_dx);
    dy_spline.set_points(waypoint_s, waypoint_dy);

    //
    // Get latest s as maximum waypoint's s
    //
    max_s = waypoint_s[waypoint_s.size() - 1];
}

/**
 * Convert Frenet coordinates to Cartesian
 *
 * @param s     - s Frenet coordinate
 * @param d     - d Frenet coordinate
 * @param x     - output Cartesian x coordinate
 * @param y     - output Cartesian y coordinate
 */
void Map::get_cartesian(double s, double d, double &x, double &y)
{
    x = x_spline(s) + d * dx_spline(s);
    y = y_spline(s) + d * dy_spline(s);
}

/**
 * Get Frenet velocity of an object
 *
 * @param s      - object s coordinate
 * @param vx     - object x axis velocity
 * @param vy     - object y axis velocity
 * @param vs     - output velocity along s axis
 * @param vd     - output velocity along d axis
 */
void Map::get_frenet_velocity(double s, double vx, double vy, double &vs, double &vd)
{
    double dx = dx_spline(s);
    double dy = dy_spline(s);
    double speed = sqrt(vx*vx + vy*vy);
    if ( speed < 1e-3 ) {
        vd = 0;
        vs = 0;
    } else {
        double cos_normal_to_speed = (dx * vx + dy * vy) / speed;
        double sin_normal_to_speed = sqrt(1 - cos_normal_to_speed * cos_normal_to_speed);

        vd = speed * cos_normal_to_speed;
        vs = speed * sin_normal_to_speed;
    }
}
