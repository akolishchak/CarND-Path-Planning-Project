#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "json.hpp"
#include "map.h"
#include "object.h"
#include "planner.h"


using namespace std;

// for convenience
using json = nlohmann::json;

//
// Vehicle moves to a new defined position every 20ms, tic duration
//
const double tic_time = 0.02;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

int main() {
    uWS::Hub h;

    //
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    //
    Map map("../data/highway_map.csv");

    //
    // Initialize planner
    //
    Planner planner(map);

    //
    // Previous trajectory s and d coordinates
    //
    vector<double> prev_s;
    vector<double> prev_d;

    //
    // Previous velocities and accelerations
    //
    double prev_v_s = 0, prev_a_s = 0, prev_v_d = 0, prev_a_d = 0;

    h.onMessage([&map, &prev_s, &prev_d, &prev_v_s, &prev_a_s, &prev_v_d, &prev_a_d, &planner]
                        (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    json msgJson;

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    //
                    // Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
                    //

                    //
                    // Frenet trajectory
                    //
                    vector<double> current_s;
                    vector<double> current_d;

                    //
                    // Number of previously set points
                    //
                    const size_t trajectory_points = prev_s.size();

                    //
                    // Number of unprocessed points
                    //
                    size_t remaining_points = previous_path_x.size();

                    //
                    // Number of points (tics) we are going to use from previously unprocessed tra
                    //
                    size_t points_from_prev_trajectory = remaining_points;

                    //
                    // Number of processed points
                    //
                    size_t processed_points = remaining_points > 0 ? trajectory_points - remaining_points : 0;

                    //
                    // Remove processed points from previous trajectory s and d list
                    //
                    for (size_t i = 0; i < processed_points; i++) {
                        prev_s.erase(prev_s.begin());
                        prev_d.erase(prev_d.begin());
                    }

                    //
                    // Add points from the previous trajectory to the new trajectory
                    //
                    for (size_t i = 0; i < points_from_prev_trajectory; i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                        current_s.push_back(prev_s[i]);
                        current_d.push_back(prev_d[i]);
                    }

                    //
                    // if previous trajectory size is greater or equal to 15 then use previous trajectory
                    //
                    if ( points_from_prev_trajectory < 15 ) {
                        //
                        // if previous trajectory size is less than 15 then plan new trajectory
                        //

                        //
                        // Retrieve objects from sensor fusion data
                        //
                        vector<Object> objects;
                        for (size_t i = 0; i < sensor_fusion.size(); i++) {
                            auto &entry = sensor_fusion[i];

                            Object::params params;
                            params.d = entry[6];
                            // ignore all objects outside of the road
                            if ( map.is_road(params.d) ) {

                                params.type = Object::type_vehicle;
                                params.id = entry[0];
                                params.s = entry[5];
                                double vx = entry[3];
                                double vy = entry[4];
                                map.get_frenet_velocity(params.s, vx, vy, params.vs, params.vd);
                                params.s += params.vs * points_from_prev_trajectory * tic_time;
                                objects.push_back(Object(params));
                            }
                        }

                        //
                        // Compute latest positions, velocity and accelerations
                        //
                        double s = car_s, v_s = prev_v_s, a_s = prev_a_s;
                        double d = car_d, v_d = prev_v_d, a_d = prev_a_d;
                        //
                        // If previous trajectory size exceeds 3 then we can compute postion, velocity and acceleration
                        // otherwise use previous values
                        //
                        if ( points_from_prev_trajectory >= 3 ) {
                            //
                            // s position, velocity, and acceleration
                            //

                            // position
                            s = prev_s[points_from_prev_trajectory - 1];
                            double s_1 = prev_s[points_from_prev_trajectory - 2];
                            double s_2 = prev_s[points_from_prev_trajectory - 3];

                            // velocity
                            v_s = map.diff_s(s, s_1) / tic_time;

                            // acceleration
                            double vs_perv = map.diff_s(s_1, s_2) / tic_time;
                            a_s = (v_s - vs_perv) / tic_time;

                            //
                            // d position, velocity, and acceleration
                            //

                            // position
                            d = prev_d[points_from_prev_trajectory - 1];
                            double d_1 = prev_d[points_from_prev_trajectory - 2];
                            double d_2 = prev_d[points_from_prev_trajectory - 3];

                            // velocity
                            v_d = ( d - d_1 ) / tic_time;

                            // acceleration
                            double vd_perv = ( d_1 - d_2 ) / tic_time;
                            a_d = ( v_d - vd_perv ) / tic_time;

                            //
                            // save computed positions, velocity and accelerations
                            //
                            prev_v_s = v_s;
                            prev_a_s = a_s;
                            prev_v_d = v_d;
                            prev_a_d = a_d;
                        }

                        //
                        // Plan a new trajectory in Frenet
                        //
                        planner.update_state(s, v_s, a_s, d, v_d, a_d, objects);
                        Trajectory s_trajectory = planner.get_s_trajectory();
                        Trajectory d_trajectory = planner.get_d_trajectory();

                        //
                        // Add new trajectory points
                        //
                        for (size_t i = 0; i < 50 - points_from_prev_trajectory; i++) {
                            //
                            // get Frenet coordinates
                            //
                            double s_i = s_trajectory((i + 1) * tic_time);
                            double d_i = d_trajectory((i + 1) * tic_time);
                            s_i = map.normalize_s(s_i);

                            //
                            // Convert to Cartesian
                            //
                            double x, y;
                            map.get_cartesian(s_i, d_i, x, y);

                            //
                            // Add points to the lists
                            //
                            next_x_vals.push_back(x);
                            next_y_vals.push_back(y);

                            current_s.push_back(s_i);
                            current_d.push_back(d_i);
                        }

                        //
                        // Save Frenet trajectory
                        //
                        prev_s = current_s;
                        prev_d = current_d;
                    }

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
