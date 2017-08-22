#include <fstream>
#include <uWS/uWS.h>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "json.hpp"

#include "car.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos)
	{
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos)
	{
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

Car car;

int main()
{
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line))
	{
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	car.path_module_.map_wp_x_ = map_waypoints_x;
	car.path_module_.map_wp_y_ = map_waypoints_y;
	car.path_module_.map_wp_s_ = map_waypoints_s;

	h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length,
			uWS::OpCode opCode)
		{
			// "42" at the start of the message means there's a websocket message event.
			// The 4 signifies a websocket message
			// The 2 signifies a websocket event
			//auto sdata = string(data).substr(0, length);
			//cout << sdata << endl;
			
			if (length && length > 2 && data[0] == '4' && data[1] == '2')
			{
				auto s = hasData(data);

				if (s != "")
				{
					auto j = json::parse(s);

					string event = j[0].get<string>();

					if (event == "telemetry")
					{
						// j[1] is the data JSON object

						CarState state;
						auto jsonInput = j[1];

						state.x = jsonInput["x"];
						state.y = jsonInput["y"];
						state.s = jsonInput["s"];
						state.d = jsonInput["d"];
						state.yaw = jsonInput["yaw"];
						state.speed = jsonInput["speed"];

						auto prev_path_x = jsonInput["previous_path_x"];
						auto prev_path_y = jsonInput["previous_path_y"];
						for (auto i = 0; i < prev_path_x.size(); i++)
						{
							state.previous_path_x.push_back(prev_path_x[i]);
							state.previous_path_y.push_back(prev_path_y[i]);
						}

						// Sensor Fusion Data, a list of all other cars on the same side of the road.
						auto sensor_fusion = j[1]["sensor_fusion"];
						vector<DetectedCarState> detected_cars;

						for (int i = 0 ; i < sensor_fusion.size(); i++)
						{
							auto detected_sensor = sensor_fusion[i];
							DetectedCarState detected_car;
							detected_car.id = detected_sensor[0];
							detected_car.x = detected_sensor[1];
							detected_car.y = detected_sensor[2];
							detected_car.vx = detected_sensor[3];
							detected_car.vy = detected_sensor[4];
							detected_car.s = detected_sensor[5];
							detected_car.d = detected_sensor[6];
							detected_cars.push_back(detected_car);
						}

						car.Process(state, detected_cars);

						json msgJson;
						
						msgJson["next_x"] = car.path_module_.path_x_;
						msgJson["next_y"] = car.path_module_.path_y_;

						auto msg = "42[\"control\"," + msgJson.dump() + "]";

						//this_thread::sleep_for(chrono::milliseconds(1000));
						ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
					}
				}
				else
				{
					// Manual driving
					std::string msg = "42[\"manual\",{}]";
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
		});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse* res, uWS::HttpRequest req, char* data,
			size_t, size_t)
		{
			const std::string s = "<h1>Hello world!</h1>";
			if (req.getUrl().valueLength == 1)
			{
				res->end(s.data(), s.length());
			}
			else
			{
				// i guess this should be done more gracefully?
				res->end(nullptr, 0);
			}
		});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
		{
			std::cout << "Connected!!!" << std::endl;
		});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
			char* message, size_t length)
		{
			ws.close();
			std::cout << "Disconnected" << std::endl;
		});

	int port = 4567;
	if (h.listen(port))
	{
		std::cout << "Listening to port " << port << std::endl;
	}
	else
	{
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
