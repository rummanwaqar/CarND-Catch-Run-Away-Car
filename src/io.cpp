#include "io.hpp"

SimIO::SimIO(int port, ProcessCb cb) : port_(port), callbackFunc_(cb) {
  /*
   * Register event handlers for uWS
   */
  h_.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      std::string s = hasData(std::string(data));

      if(s != "") { // data available
        // parse json
        auto j = nlohmann::json::parse(s);
        std::string event = j[0].get<std::string>();

        if(event == "telemetry") {
          // read hunter state
          double hunter_x = std::stod(j[1]["hunter_x"].get<std::string>());
          double hunter_y = std::stod(j[1]["hunter_y"].get<std::string>());
          double hunter_heading = std::stod(j[1]["hunter_heading"].get<std::string>());

          // reads raw_measurements
          auto lidar_measurement = mkMeasurementPackage(j[1]["lidar_measurement"]);
          auto radar_measurement = mkMeasurementPackage(j[1]["radar_measurement"]);

          // process
          double heading_diff, distance_diff;
          std::tie(heading_diff, distance_diff) = callbackFunc_(hunter_x,
            hunter_y, hunter_heading, lidar_measurement, radar_measurement);
            
          // send output
          nlohmann::json msgJson;
          msgJson["turn"] = heading_diff;
          msgJson["dist"] = distance_diff;
          auto msg = "42[\"move_hunter\"," + msgJson.dump() + "]";
          // // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  h_.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h_.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });
}

void SimIO::run() {
  // listen and wait for connection
  if (h_.listen(port_)) {
    std::cout << "Listening to port " << port_ << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return;
  }
  // endless loop until application exists
  h_.run();
}

std::string SimIO::hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

MeasurementPackage SimIO::mkMeasurementPackage(std::string str) {
  MeasurementPackage pkg;

  std::istringstream iss(str);
  std::string type;
  iss >> type;

  if(type == "L") {
    pkg.sensor_type = MeasurementPackage::LASER;
    pkg.raw_measurements = Eigen::VectorXd(2);
    float px, py;
    iss >> px;
    iss >> py;
    pkg.raw_measurements << px, py;
  } else if (type == "R") {
    pkg.sensor_type = MeasurementPackage::RADAR;
    pkg.raw_measurements = Eigen::VectorXd(3);
    float ro, theta, ro_dot;
    iss >> ro;
    iss >> theta;
    iss >> ro_dot;
    pkg.raw_measurements << ro , theta, ro_dot;
  }

  long timestamp;
  iss >> timestamp;
  pkg.timestamp = timestamp;

  return pkg;
}
