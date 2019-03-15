#ifndef IO_H
#define IO_H

#include <iostream>
#include <sstream>
#include <fstream>
#include <tuple>
#include <string>
#include <functional>
#include <uWS/uWS.h>
#include "json.h"

#include "types.hpp"

// callback function definition
// takes hunter state and sensor measurements
// returns angle and distance diff to target
typedef std::function< std::tuple<double, double>(double hunter_x,
  double hunter_y, double hunter_heading, MeasurementPackage lidar_m,
  MeasurementPackage radar_m) > ProcessCb;

/*
 * Interface to simulator
 */
class SimIO {
public:
  /*
   * Constructor
   * Creates uWebSocket object and defines all event handlers
   * @param port - port number for simulator uWebSocket
   * @param cb callback for processing function
   */
  SimIO(int port, ProcessCb cb);

  /*
   * Destructor
   */
  ~SimIO() = default;

  /*
   * Initializes connection to simulator and blocks it until simulator is closed.
   * event handling for simulator
   */
  void run();

private:
  /*
   * Checks if the SocketIO event has JSON data.
   * If there is data the JSON object in string format will be returned,
   * else the empty string "" will be returned.
   */
  std::string hasData(std::string s);

  /*
   * make measurement package from string
   */
  MeasurementPackage mkMeasurementPackage(std::string str);

  // uWS object
  uWS::Hub h_;

  // connection port
  int port_;

  // processing callback
  ProcessCb callbackFunc_;
};

  #endif
