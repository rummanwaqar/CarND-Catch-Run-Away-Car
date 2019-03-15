#include <iostream>
#include <tuple>
#include "ukf.hpp"
#include "io.hpp"
#include <fstream>
#include <sstream>
#include "matplotlibcpp.h"

const int PORT = 4567;

int main() {
  Ukf ukf;

  std::cout << "Connecting to simulator" << std::endl;
  SimIO simulator(PORT, [&](double hunter_x, double hunter_y,
    double hunter_heading, MeasurementPackage lidar_m, MeasurementPackage radar_m) {
    ukf.processMeasurement(lidar_m);
    ukf.processMeasurement(radar_m);

    double time_in_future = 0.6;
    double target_x = ukf.x_[0] + ukf.x_[2] * cos(ukf.x_[3]) * time_in_future;
    double target_y = ukf.x_[1] + ukf.x_[2] * sin(ukf.x_[3]) * time_in_future;

    double dist_diff = sqrt((target_y - hunter_y)*(target_y - hunter_y)
      + (target_x - hunter_x)*(target_x - hunter_x));

    double heading_to_target = atan2(target_y - hunter_y, target_x - hunter_x);
	  while (heading_to_target > M_PI) heading_to_target-=2.*M_PI;
	  while (heading_to_target <-M_PI) heading_to_target+=2.*M_PI;
	  //turn towards the target
	  double heading_difference = heading_to_target - hunter_heading;
	  while (heading_difference > M_PI) heading_difference-=2.*M_PI;
	  while (heading_difference <-M_PI) heading_difference+=2.*M_PI;

    return std::make_tuple(heading_difference, dist_diff);
  });
  simulator.run();
  return 0;
}
