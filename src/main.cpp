#include <iostream>
#include <tuple>
#include "io.hpp"

const int PORT = 4567;


int main() {
  std::cout << "Connecting to simulator" << std::endl;
  SimIO simulator(PORT, [&](double hunter_x, double hunter_y,
    double hunter_heading, MeasurementPackage lidar_m, MeasurementPackage radar_m) {
    double heading_diff, dist_diff;
    

    return std::make_tuple(heading_diff, dist_diff);
  });
  simulator.run();

  return 0;
}