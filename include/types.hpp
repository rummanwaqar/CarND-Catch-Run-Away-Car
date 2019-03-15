#pragma once

#include "Eigen/Dense"

class MeasurementPackage {
public:
  long timestamp;

  enum SensorType {
    LASER,
    RADAR
  } sensor_type;

  Eigen::VectorXd raw_measurements;
};
