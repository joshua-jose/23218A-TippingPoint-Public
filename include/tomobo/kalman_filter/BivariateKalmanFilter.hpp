#pragma once

#include "tomobo/kalman_filter/Gaussian.hpp"
#include "tomobo/units.hpp"

namespace tomobo {
class BivariateKalmanFilter {
public:
    BivariateKalmanFilter(double measurement_std, Gaussian vel_model,
        Gaussian x = { 0, 500 }, Gaussian dx = { 0, 500 });

    Gaussian& predict();
    Gaussian& update(okapi::QLength pos_reading, okapi::QSpeed vel_reading);

    // All values should be user initialised
    Gaussian x{ 0, 500 };
    Gaussian dx{ 0, 500 };

    double measurement_std = 0;
    Gaussian vel_model{ 0, 0 };

    okapi::QTime dt = 0.01 * okapi::second;
};
}