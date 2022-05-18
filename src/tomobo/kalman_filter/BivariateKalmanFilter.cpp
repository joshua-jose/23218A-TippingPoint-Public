#include "tomobo/kalman_filter/BivariateKalmanFilter.hpp"

namespace tomobo {

BivariateKalmanFilter::BivariateKalmanFilter(double imeasurement_std, Gaussian ivel_model,
    Gaussian ix, Gaussian idx) :
    measurement_std(imeasurement_std),
    vel_model(ivel_model),
    x(ix), dx(idx){};

static Gaussian empty{ 0, 0 }; // exists just so the redacted functions can return smth

Gaussian& BivariateKalmanFilter::predict() {
    // redacted
    return empty;
};

Gaussian& BivariateKalmanFilter::update(okapi::QLength pos_reading, okapi::QSpeed vel_reading) {
    // redacted
    return empty;
};

}