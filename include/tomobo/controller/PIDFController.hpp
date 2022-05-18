#pragma once

#include "tomobo/controller/PIDController.hpp"

namespace tomobo {

class PIDFController : public PIDController {
public:
    struct Gains {
        double kP, kI, kD, kF;
    };

    PIDFController(double ikP, double ikI, double ikD, double ikF,
        bool iresetOnCross = false, bool islewEnabled = false, okapi::QTime islewTime = 1_s);
    PIDFController(struct Gains gains);
    PIDFController(const PIDFController& other);

    ~PIDFController() = default;

    double step(double reading) override;

    void setGains(double ikP, double ikI, double ikD, double ikF);
    void setGains(struct Gains gains);
    struct Gains getGains();

protected:
    double kF;
};

}
