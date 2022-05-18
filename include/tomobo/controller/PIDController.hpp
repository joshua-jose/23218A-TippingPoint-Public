#pragma once

#include "tomobo/controller/FeedbackController.hpp"
#include <limits>

namespace tomobo {

class PIDController : public FeedbackController {

public:
    struct Gains {
        double kP, kI, kD;
    };

    PIDController(double ikP, double ikI, double ikD, bool iresetOnCross = false,
        bool islewEnabled = false, okapi::QTime islewTime = 1_s);
    PIDController(struct Gains gains);
    PIDController(const PIDController& other);

    ~PIDController() = default;

    double step(double reading) override;
    void reset() override;

    void setGains(double ikP, double ikI, double ikD);
    void setGains(struct Gains gains);
    struct Gains getGains();

    void setResetOnCross(bool input);
    void setIntegrateOnSaturation(bool input);
    void setIntegralSumBounds(double errorMin, double errorMax);
    void setIntegralMax(double input);

protected:
    double kP, kI, kD;
    double lastError{ 0 }, integral{ 0 };

    bool resetOnCross;
    bool integrateOnSaturation{ true };

    double integralMaxSum = std::numeric_limits<double>::max();
    double integralMinSum = 0;

    double integralMax = std::numeric_limits<double>::max();
};

}
