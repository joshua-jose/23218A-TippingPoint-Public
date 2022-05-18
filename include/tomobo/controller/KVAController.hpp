#pragma once

#include "tomobo/controller/FeedbackController.hpp"

namespace tomobo {

class KVAController : public FeedbackController {
public:
    struct Gains {
        double kV, kA, kP;
    };

    KVAController(double ikV, double ikA, double ikP,
        bool iresetOnCross = false, bool islewEnabled = false, okapi::QTime islewTime = 1_s);
    KVAController(struct Gains gains);
    KVAController(const KVAController& other);

    ~KVAController() = default;

    void setTarget(double newTarget);
    void setTarget(double newTarget, double newAccelTarget);

    double step(double ierror);
    void reset();

    void setGains(double ikV, double ikA, double ikP);
    void setGains(struct Gains gains);
    struct Gains getGains();

protected:
    double kV, kA, kP;

    double accelTarget = 0;
};

}
