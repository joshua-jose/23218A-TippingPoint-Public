#include "tomobo/controller/KVAController.hpp"

namespace tomobo {
KVAController::KVAController(double ikV, double ikA, double ikP,
    bool iresetOnCross, bool islewEnabled, okapi::QTime islewTime) {

    setGains(ikV, ikA, ikP);

    slewEnabled = islewEnabled;
    slewTime = islewTime;
};

KVAController::KVAController(struct Gains gains) :
    KVAController(gains.kV, gains.kA, gains.kP){};

KVAController::KVAController(const KVAController& other) :
    KVAController(other.kV, other.kA, other.kP){};

double KVAController::step(double ierror) {
    if (disabled)
        return 0;

    okapi::QTime DtCurrent = peekDt();

    if (DtCurrent >= sampleRate) {
        getDt();
        error = ierror;

        if (accelTarget == 0) {
            //accelTarget = (target - lastTarget) / Dt.convert(okapi::second);
        }
        //lastTarget = target;

        output = (kV * target) + (kA * accelTarget) + (kP * error);
        output = std::clamp(output, minOutput, maxOutput);
        output = slew(output);
        settledUtil->isSettled(error);
    }
    return output;
};

void KVAController::reset() {
    error = 0;
    target = 0;
    accelTarget = 0;
    //lastTarget = 0;
    output = 0;
    slewLastInput = 0;
    settledUtil->reset();
};

void KVAController::setTarget(double newTarget) {
    target = newTarget;
    accelTarget = 0;
};
void KVAController::setTarget(double newTarget, double newAccelTarget) {
    target = newTarget;
    accelTarget = newAccelTarget;
};

void KVAController::setGains(double ikV, double ikA, double ikP) {
    kV = ikV;
    kA = ikA;
    kP = ikP;
};
void KVAController::setGains(struct Gains gains) {
    setGains(gains.kV, gains.kA, gains.kP);
};

KVAController::Gains KVAController::getGains() {
    return { kV, kA, kP };
};

}
