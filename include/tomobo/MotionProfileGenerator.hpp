#pragma once

#include "okapi/api/control/async/asyncMotionProfileController.hpp"

extern "C" {
#include "okapi/pathfinder/include/pathfinder.h"
}

class MotionProfileGenerator : public okapi::AsyncMotionProfileController {
public:
    MotionProfileGenerator(const okapi::PathfinderLimits& ilimits, const okapi::ChassisScales& iscales);

    //void executeSinglePath(const TrajectoryPair& path);

    struct TrajectoryPair {
        SegmentPtr left;
        SegmentPtr right;
        int length;
    };
    TrajectoryPair getPath(std::string pathId);

private:
    std::shared_ptr<okapi::ChassisModel> model;
    // Since the above pointer never gets initialised... we can't call anything that may  try to access it!
    void loop();
    virtual void executeSinglePath(const TrajectoryPair& path, std::unique_ptr<okapi::AbstractRate> rate);
};

/*
class FakeChassisModel : public okapi::ReadOnlyChassisModel {
    std::valarray<std::int32_t> getSensorVals() const {
        return { 0 };
    };
};
*/