#include "tomobo/MotionProfileGenerator.hpp"
#include "okapi/api/chassis/model/chassisModel.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"

MotionProfileGenerator::MotionProfileGenerator(const okapi::PathfinderLimits& ilimits, const okapi::ChassisScales& iscales) :
    AsyncMotionProfileController(okapi::TimeUtilFactory::createDefault(), ilimits, model, iscales, okapi::AbstractMotor::gearset::green) {}

// crashes :)
MotionProfileGenerator::TrajectoryPair MotionProfileGenerator::getPath(std::string pathId) {
    auto path = paths.find(currentPath);

    if (path == paths.end())
        return MotionProfileGenerator::TrajectoryPair{ SegmentPtr(nullptr, free), SegmentPtr(nullptr, free), -1 };

    // Get underlying Segment* ptr and create new struct
    auto left = path->second.left.get();
    auto right = path->second.right.get();
    return { SegmentPtr(left, free), SegmentPtr(right, free), path->second.length };
}

void MotionProfileGenerator::executeSinglePath(const TrajectoryPair& path, std::unique_ptr<okapi::AbstractRate> rate){

};
