#include "project/chassis.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"
#include "project/robot.hpp"

const char tracking_left_port = 'C', tracking_right_port = 'G', tracking_middle_port = 'E';

const ChassisScales ctrackingScales{
    { 2.7668 * inch, 9.223 * inch, // wheel diam, wheelbase diam
        3.72 * inch, 2.772 * inch }, // middle wheel distance, middle wheel diam
    okapi::quadEncoderTPR
};

const ChassisScales chassisScales{
    { 4.125 * inch, 15 * inch }, // wheel diam, wheelbase diam
    okapi::imev5RedTPR
};

/*
pros::ADIAnalogIn line_middle (line_middle_port);
pros::ADIAnalogIn line_right (line_right_port);
const okapi::QLength line_tracker_width = 9.25_in;
*/

Chassis::Chassis() :
    fl_motor(FL_PORT, pros::E_MOTOR_GEARSET_36, true),
    fr_motor(FR_PORT, pros::E_MOTOR_GEARSET_36),
    bl_motor(BL_PORT, pros::E_MOTOR_GEARSET_36, true),
    br_motor(BR_PORT, pros::E_MOTOR_GEARSET_36),

    odometry(trackingScales, tracking_left, tracking_right, tracking_mid, inertial),

    tomobo::TankDriveController(ctrackingScales, chassisScales, &odometry,
        //                                     resetOnCross, slewEnabled
        std::make_unique<PIDController>(2.154687, 0, 0.02875, false, true, 0.1_s), // Forward distance controller
        std::make_unique<PIDController>(0.539, 0, 0.010938, false, true, 0.0_s) // Angle Controller
        ),

    // Create encoder objects
    tracking_left({ 20, tracking_left_port, tracking_left_port + 1 }, false),
    tracking_right({ 20, tracking_right_port, tracking_right_port + 1 }, true),
    tracking_mid({ 20, tracking_middle_port, tracking_middle_port + 1 }, false),
    inertial(INERTIAL_PORT) {

    inertial.set_data_rate(5);

    // Set up config for settled utils
    forwardDistanceController->setSettledUtil(
        okapi::TimeUtilFactory()
            .withSettledUtilParams(0.02, 0.005, 200_ms)
            .getSettledUtil());
    angleController->setSettledUtil(
        okapi::TimeUtilFactory()
            .withSettledUtilParams(0.035, 0.01, 200_ms)
            .getSettledUtil());
};

float kGrad = 0.8f; // Gradient of the curve
float kScaleX = 127.f; // Input Scale
float kScaleY = 12000.f; // Output Scale
float kMixingScale = 127.f; // Mixing Rate
float kMixingMin = 40.f; // Min Mixing Value

void Chassis::oc_update(int32_t joy_y, int32_t joy_a, bool brake_mode) {
    static bool current_brake_mode = false;
    /*
    if (fl_motor.get_temperature() > 50 || fr_motor.get_temperature() > 50 || bl_motor.get_temperature() > 50 || br_motor.get_temperature() > 50) {
        if (!pros::competition::is_connected()) {

            printf("Motor overheat!\n");
            fl_motor.move_voltage(0);
            fr_motor.move_voltage(0);
            bl_motor.move_voltage(0);
            br_motor.move_voltage(0);
            //throw std::runtime_error("Overheat");
            return;
        }
    }
    */

    // curteezy drive function
    float x = joy_a;
    float y = joy_y;

    float Mixing = std::clamp(y, kMixingMin, kMixingScale);

    // curve redacted (not mine)
    float vy = y;
    float va = x;

    std::int16_t flv = vy + va;
    std::int16_t frv = vy - va;
    std::int16_t blv = vy + va;
    std::int16_t brv = vy - va;

    if (!current_brake_mode && brake_mode) {
        fl_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        fr_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        bl_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        br_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    } else if (current_brake_mode && !brake_mode) {
        fl_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        fr_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        bl_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        br_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }
    current_brake_mode = brake_mode;

    if (brake_mode) {
        if (abs(flv) < 1)
            flv = 0;
        if (abs(frv) < 1)
            frv = 0;
        if (abs(blv) < 1)
            blv = 0;
        if (abs(brv) < 1)
            brv = 0;
        fl_motor.move_velocity((flv * 100) / 12000);
        fr_motor.move_velocity((frv * 100) / 12000);
        bl_motor.move_velocity((blv * 100) / 12000);
        br_motor.move_velocity((brv * 100) / 12000);
    } else {
        fl_motor.move_voltage(flv);
        fr_motor.move_voltage(frv);
        bl_motor.move_voltage(blv);
        br_motor.move_voltage(brv);
    }
};

void Chassis::updateSensors() {
    leftEncoder = tracking_left.get_value();
    rightEncoder = tracking_right.get_value();
    midEncoder = tracking_mid.get_value();
};
void Chassis::updateMotors() {
    // if (!pros::competition::is_autonomous())
    //     return;
    /*
    if (fl_motor.get_temperature() > 50 || fr_motor.get_temperature() > 50 || bl_motor.get_temperature() > 50 || br_motor.get_temperature() > 50) {
        printf("Motor overheat!\n");
        fl_motor.move_voltage(0);
        fr_motor.move_voltage(0);
        bl_motor.move_voltage(0);
        br_motor.move_voltage(0);

        throw std::runtime_error("Overheat");
        return;
    }
    */
    if (useVelocity) {
        std::int32_t leftv = std::max((double)-100, std::min(std::round(leftVelocity.load(std::memory_order_acquire) * 100), (double)100));
        std::int32_t rightv = std::max((double)-100, std::min(std::round(rightVelocity.load(std::memory_order_acquire) * 100), (double)100));

        fl_motor.move_velocity(leftv);
        fr_motor.move_velocity(rightv);
        bl_motor.move_velocity(leftv);
        br_motor.move_velocity(rightv);

    } else {
        // Take numbers from -1 ... 1, and turn them into -12,000 ... 12,000
        std::int32_t leftv = std::max((double)-12000, std::min(std::round(leftPower.load(std::memory_order_acquire) * 12000), (double)12000));
        std::int32_t rightv = std::max((double)-12000, std::min(std::round(rightPower.load(std::memory_order_acquire) * 12000), (double)12000));

        fl_motor.move_voltage(leftv);
        fr_motor.move_voltage(rightv);
        bl_motor.move_voltage(leftv);
        br_motor.move_voltage(rightv);
    }
};

void Chassis::driveStraightHandler() {
    double left = (leftEncoder - leftStart) / trackingScales.straight;
    double right = (rightEncoder - rightStart) / trackingScales.straight;

    QAngle angle = ((left - right) / trackingScales.wheelTrack.convert(okapi::meter)) * radian;
    double distanceError = driveStraightTarget.convert(meter) - ((left + right) / (2.0));

    double straightOut = forwardDistanceController->step(distanceError);
    double angleOut = 0; // angleController->step((-angle).convert(radian)) / 4;

    leftPower = straightOut + angleOut;
    rightPower = straightOut - angleOut;
};

/*
void Chassis::turnHandler() {
    // double left = (leftEncoder - leftStart) / trackingScales.straight;
    // double right = (rightEncoder - rightStart) / trackingScales.straight;

    QAngle angle = (inertial.get_rotation() - angleStart) * degree;

    double angleOut = angleController->step((angleTarget - angle).convert(radian));

    leftPower = angleOut;
    rightPower = -angleOut;
};
*/