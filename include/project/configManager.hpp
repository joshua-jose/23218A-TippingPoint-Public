#pragma once

#include "tomobo/odometry/Pose.hpp"
#include <tuple>
#include <vector>

typedef void (*auton_func)();
typedef std::tuple<std::string, auton_func, tomobo::Pose> auton_routine;

class ConfigManager {
public:
    //ConfigManager();

    std::vector<auton_routine> auton_routines;

    int selected_auton = 0; // Keep it safe from nullptr hopefully
    int selected_team = 1; // Turns negative on red side

    void register_auton(std::string name, auton_func func);
    void register_auton(std::string name, auton_func func, tomobo::Pose state);
    void select_auton(int id);

    auton_func get_auton_func(int id);
    std::string get_auton_name(int id);
    tomobo::Pose get_auton_state(int id);

    void save_config();
    void load_config();

protected:
    std::string save_filepath = "/usd/comp_config.cfg";
};
