#pragma once

#include "main.h"
#include <sstream>

class GUI {
public:
    static std::shared_ptr<GUI> get();

    void gui_build();
    void build_main(lv_obj_t* parent);
    void build_diagnostics(lv_obj_t* parent);
    static void build_control(lv_obj_t* parent);
    void build_console(lv_obj_t* parent);

    lv_obj_t* console_box;
    lv_obj_t* arm_temp_guage;
    lv_obj_t* chassis_temp_guage;
    lv_obj_t* claw_temp_guage;
    lv_obj_t* heading_label;

    lv_obj_t* splash;

    std::vector<std::string> console_buffer;
    void update_console();
    void set_line(int line, std::string contents);
    void add_line(std::string contents);
    void update_inertial_status(pros::Imu inertial);
    void set_splash_hidden(bool hidden);

protected:
    GUI();
    int state;
    lv_theme_t* th = lv_theme_alien_init(210, &lv_font_dejavu_20);

    static lv_res_t cb_auton_select(lv_obj_t* auton_select);
    static lv_res_t cb_side(lv_obj_t* side);
    static lv_res_t cb_splash(lv_obj_t* splash);
    static lv_res_t cb_splash_control(lv_obj_t* splash_control);
};
