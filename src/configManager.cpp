#include "project/configManager.hpp"
#include <fstream>
#include <sstream>

void ConfigManager::save_config() {
    printf("Saving config");
    std::ofstream save_file(save_filepath, std::ofstream::out | std::ofstream::trunc);
    save_file.clear();
    save_file << selected_auton << "\n";
    save_file.close();
}

void ConfigManager::load_config() {
    bool file_exists;
    if (FILE* file = fopen(save_filepath.c_str(), "r")) {
        fclose(file);
        file_exists = true;
    } else {
        file_exists = false;
    }

    if (file_exists) {
        std::ifstream input_file(save_filepath);
        char temp_string[256];

        input_file.getline(temp_string, 256);
        selected_auton = std::stoi(temp_string);

        input_file.close();
    }
}

void ConfigManager::register_auton(std::string name, auton_func func) {
    auto routine = std::make_tuple(name, func, tomobo::Pose());
    auton_routines.push_back(routine);
};
void ConfigManager::register_auton(std::string name, auton_func func, tomobo::Pose state) {
    auto routine = std::make_tuple(name, func, state);
    auton_routines.push_back(routine);
};

auton_func ConfigManager::get_auton_func(int id) {
    return std::get<1>(auton_routines[id]);
}
std::string ConfigManager::get_auton_name(int id) {
    return std::get<0>(auton_routines[id]);
}
tomobo::Pose ConfigManager::get_auton_state(int id) {
    return std::get<2>(auton_routines[id]);
}

void ConfigManager::select_auton(int id) {
    //if (id > auton_routines.size()){
    //if (auton_routines.size() == 0)
    //  throw std::range_error("An auton must exist! This is a nullptr catch");
    //selected_auton = 0;
    //}
    //else
    selected_auton = id;
    this->save_config();
}