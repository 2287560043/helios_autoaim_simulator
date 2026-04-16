#include "../include/node_params.hpp"

#include <filesystem>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace helios_cv
{
namespace
{

std::string trim(const std::string& text)
{
    size_t begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }
    size_t end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

std::string strip_comment(const std::string& text)
{
    bool in_single = false;
    bool in_double = false;
    for (size_t i = 0; i < text.size(); i++) {
        char ch = text[i];
        if (ch == '\'' && !in_double) {
            in_single = !in_single;
            continue;
        }
        if (ch == '"' && !in_single) {
            in_double = !in_double;
            continue;
        }
        if (ch == '#' && !in_single && !in_double) {
            return text.substr(0, i);
        }
    }
    return text;
}

std::string normalize_key(std::string key)
{
    key = trim(key);
    while (!key.empty() && key.front() == '/') {
        key.erase(key.begin());
    }
    if (key.size() >= 2 && ((key.front() == '"' && key.back() == '"') || (key.front() == '\'' && key.back() == '\''))) {
        key = key.substr(1, key.size() - 2);
    }
    return key;
}

std::string normalize_value(std::string value)
{
    value = trim(value);
    if (value.size() >= 2 && ((value.front() == '"' && value.back() == '"') || (value.front() == '\'' && value.back() == '\''))) {
        value = value.substr(1, value.size() - 2);
    }
    return value;
}

std::map<std::string, std::string> parse_yaml_scalars(const std::string& path)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("failed to open params file: " + path);
    }

    std::map<std::string, std::string> values;
    std::vector<std::pair<int, std::string>> path_stack;
    std::string line;
    while (std::getline(file, line)) {
        std::string clean = trim(strip_comment(line));
        if (clean.empty() || clean.front() == '-') {
            continue;
        }

        int indent = 0;
        while (indent < static_cast<int>(line.size()) && line[static_cast<size_t>(indent)] == ' ') {
            indent += 1;
        }

        size_t colon = clean.find(':');
        if (colon == std::string::npos) {
            continue;
        }

        std::string key = normalize_key(clean.substr(0, colon));
        std::string value = normalize_value(clean.substr(colon + 1));
        while (!path_stack.empty() && indent <= path_stack.back().first) {
            path_stack.pop_back();
        }

        std::string full_path;
        for (const auto& item: path_stack) {
            full_path += "/" + item.second;
        }
        full_path += "/" + key;

        if (value.empty()) {
            path_stack.emplace_back(indent, key);
            continue;
        }
        values[full_path] = value;
    }
    return values;
}

double require_double(const std::map<std::string, std::string>& values, const std::string& path)
{
    auto it = values.find(path);
    if (it == values.end()) {
        throw std::runtime_error("missing params entry: " + path);
    }
    try {
        return std::stod(it->second);
    } catch (const std::exception&) {
        throw std::runtime_error("invalid numeric params entry: " + path + " = " + it->second);
    }
}

} // namespace

std::string find_default_params_file()
{
    for (const auto& candidate: {
             std::string("autoaim_bring_up/config/node_params.yaml"),
             std::string("insentry/autoaim_bring_up/config/node_params.yaml"),
             std::string("../autoaim_bring_up/config/node_params.yaml")})
    {
        if (std::filesystem::exists(candidate)) {
            return candidate;
        }
    }
    return "";
}

TrackerParamSet load_tracker_params(const std::string& path)
{
    auto values = parse_yaml_scalars(path);

    TrackerParamSet params;
    params.top = {
        require_double(values, "/armor_predictor/ros__parameters/standard_observer/ekf/q_xy"),
        require_double(values, "/armor_predictor/ros__parameters/standard_observer/ekf/q_vxy"),
        require_double(values, "/armor_predictor/ros__parameters/standard_observer/ekf/q_z"),
        require_double(values, "/armor_predictor/ros__parameters/standard_observer/ekf/q_vz"),
        require_double(values, "/armor_predictor/ros__parameters/standard_observer/ekf/q_yaw"),
        require_double(values, "/armor_predictor/ros__parameters/standard_observer/ekf/q_vyaw"),
        require_double(values, "/armor_predictor/ros__parameters/standard_observer/ekf/q_r"),
        require_double(values, "/armor_predictor/ros__parameters/standard_observer/ekf/r_xy_factory"),
        require_double(values, "/armor_predictor/ros__parameters/standard_observer/ekf/r_z"),
        require_double(values, "/armor_predictor/ros__parameters/standard_observer/ekf/r_yaw_factory"),
    };
    params.simple = {
        params.top.q_xy,
        params.top.q_vxy,
        params.top.q_z,
        params.top.q_vz,
        params.top.r_xy_factory,
        params.top.r_z,
    };
    params.singer = {
        params.top.q_xy,
        params.top.q_vxy,
        params.top.q_z,
        params.top.q_vz,
        params.top.r_xy_factory,
        params.top.r_z,
    };
    params.top3 = {
        require_double(values, "/armor_predictor/ros__parameters/outpost_observer/ekf/q_xyz"),
        require_double(values, "/armor_predictor/ros__parameters/outpost_observer/ekf/q_xyz"),
        require_double(values, "/armor_predictor/ros__parameters/outpost_observer/ekf/q_xyz"),
        require_double(values, "/armor_predictor/ros__parameters/outpost_observer/ekf/q_yaw"),
        require_double(values, "/armor_predictor/ros__parameters/outpost_observer/ekf/q_vyaw"),
        require_double(values, "/armor_predictor/ros__parameters/outpost_observer/ekf/q_r"),
        require_double(values, "/armor_predictor/ros__parameters/outpost_observer/ekf/q_dz"),
        require_double(values, "/armor_predictor/ros__parameters/outpost_observer/ekf/r_xyz_factory"),
        require_double(values, "/armor_predictor/ros__parameters/outpost_observer/ekf/r_xyz_factory"),
        require_double(values, "/armor_predictor/ros__parameters/outpost_observer/ekf/r_xyz_factory"),
        require_double(values, "/armor_predictor/ros__parameters/outpost_observer/ekf/r_yaw_factory"),
    };
    return params;
}

} // namespace helios_cv
