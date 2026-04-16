#pragma once

#include "armor_predictor/tracker/SimpleTracker.hpp"
#include "armor_predictor/tracker/SingerTracker.hpp"
#include "armor_predictor/tracker/Top3Tracker.hpp"
#include "armor_predictor/tracker/TopTracker.hpp"
#include "autoaim_utilities/BulletTrajectory.hpp"

#include <angles/angles.h>
#include <opencv2/core.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace helios_cv
{

inline double pi() { return std::acos(-1.0); }
inline double nan_v() { return std::numeric_limits<double>::quiet_NaN(); }

struct PoseSample {
    Eigen::Matrix3d R_c_w = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_c_w = Eigen::Vector3d::Zero();
};

struct GimbalSample {
    double t = 0.0;
    double yaw = 0.0;
    double pitch = 0.0;
};

struct SimConfig {
    std::string tracker = "all";
    std::string standard_mode = "translate_const";
    std::string top_mode = "spin_var";
    std::string outpost_mode = "outpost_standard";
    std::string preset = "default";
    std::string params_file;
    std::string output_dir = "output";
    double fps = 100.0;
    double duration = 20.0;
    int seed = 1;
    double image_delay_ms = 4.0;
    double imu_delay_ms = 1.0;
    double imu_yaw_bias_deg = 0.8;
    double imu_pitch_bias_deg = 0.4;
    double dt_jitter_ratio = 0.03;
    double stutter_prob = 0.01;
    double stutter_mult = 2.5;
    double drop_prob = 0.012;
    int drop_min_frames = 2;
    int drop_max_frames = 6;
    double pos_xy_sigma = 0.008;
    double pos_z_sigma = 0.004;
    double yaw_sigma_deg = 1.5;
    double corner_sigma = 0.6;
    double bullet_speed = 0.0;
    double distance_m = 0.0;
    double latency_ms = 12.0;
    double ego_xy_amp = 0.0;
    double ego_z_amp = 0.0;
    double ego_yaw_amp_deg = 0.0;
    double ego_pitch_amp_deg = 0.0;
    bool help = false;
};

struct CommandRow {
    double t = 0.0;
    double dt_ms = 0.0;
    bool stutter = false;
    bool occluded = false;
    int raw_obs = 0;
    int processed_obs = 0;
    int selected_id = -1;
    int top_level = 0;
    double raw_target_yaw_deg = nan_v();
    double target_yaw_deg = nan_v();
    double truth_target_yaw_deg = nan_v();
    double raw_target_pitch_deg = nan_v();
    double target_pitch_deg = nan_v();
    double truth_target_pitch_deg = nan_v();
    double center_speed = 0.0;
    double reported_vyaw_deg_s = 0.0;
};

struct Report {
    std::string tracker;
    std::string mode;
    std::string csv_path;
    int samples = 0;
    int future_samples = 0;
    int selected = 0;
    int occluded_frames = 0;
    int stutter_frames = 0;
    int issues = 0;
    double max_cmd_yaw_step_deg = 0.0;
    double max_cmd_pitch_step_deg = 0.0;
    double max_future_yaw_step_deg = 0.0;
    double max_future_pitch_step_deg = 0.0;
    double max_cmd_yaw_rate_deg_s = 0.0;
    double max_future_yaw_rate_deg_s = 0.0;
    double max_cmd_yaw_err_deg = 0.0;
    double max_cmd_pitch_err_deg = 0.0;
    double max_future_yaw_err_deg = 0.0;
    double max_future_pitch_err_deg = 0.0;
    double max_center_speed = 0.0;
    double max_vyaw_deg_s = 0.0;
    std::vector<std::string> details;

    void add_issue(const std::string& text)
    {
        issues += 1;
        if (details.size() < 6) {
            details.push_back(text);
        }
    }

    bool pass() const
    {
        return issues == 0
            && max_cmd_yaw_step_deg < 20.0
            && max_future_yaw_step_deg < 24.0
            && max_cmd_pitch_step_deg < 12.0
            && max_future_pitch_step_deg < 14.0
            && max_cmd_yaw_rate_deg_s < 1200.0
            && max_future_yaw_rate_deg_s < 1400.0
            && max_center_speed < 15.0
            && max_vyaw_deg_s < 1200.0;
    }
};

struct SelectorState {
    int top0_lock_id = -1;
    int lock_id = -1;
    double last_lock_t = 0.0;
};

struct CommandResult {
    Aim hit;
    double yaw_deg = nan_v();
    double pitch_deg = nan_v();
    double fly_time = 0.0;
    bool solvable = false;
};

inline double uniform01(std::mt19937& rng)
{
    return std::uniform_real_distribution<double>(0.0, 1.0)(rng);
}

inline double resolved_distance(const SimConfig& cfg, const std::string& tracker_family)
{
    if (cfg.distance_m > 0.0) return cfg.distance_m;
    if (tracker_family == "outpost") return 5.0;
    if (tracker_family == "top") return 3.0;
    return 1.5;
}

inline double resolved_bullet_speed(const SimConfig& cfg, bool outpost)
{
    if (cfg.bullet_speed > 0.0) return cfg.bullet_speed;
    return outpost ? 11.8 : 22.8;
}

inline std::string describe_mode(const std::string& mode, const std::string& preset)
{
    return preset == "default" ? mode : mode + " | " + preset;
}

inline double gauss(std::mt19937& rng, double sigma)
{
    return std::normal_distribution<double>(0.0, sigma)(rng);
}

inline int randint(std::mt19937& rng, int low, int high)
{
    return std::uniform_int_distribution<int>(low, high)(rng);
}

inline std::string safe_name(std::string text)
{
    for (auto& ch: text) {
        if (!std::isalnum(static_cast<unsigned char>(ch))) {
            ch = '_';
        }
    }
    return text;
}

inline std::string value_tag(double value, const std::string& suffix)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(1) << value;
    return safe_name(os.str()) + suffix;
}

inline bool finite_vec(const Eigen::Vector3d& v)
{
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

inline double command_yaw(const Eigen::Vector3d& pos)
{
    return std::atan2(pos.y(), pos.x());
}

inline double command_pitch(const Eigen::Vector3d& pos)
{
    return std::atan2(pos.z(), std::hypot(pos.x(), pos.y()));
}

inline double gimbal_yaw_at(double t, double yaw_bias)
{
    return angles::from_degrees(2.0) * std::sin(0.9 * t) + yaw_bias;
}

inline double gimbal_pitch_at(double t, double pitch_bias)
{
    return -angles::from_degrees(3.0) + angles::from_degrees(1.2) * std::sin(0.4 * t + 0.2) + pitch_bias;
}

inline CameraParam make_camera()
{
    CameraParam cam;
    cam.K = (cv::Mat_<double>(3, 3) << 1200.0, 0.0, 640.0, 0.0, 1200.0, 360.0, 0.0, 0.0, 1.0);
    cam.width = 1280;
    cam.height = 720;
    return cam;
}

inline Eigen::Matrix3d base_camera_pose()
{
    Eigen::Matrix3d R_c_w;
    R_c_w << 0.0, 0.0, 1.0,
        -1.0, 0.0, 0.0,
        0.0, -1.0, 0.0;
    return R_c_w;
}

inline void record_gimbal_sample(std::vector<GimbalSample>& history, double t, double yaw, double pitch)
{
    if (!history.empty() && t <= history.back().t) {
        history.back() = {t, yaw, pitch};
        return;
    }
    history.push_back({t, yaw, pitch});
}

inline GimbalSample sample_gimbal(const std::vector<GimbalSample>& history, double t)
{
    if (history.empty()) {
        return {};
    }
    if (t <= history.front().t) {
        return history.front();
    }
    for (size_t i = history.size(); i > 0; i--) {
        if (history[i - 1].t <= t) {
            return history[i - 1];
        }
    }
    return history.front();
}

inline void latch_gimbal_command(const CommandResult& cmd, double& yaw, double& pitch)
{
    if (cmd.hit.id < 0 || !std::isfinite(cmd.yaw_deg) || !std::isfinite(cmd.pitch_deg)) {
        return;
    }
    yaw = angles::from_degrees(cmd.yaw_deg);
    pitch = angles::from_degrees(cmd.pitch_deg);
}

inline double truth_pitch_deg(const Aim& hit, double bullet_speed, double gimbal_yaw)
{
    if (hit.id < 0 || !finite_vec(hit.aim_pos)) return nan_v();
    Trajectory traj(hit.aim_pos, bullet_speed, gimbal_yaw);
    return angles::to_degrees(traj.solvable() ? traj.get_pitch() : command_pitch(hit.aim_pos));
}

inline CommandResult raw_observation_command(const Observation& obs, double bullet_speed, double gimbal_yaw)
{
    CommandResult result;
    result.hit.id = obs.id;
    result.hit.aim_pos = obs.pos;
    Trajectory traj(obs.pos, bullet_speed, gimbal_yaw);
    result.fly_time = traj.solvable() ? traj.get_flyTime() : 0.0;
    result.solvable = traj.solvable();
    result.yaw_deg = angles::to_degrees(traj.solvable() ? traj.get_yaw() : command_yaw(obs.pos));
    result.pitch_deg = angles::to_degrees(traj.solvable() ? traj.get_pitch() : command_pitch(obs.pos));
    return result;
}

inline const Observation* find_selected_observation(
    const std::vector<Observation>& raw_obs, const std::vector<Observation>& processed_obs, int selected_id)
{
    size_t n = std::min(raw_obs.size(), processed_obs.size());
    for (size_t i = 0; i < n; i++) {
        if (processed_obs[i].id == selected_id) {
            return &raw_obs[i];
        }
    }
    return nullptr;
}

inline double compute_dt(std::mt19937& rng, const SimConfig& cfg, bool& stutter)
{
    double dt = 1.0 / cfg.fps;
    dt *= std::max(0.2, 1.0 + gauss(rng, cfg.dt_jitter_ratio));
    stutter = false;
    if (uniform01(rng) < cfg.stutter_prob) {
        dt *= cfg.stutter_mult;
        stutter = true;
    }
    return dt;
}

inline bool is_occluded_frame(std::mt19937& rng, const SimConfig& cfg, int& left_frames)
{
    if (left_frames > 0) {
        left_frames -= 1;
        return true;
    }
    if (uniform01(rng) < cfg.drop_prob) {
        left_frames = randint(rng, cfg.drop_min_frames, cfg.drop_max_frames) - 1;
        return true;
    }
    return false;
}

template <class SelectAt>
CommandResult solve_command(double t, double bullet_speed, double latency_s, double gimbal_yaw, SelectAt&& select_at)
{
    CommandResult result;
    Aim preview_hit = select_at(t);
    if (preview_hit.id < 0 || !finite_vec(preview_hit.aim_pos)) {
        return result;
    }

    Trajectory traj(preview_hit.aim_pos, bullet_speed, gimbal_yaw);
    double prev_fly_time = traj.solvable() ? traj.get_flyTime() : 0.0;
    double target_t = t + prev_fly_time + latency_s;

    Aim final_hit = select_at(target_t);
    if (final_hit.id < 0 || !finite_vec(final_hit.aim_pos)) {
        return result;
    }

    traj.set(final_hit.aim_pos, bullet_speed, gimbal_yaw);
    double final_fly_time = traj.solvable() ? traj.get_flyTime() : 0.0;

    if (std::abs(final_fly_time - prev_fly_time) > 0.001) {
        final_hit = select_at(t + final_fly_time + latency_s);
        if (final_hit.id < 0 || !finite_vec(final_hit.aim_pos)) {
            return result;
        }
        traj.set(final_hit.aim_pos, bullet_speed, gimbal_yaw);
        final_fly_time = traj.solvable() ? traj.get_flyTime() : 0.0;
    }

    result.hit = final_hit;
    result.fly_time = final_fly_time;
    result.solvable = traj.solvable();
    result.yaw_deg = angles::to_degrees(result.solvable ? traj.get_yaw() : command_yaw(final_hit.aim_pos));
    result.pitch_deg = angles::to_degrees(result.solvable ? traj.get_pitch() : command_pitch(final_hit.aim_pos));
    return result;
}

inline void update_current_metrics(
    Report& report, const CommandResult& predicted, const CommandResult& truth, double t,
    bool& has_prev, double& prev_t, double& prev_yaw, double& prev_pitch)
{
    if (predicted.hit.id < 0 || truth.hit.id < 0) {
        return;
    }
    if (!finite_vec(predicted.hit.aim_pos) || !finite_vec(predicted.hit.center_vel)) {
        report.add_issue("non-finite current output");
        return;
    }

    report.selected += 1;
    report.samples += 1;
    report.max_cmd_yaw_err_deg = std::max(
        report.max_cmd_yaw_err_deg,
        std::abs(angles::shortest_angular_distance(angles::from_degrees(truth.yaw_deg), angles::from_degrees(predicted.yaw_deg))) * 180.0 / pi());
    report.max_cmd_pitch_err_deg = std::max(report.max_cmd_pitch_err_deg, std::abs(predicted.pitch_deg - truth.pitch_deg));
    report.max_center_speed = std::max(report.max_center_speed, predicted.hit.center_vel.norm());

    if (has_prev) {
        double dt = std::max(t - prev_t, 1e-6);
        double yaw_step = std::abs(angles::shortest_angular_distance(angles::from_degrees(prev_yaw), angles::from_degrees(predicted.yaw_deg))) * 180.0 / pi();
        double pitch_step = std::abs(predicted.pitch_deg - prev_pitch);
        report.max_cmd_yaw_step_deg = std::max(report.max_cmd_yaw_step_deg, yaw_step);
        report.max_cmd_pitch_step_deg = std::max(report.max_cmd_pitch_step_deg, pitch_step);
        report.max_cmd_yaw_rate_deg_s = std::max(report.max_cmd_yaw_rate_deg_s, yaw_step / dt);
    }

    prev_t = t;
    prev_yaw = predicted.yaw_deg;
    prev_pitch = predicted.pitch_deg;
    has_prev = true;
}

inline void update_future_metrics(
    Report& report, const Aim& predicted, const Aim& truth, double t,
    bool& has_prev, double& prev_t, double& prev_yaw, double& prev_pitch)
{
    if (predicted.id < 0 || truth.id < 0) {
        return;
    }
    if (!finite_vec(predicted.aim_pos)) {
        report.add_issue("non-finite future output");
        return;
    }

    double yaw = angles::to_degrees(command_yaw(predicted.aim_pos));
    double pitch = angles::to_degrees(command_pitch(predicted.aim_pos));
    double truth_yaw = angles::to_degrees(command_yaw(truth.aim_pos));
    double truth_pitch = angles::to_degrees(command_pitch(truth.aim_pos));

    report.future_samples += 1;
    report.max_future_yaw_err_deg = std::max(
        report.max_future_yaw_err_deg,
        std::abs(angles::shortest_angular_distance(angles::from_degrees(truth_yaw), angles::from_degrees(yaw))) * 180.0 / pi());
    report.max_future_pitch_err_deg = std::max(report.max_future_pitch_err_deg, std::abs(truth_pitch - pitch));

    if (has_prev) {
        double dt = std::max(t - prev_t, 1e-6);
        double yaw_step = std::abs(angles::shortest_angular_distance(angles::from_degrees(prev_yaw), angles::from_degrees(yaw))) * 180.0 / pi();
        double pitch_step = std::abs(pitch - prev_pitch);
        report.max_future_yaw_step_deg = std::max(report.max_future_yaw_step_deg, yaw_step);
        report.max_future_pitch_step_deg = std::max(report.max_future_pitch_step_deg, pitch_step);
        report.max_future_yaw_rate_deg_s = std::max(report.max_future_yaw_rate_deg_s, yaw_step / dt);
    }

    prev_t = t;
    prev_yaw = yaw;
    prev_pitch = pitch;
    has_prev = true;
}

inline void write_csv(const std::string& path, const std::vector<CommandRow>& rows)
{
    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    std::ofstream out(path);
    out << "t,dt_ms,stutter,occluded,raw_obs,processed_obs,selected_id,top_level,raw_target_yaw_deg,target_yaw_deg,truth_target_yaw_deg,raw_target_pitch_deg,target_pitch_deg,truth_target_pitch_deg,center_speed,reported_vyaw_deg_s\n";
    for (const auto& row: rows) {
        auto write_value = [&](double value) {
            if (std::isfinite(value)) {
                out << std::fixed << std::setprecision(6) << value;
            }
        };
        write_value(row.t); out << ",";
        write_value(row.dt_ms); out << ",";
        out << (row.stutter ? 1 : 0) << ",";
        out << (row.occluded ? 1 : 0) << ",";
        out << row.raw_obs << ",";
        out << row.processed_obs << ",";
        out << row.selected_id << ",";
        out << row.top_level << ",";
        write_value(row.raw_target_yaw_deg); out << ",";
        write_value(row.target_yaw_deg); out << ",";
        write_value(row.truth_target_yaw_deg); out << ",";
        write_value(row.raw_target_pitch_deg); out << ",";
        write_value(row.target_pitch_deg); out << ",";
        write_value(row.truth_target_pitch_deg); out << ",";
        write_value(row.center_speed); out << ",";
        write_value(row.reported_vyaw_deg_s); out << "\n";
    }
}

inline std::string build_csv_path(const SimConfig& cfg, const std::string& tracker, const std::string& mode, double distance, double bullet_speed)
{
    std::filesystem::path dir(cfg.output_dir);
    dir /= safe_name(tracker);
    dir /= safe_name(cfg.preset);
    std::string stem = safe_name(tracker) + "__" + safe_name(mode);
    if (cfg.preset != "default") {
        stem += "__" + safe_name(cfg.preset);
    }
    stem += "__" + value_tag(distance, "m");
    stem += "__" + value_tag(bullet_speed, "mps");
    return (dir / (stem + ".csv")).string();
}

inline void print_report(const Report& report)
{
    std::cout << report.tracker << " [" << report.mode << "]\n";
    std::cout << "  safety: " << (report.pass() ? "PASS" : "FAIL") << "\n";
    std::cout << "  selected/current/future: " << report.selected << "/" << report.samples << "/" << report.future_samples << "\n";
    std::cout << "  occluded/stutter: " << report.occluded_frames << "/" << report.stutter_frames << "\n";
    std::cout << "  max current yaw step: " << std::fixed << std::setprecision(3) << report.max_cmd_yaw_step_deg << " deg\n";
    std::cout << "  max future yaw step: " << report.max_future_yaw_step_deg << " deg\n";
    std::cout << "  max current yaw rate: " << report.max_cmd_yaw_rate_deg_s << " deg/s\n";
    std::cout << "  max future yaw rate: " << report.max_future_yaw_rate_deg_s << " deg/s\n";
    std::cout << "  max current yaw err: " << report.max_cmd_yaw_err_deg << " deg\n";
    std::cout << "  max future yaw err: " << report.max_future_yaw_err_deg << " deg\n";
    std::cout << "  max center speed: " << report.max_center_speed << " m/s\n";
    std::cout << "  max |vyaw|: " << report.max_vyaw_deg_s << " deg/s\n";
    std::cout << "  csv: " << report.csv_path << "\n";
    if (!report.details.empty()) {
        std::cout << "  issues:\n";
        for (const auto& detail: report.details) {
            std::cout << "    - " << detail << "\n";
        }
    }
}

} // namespace helios_cv
