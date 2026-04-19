#pragma once

#include "armor_predictor/tracker/SimpleTracker.hpp"
#include "armor_predictor/tracker/SingerTracker.hpp"
#include "armor_predictor/tracker/Top3Tracker.hpp"
#include "armor_predictor/tracker/TopTracker.hpp"
#include "autoaim_utilities/BulletTrajectory.hpp"
#include "autoaim_utilities/PredictorTypes.hpp"

#include <angles/angles.h>
#include <opencv2/core.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
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
    bool no_plot = false;
    bool help = false;
    bool duration_overridden = false;
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
    std::string plot_path;
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
        // if (tracker == "SmallEnergyTracker" || tracker == "BigEnergyTracker") {
        //     return issues == 0
        //         && max_cmd_yaw_err_deg < 8.0
        //         && max_future_yaw_err_deg < 10.0
        //         && max_cmd_pitch_err_deg < 8.0
        //         && max_future_pitch_err_deg < 10.0
        //         && max_center_speed < 5.0;
        // }
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
    ArmorAim hit;
    double yaw_deg = nan_v();
    double pitch_deg = nan_v();
    double fly_time = 0.0;
    bool solvable = false;
};

/* Energy simulator helpers are commented out for now.
struct EnergyTruthFrame {
    EnergyObservation observation;
    std::vector<EnergyAim> aims;
};

struct EnergyCommandRow {
    double t = 0.0;
    double dt_ms = 0.0;
    bool stutter = false;
    bool occluded = false;
    int selected_fan_id = -1;
    double raw_target_yaw_deg = nan_v();
    double target_yaw_deg = nan_v();
    double truth_target_yaw_deg = nan_v();
    double raw_target_pitch_deg = nan_v();
    double target_pitch_deg = nan_v();
    double truth_target_pitch_deg = nan_v();
    int score = 0;
    int progress_group = 0;
    int lit_mask = 0;
    bool fire = false;
    bool hit_success = false;
    std::string state = "activating";
    std::string reset_reason = "none";
    double raw_roll = nan_v();
    double pred_roll = nan_v();
    double truth_roll = nan_v();
};

struct EnergyCommandResult {
    EnergyAim hit;
    double yaw_deg = nan_v();
    double pitch_deg = nan_v();
    double fly_time = 0.0;
    bool solvable = false;
    bool fire = false;
};
*/

inline double uniform01(std::mt19937& rng)
{
    return std::uniform_real_distribution<double>(0.0, 1.0)(rng);
}

inline double resolved_distance(const SimConfig& cfg, const std::string& tracker_family)
{
    if (cfg.distance_m > 0.0) return cfg.distance_m;
    // if (tracker_family == "energy") return 2.0;
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

inline double truth_pitch_deg(const ArmorAim& hit, double bullet_speed, double gimbal_yaw)
{
    if (hit.id < 0 || !finite_vec(hit.aim_pos)) return nan_v();
    Trajectory traj(hit.aim_pos, bullet_speed, gimbal_yaw);
    return angles::to_degrees(traj.solvable() ? traj.get_pitch() : command_pitch(hit.aim_pos));
}

inline CommandResult raw_observation_command(const ArmorObservation& obs, double bullet_speed, double gimbal_yaw)
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

/* Energy simulator helpers are commented out for now.
inline Eigen::Vector3d energy_fan_position(const Eigen::Vector3d& center, double yaw, double roll, double radius)
{
    return {
        center.x() + radius * std::sin(roll) * std::sin(yaw),
        center.y() - radius * std::sin(roll) * std::cos(yaw),
        center.z() + radius * std::cos(roll)};
}

inline EnergyCommandResult raw_energy_command(
    const EnergyObservation& obs, int selected_fan_id, double radius, double bullet_speed, double gimbal_yaw)
{
    EnergyCommandResult result;
    if (!obs.detected || selected_fan_id < 0 || selected_fan_id >= 5) {
        return result;
    }
    result.hit.id = selected_fan_id;
    result.hit.type = obs.fans[static_cast<size_t>(selected_fan_id)].type;
    result.hit.center_pos = obs.center_pos;
    result.hit.yaw = obs.yaw;
    result.hit.roll = obs.raw_roll + selected_fan_id * 2.0 * pi() / 5.0;
    result.hit.energy_radius = radius;
    result.hit.aim_pos = energy_fan_position(obs.center_pos, obs.yaw, result.hit.roll, radius);
    Trajectory traj(result.hit.aim_pos, bullet_speed, command_yaw(result.hit.aim_pos));
    result.fly_time = traj.solvable() ? traj.get_flyTime() : 0.0;
    result.solvable = traj.solvable();
    result.yaw_deg = angles::to_degrees(traj.solvable() ? traj.get_yaw() : command_yaw(result.hit.aim_pos));
    result.pitch_deg = angles::to_degrees(traj.solvable() ? traj.get_pitch() : -command_pitch(result.hit.aim_pos));
    return result;
}
*/

inline const ArmorObservation* find_selected_observation(
    const std::vector<ArmorObservation>& raw_obs, const std::vector<ArmorObservation>& processed_obs, int selected_id)
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
    ArmorAim preview_hit = select_at(t);
    if (preview_hit.id < 0 || !finite_vec(preview_hit.aim_pos)) {
        return result;
    }

    Trajectory traj(preview_hit.aim_pos, bullet_speed, gimbal_yaw);
    double prev_fly_time = traj.solvable() ? traj.get_flyTime() : 0.0;
    double target_t = t + prev_fly_time + latency_s;

    ArmorAim final_hit = select_at(target_t);
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

/* Energy simulator helpers are commented out for now.
template <class SelectAt>
EnergyCommandResult solve_energy_command(double t, double bullet_speed, double latency_s, double gimbal_yaw, SelectAt&& select_at)
{
    EnergyCommandResult result;
    EnergyAim preview_hit = select_at(t);
    if (preview_hit.id < 0 || !finite_vec(preview_hit.aim_pos)) {
        return result;
    }

    Trajectory traj(preview_hit.aim_pos, bullet_speed, command_yaw(preview_hit.aim_pos));
    double prev_fly_time = traj.solvable() ? traj.get_flyTime() : 0.0;
    double target_t = t + prev_fly_time + latency_s;

    EnergyAim final_hit = select_at(target_t);
    if (final_hit.id < 0 || !finite_vec(final_hit.aim_pos)) {
        return result;
    }

    traj.set(final_hit.aim_pos, bullet_speed, command_yaw(final_hit.aim_pos));
    double final_fly_time = traj.solvable() ? traj.get_flyTime() : 0.0;

    if (std::abs(final_fly_time - prev_fly_time) > 0.001) {
        final_hit = select_at(t + final_fly_time + latency_s);
        if (final_hit.id < 0 || !finite_vec(final_hit.aim_pos)) {
            return result;
        }
        traj.set(final_hit.aim_pos, bullet_speed, command_yaw(final_hit.aim_pos));
        final_fly_time = traj.solvable() ? traj.get_flyTime() : 0.0;
    }

    result.hit = final_hit;
    result.fly_time = final_fly_time;
    result.solvable = traj.solvable();
    result.fire = result.solvable && final_hit.trusted && final_hit.type == 0;
    result.yaw_deg = angles::to_degrees(result.solvable ? traj.get_yaw() : command_yaw(final_hit.aim_pos));
    result.pitch_deg = angles::to_degrees(result.solvable ? traj.get_pitch() : -command_pitch(final_hit.aim_pos));
    return result;
}

inline int energy_lit_mask(const std::array<int, 5>& fan_types)
{
    int mask = 0;
    for (int i = 0; i < 5; i++) {
        if (fan_types[static_cast<size_t>(i)] == 0) {
            mask |= 1 << i;
        }
    }
    return mask;
}
*/

inline int score_from_offset_m(double offset_m)
{
    if (!std::isfinite(offset_m) || offset_m < 0.0 || offset_m > 0.150) {
        return 0;
    }
    int bucket = static_cast<int>(std::ceil(offset_m / 0.015 - 1e-9));
    return std::clamp(11 - bucket, 1, 10);
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
    Report& report, const ArmorAim& predicted, const ArmorAim& truth, double t,
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

/* Energy simulator helpers are commented out for now.
inline void update_energy_current_metrics(
    Report& report, const EnergyCommandResult& predicted, const EnergyCommandResult& truth, double t,
    bool& has_prev, double& prev_t, double& prev_yaw, double& prev_pitch)
{
    if (predicted.hit.id < 0 || truth.hit.id < 0) {
        return;
    }
    if (!finite_vec(predicted.hit.aim_pos) || !finite_vec(predicted.hit.center_vel)) {
        report.add_issue("non-finite current energy output");
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

inline void update_energy_future_metrics(
    Report& report, const EnergyAim& predicted, const EnergyAim& truth, double t,
    bool& has_prev, double& prev_t, double& prev_yaw, double& prev_pitch)
{
    if (predicted.id < 0 || truth.id < 0) {
        return;
    }
    if (!finite_vec(predicted.aim_pos)) {
        report.add_issue("non-finite future energy output");
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
*/

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

inline std::string build_plot_path(const std::string& csv_path)
{
    std::filesystem::path path(csv_path);
    return (path.parent_path() / (path.stem().string() + "_target_yaw.svg")).string();
}

inline void write_svg_plot(const std::string& path, const std::vector<CommandRow>& rows)
{
    if (rows.empty()) {
        return;
    }

    // [1] 画布尺寸
    double cw = 6000.0;          // 画布总宽 (调大可以把波形横向拉长)
    double ch = 3000.0;          // 画布总高

    // [2] 线条与刻度配置
    int x_tick_count = 32;         // X轴时间刻度数量
    double line_width = 3.5;       // 基础曲线宽度
    double target_line_mult = 0.5; // 预测目标曲线(蓝色)的增粗倍数
    double truth_line_mult = 0.8;  // 真实标注曲线(绿色)的增粗倍数 
    double grid_width = 1.5;       // 背景网格线粗细
    double axis_line_width = 2.5;  // 0刻度基准线(虚线)粗细

    // [3] 字体缩放比例 (相对于单个面板的高度)
    double title_font_scale = 0.09;  // 面板左上角主标题大小
    double label_font_scale = 0.065; // Y轴文字大小
    double tick_font_scale = 0.055;  // 坐标轴数字刻度大小

    // [4] 配色方案 (Tech & Science Light)
    std::string bg_color = "#F0F4F8";       // 画布最底层背景 
    std::string panel_bg = "#FFFFFF";       // 每个数据面板的背景色
    std::string text_main = "#1E293B";      // 主标题字体颜色 
    std::string text_sub = "#64748B";       // 坐标轴/副标题字体颜色 
    std::string grid_color = "#E2E8F0";     // 普通网格线颜色 
    std::string grid_dark = "#94A3B8";      // 0基准网格线颜色 
    
    std::string c_target = "#028fc7";       // 预测目标线 (Sky Blue)
    std::string c_truth = "#191882";        // 真实标注线 (Emerald)
    std::string c_raw = "#8B5CF6";          // 原始数据线 (Violet)
    std::string c_err = "#E11D48";          // Yaw 误差线 (Rose)
    std::string c_pitch_err = "#0D9488";    // Pitch 误差线 (Teal)
    std::string c_dt = "#F59E0B";           // Dt指标线 (Amber)
    
    std::string c_flag_occluded = "#EF4444"; // 遮挡事件色带 (纯红)
    std::string c_flag_stutter = "#F59E0B";  // 卡顿事件色带 (纯橙)
    

    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    std::ofstream out(path);
    if (!out.is_open()) {
        return;
    }

    std::vector<double> t, dt_ms, raw_yaw, pred_yaw, truth_yaw, pred_pitch, truth_pitch, yaw_err, pitch_err;
    t.reserve(rows.size()); dt_ms.reserve(rows.size()); raw_yaw.reserve(rows.size());
    pred_yaw.reserve(rows.size()); truth_yaw.reserve(rows.size()); pred_pitch.reserve(rows.size());
    truth_pitch.reserve(rows.size()); yaw_err.reserve(rows.size()); pitch_err.reserve(rows.size());

    for (const auto& row: rows) {
        t.push_back(row.t);
        dt_ms.push_back(row.dt_ms);
        raw_yaw.push_back(row.raw_target_yaw_deg);
        pred_yaw.push_back(row.target_yaw_deg);
        truth_yaw.push_back(row.truth_target_yaw_deg);
        pred_pitch.push_back(row.target_pitch_deg);
        truth_pitch.push_back(row.truth_target_pitch_deg);
        yaw_err.push_back(
            std::isfinite(row.target_yaw_deg) && std::isfinite(row.truth_target_yaw_deg)
                ? angles::to_degrees(angles::shortest_angular_distance(
                    angles::from_degrees(row.truth_target_yaw_deg), angles::from_degrees(row.target_yaw_deg)))
                : nan_v());
        pitch_err.push_back(
            std::isfinite(row.target_pitch_deg) && std::isfinite(row.truth_target_pitch_deg)
                ? row.target_pitch_deg - row.truth_target_pitch_deg
                : nan_v());
    }

    auto range_of = [&](std::initializer_list<const std::vector<double>*> series, double fallback_min, double fallback_max) {
        double lo = std::numeric_limits<double>::infinity();
        double hi = -std::numeric_limits<double>::infinity();
        for (const auto* values: series) {
            for (double value: *values) {
                if (!std::isfinite(value)) { continue; }
                lo = std::min(lo, value);
                hi = std::max(hi, value);
            }
        }
        if (!std::isfinite(lo) || !std::isfinite(hi)) { lo = fallback_min; hi = fallback_max; }
        if (std::abs(hi - lo) < 1e-6) { lo -= 1.0; hi += 1.0; } 
        else { double pad = (hi - lo) * 0.1; lo -= pad; hi += pad; } 
        return std::pair<double, double> {lo, hi};
    };

    double t_min = t.front();
    double t_max = t.back();
    if (!(t_max > t_min)) t_max = t_min + 1.0;

    auto yaw_combined_range = range_of({&pred_yaw, &truth_yaw, &raw_yaw}, -1.0, 1.0);
    auto pitch_combined_range = range_of({&pred_pitch, &truth_pitch}, -1.0, 1.0);
    auto err_combined_range = range_of({&yaw_err, &pitch_err}, -1.0, 1.0);
    auto dt_range = range_of({&dt_ms}, 0.0, 1.0);

    double px = cw * 0.025; 
    double py = ch * 0.03;  
    double gap_y = ch * 0.015; 
    int num_panels = 4;
    
    double panel_w = cw - 2 * px;
    double panel_h = (ch - 2 * py - (num_panels - 1) * gap_y) / num_panels;
    
    double g_px = panel_w * 0.06; 
    double graph_x = px + g_px;
    double graph_w = panel_w - 2 * g_px;
    double graph_y_offset = panel_h * 0.32; 
    double graph_h = panel_h * 0.58;        

    double f_title = panel_h * title_font_scale;
    double f_label = panel_h * label_font_scale;
    double f_tick  = panel_h * tick_font_scale;

    out << "<svg xmlns='http://www.w3.org/2000/svg' width='" << cw << "' height='" << ch << "' viewBox='0 0 " << cw << " " << ch << "'>\n";
    out << "<defs>\n";
    out << "<filter id='shadow' x='-2%' y='-5%' width='104%' height='115%'>\n";
    out << "<feDropShadow dx='0' dy='" << ch*0.003 << "' stdDeviation='" << ch*0.004 << "' flood-color='#0F172A' flood-opacity='0.04'/>\n";
    out << "</filter>\n";
    out << "</defs>\n";
    out << "<rect width='100%' height='100%' fill='" << bg_color << "'/>\n";

    auto x_of = [&](double value) {
        return graph_x + (value - t_min) * graph_w / (t_max - t_min);
    };
    auto y_of = [&](double value, double lo, double hi, double p_top) {
        return p_top + graph_y_offset + (hi - value) * graph_h / (hi - lo);
    };

    auto draw_panel = [&](double p_top, const std::string& title, const std::string& ylabel, double lo, double hi) {
        out << "<rect x='" << px << "' y='" << p_top << "' width='" << panel_w << "' height='" << panel_h 
            << "' rx='" << panel_h*0.1 << "' fill='" << panel_bg << "' filter='url(#shadow)'/>\n";
        
        out << "<rect x='" << px + panel_w*0.02 << "' y='" << p_top + panel_h*0.1 << "' width='" << panel_w*0.004 
            << "' height='" << f_title*1.2 << "' rx='" << panel_w*0.002 << "' fill='" << c_target << "'/>\n";
        out << "<text x='" << px + panel_w*0.03 << "' y='" << p_top + panel_h*0.1 + f_title*0.9 
            << "' font-size='" << f_title << "' font-weight='700' font-family='system-ui, -apple-system, sans-serif' fill='" << text_main << "'>" << title << "</text>\n";
        
        double y_label_x = px + panel_w*0.015;
        double y_label_y = p_top + graph_y_offset + graph_h/2.0;
        out << "<text x='" << y_label_x << "' y='" << y_label_y << "' font-size='" << f_label 
            << "' font-family='system-ui, -apple-system, sans-serif' fill='" << text_sub << "' text-anchor='middle' transform='rotate(-90 " << y_label_x << " " << y_label_y << ")'>" << ylabel << "</text>\n";

        for (int i = 0; i <= x_tick_count; i++) {
            double x = graph_x + graph_w * i / (double)x_tick_count;
            out << "<line x1='" << x << "' y1='" << p_top + graph_y_offset << "' x2='" << x << "' y2='" << p_top + graph_y_offset + graph_h
                << "' stroke='" << grid_color << "' stroke-width='" << grid_width << "'/>\n";
                
            double tick = t_min + (t_max - t_min) * i / (double)x_tick_count;
            out << "<text x='" << x << "' y='" << p_top + graph_y_offset + graph_h + f_tick*1.5 
                << "' text-anchor='middle' font-size='" << f_tick << "' font-family='system-ui, -apple-system, sans-serif' fill='" << text_sub << "'>"
                << std::fixed << std::setprecision(1) << tick << "</text>\n";
        }
        
        for (int i = 0; i <= 4; i++) {
            double y = p_top + graph_y_offset + graph_h * i / 4.0;
            double tick = hi - (hi - lo) * i / 4.0;
            out << "<line x1='" << graph_x << "' y1='" << y << "' x2='" << graph_x + graph_w << "' y2='" << y 
                << "' stroke='" << grid_color << "' stroke-width='" << grid_width << "'/>\n";
            out << "<text x='" << graph_x - panel_w*0.01 << "' y='" << y + f_tick*0.35 
                << "' text-anchor='end' font-size='" << f_tick << "' font-family='system-ui, -apple-system, sans-serif' fill='" << text_sub << "'>"
                << std::fixed << std::setprecision(1) << tick << "</text>\n";
        }
    };

    auto draw_zero_line = [&](double p_top, double lo, double hi) {
        if (lo >= 0.0 || hi <= 0.0) return;
        double y0 = y_of(0.0, lo, hi, p_top);
        out << "<line x1='" << graph_x << "' y1='" << y0 << "' x2='" << graph_x + graph_w << "' y2='" << y0 
            << "' stroke='" << grid_dark << "' stroke-width='" << axis_line_width << "' stroke-dasharray='8 8'/>\n";
    };

    auto draw_series = [&](double p_top, double lo, double hi, const std::vector<double>& values, const std::string& color, double width_mult, double opacity) {
        std::ostringstream path_data;
        path_data << std::fixed << std::setprecision(2);
        bool move = true;
        for (size_t i = 0; i < values.size(); i++) {
            if (!std::isfinite(values[i])) { move = true; continue; }
            path_data << (move ? "M " : " L ") << x_of(t[i]) << " " << y_of(values[i], lo, hi, p_top);
            move = false;
        }
        if (path_data.str().empty()) return;
        out << "<path d='" << path_data.str() << "' fill='none' stroke='" << color << "' stroke-width='" << line_width * width_mult 
            << "' stroke-opacity='" << opacity << "' stroke-linejoin='round' stroke-linecap='round'/>\n";
    };

    auto draw_flag_bands = [&](double p_top, bool stutter, const std::string& color, double opacity) {
        double y_start = p_top + graph_y_offset;
        for (size_t i = 0; i < rows.size(); ) {
            bool active = stutter ? rows[i].stutter : rows[i].occluded;
            if (!active) { i++; continue; }
            size_t begin = i;
            while (i + 1 < rows.size() && (stutter ? rows[i+1].stutter : rows[i+1].occluded)) i++;
            double left = std::max(graph_x, x_of(t[begin]) - cw*0.001);
            double right = std::min(graph_x + graph_w, x_of(t[i]) + cw*0.001);
            out << "<rect x='" << left << "' y='" << y_start << "' width='" << std::max(cw*0.002, right - left) 
                << "' height='" << graph_h << "' fill='" << color << "' fill-opacity='" << opacity << "'/>\n";
            i++;
        }
    };

    auto draw_legend = [&](double p_top, std::initializer_list<std::pair<std::string, std::string>> items) {
        double total_w = 0.0;
        double legend_h = f_tick * 2.0;
        double padding = f_tick * 0.8;
        for (const auto& item : items) total_w += (item.first.size() * f_tick * 0.6) + legend_h * 1.5;
        
        double x = graph_x + graph_w - total_w;
        double y = p_top + panel_h*0.1;
        
        for (const auto& item : items) {
            double w = (item.first.size() * f_tick * 0.6) + legend_h * 1.5;
            out << "<rect x='" << x << "' y='" << y << "' width='" << w << "' height='" << legend_h 
                << "' rx='" << legend_h*0.5 << "' fill='" << grid_color << "'/>\n";
            out << "<circle cx='" << x + legend_h*0.6 << "' cy='" << y + legend_h*0.5 << "' r='" << legend_h*0.25 << "' fill='" << item.second << "'/>\n";
            out << "<text x='" << x + legend_h*1.1 << "' y='" << y + legend_h*0.7 
                << "' font-size='" << f_tick*1.1 << "' font-family='system-ui, -apple-system, sans-serif' fill='" << text_main << "'>" << item.first << "</text>\n";
            x += w + padding;
        }
    };

    for (int i = 0; i < num_panels; i++) {
        double current_top = py + i * (panel_h + gap_y);
        
        if (i == 0) {
            // 面板 1: Yaw 综合跟踪
            draw_panel(current_top, "Yaw Tracking", "yaw (deg)", yaw_combined_range.first, yaw_combined_range.second);
            draw_series(current_top, yaw_combined_range.first, yaw_combined_range.second, raw_yaw, c_raw, 0.8, 0.6); 
            draw_series(current_top, yaw_combined_range.first, yaw_combined_range.second, truth_yaw, c_truth, truth_line_mult, 0.9);
            draw_series(current_top, yaw_combined_range.first, yaw_combined_range.second, pred_yaw, c_target, target_line_mult, 1.0);
            draw_legend(current_top, {{"Target", c_target}, {"Truth", c_truth}, {"Raw", c_raw}});
            
        } else if (i == 1) {
            // 面板 2: Pitch 综合跟踪
            draw_panel(current_top, "Pitch Tracking", "pitch (deg)", pitch_combined_range.first, pitch_combined_range.second);
            draw_series(current_top, pitch_combined_range.first, pitch_combined_range.second, truth_pitch, c_truth, truth_line_mult, 0.9);
            draw_series(current_top, pitch_combined_range.first, pitch_combined_range.second, pred_pitch, c_target, target_line_mult, 1.0);
            draw_legend(current_top, {{"Target", c_target}, {"Truth", c_truth}});
            
        } else if (i == 2) {
            // 面板 3: 角度综合误差
            draw_panel(current_top, "Tracking Errors", "err (deg)", err_combined_range.first, err_combined_range.second);
            draw_zero_line(current_top, err_combined_range.first, err_combined_range.second);
            draw_series(current_top, err_combined_range.first, err_combined_range.second, yaw_err, c_err, 1.0, 0.85);
            draw_series(current_top, err_combined_range.first, err_combined_range.second, pitch_err, c_pitch_err, 1.0, 0.85);
            draw_legend(current_top, {{"Yaw Err", c_err}, {"Pitch Err", c_pitch_err}});
            
        } else if (i == 3) {
            // 面板 4: 系统性能与事件
            draw_panel(current_top, "Frame Dt + Events", "dt (ms)", dt_range.first, dt_range.second);
            draw_flag_bands(current_top, false, c_flag_occluded, 0.15); 
            draw_flag_bands(current_top, true, c_flag_stutter, 0.15);  
            draw_series(current_top, dt_range.first, dt_range.second, dt_ms, c_dt, 1.0, 0.95);
            draw_legend(current_top, {{"Dt", c_dt}, {"Occluded", c_flag_occluded}, {"Stutter", c_flag_stutter}});
            
            // 底部时间轴大标题
            out << "<text x='" << cw/2.0 << "' y='" << current_top + panel_h + f_tick*3.0 
                << "' font-size='" << f_label << "' font-family='system-ui, -apple-system, sans-serif' font-weight='600' fill='" << text_sub 
                << "' text-anchor='middle'>TIME (SECONDS)</text>\n";
        }
    }

    out << "</svg>\n";
}

/* Energy simulator helpers are commented out for now.
inline void write_energy_csv(const std::string& path, const std::vector<EnergyCommandRow>& rows)
{
    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    std::ofstream out(path);
    out << "t,dt_ms,stutter,occluded,selected_fan_id,raw_target_yaw_deg,target_yaw_deg,truth_target_yaw_deg,raw_target_pitch_deg,target_pitch_deg,truth_target_pitch_deg,score,state,progress_group,lit_mask,fire,hit_success,reset_reason,raw_roll,pred_roll,truth_roll\n";
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
        out << row.selected_fan_id << ",";
        write_value(row.raw_target_yaw_deg); out << ",";
        write_value(row.target_yaw_deg); out << ",";
        write_value(row.truth_target_yaw_deg); out << ",";
        write_value(row.raw_target_pitch_deg); out << ",";
        write_value(row.target_pitch_deg); out << ",";
        write_value(row.truth_target_pitch_deg); out << ",";
        out << row.score << ",";
        out << row.state << ",";
        out << row.progress_group << ",";
        out << row.lit_mask << ",";
        out << (row.fire ? 1 : 0) << ",";
        out << (row.hit_success ? 1 : 0) << ",";
        out << row.reset_reason << ",";
        write_value(row.raw_roll); out << ",";
        write_value(row.pred_roll); out << ",";
        write_value(row.truth_roll); out << "\n";
    }
}

inline std::string build_energy_plot_path(const std::string& csv_path)
{
    std::filesystem::path path(csv_path);
    return (path.parent_path() / (path.stem().string() + "_energy.svg")).string();
}

inline void write_energy_svg_plot(const std::string& path, const std::vector<EnergyCommandRow>& rows, bool big)
{
    if (rows.empty()) {
        return;
    }

    double cw = 6000.0;
    double ch = 3600.0;
    int x_tick_count = 32;
    double line_width = 3.5;
    double target_line_mult = 0.5;
    double truth_line_mult = 0.8;
    double grid_width = 1.5;
    double axis_line_width = 2.5;
    double title_font_scale = 0.09;
    double label_font_scale = 0.065;
    double tick_font_scale = 0.055;

    std::string bg_color = "#F0F4F8";
    std::string panel_bg = "#FFFFFF";
    std::string text_main = "#1E293B";
    std::string text_sub = "#64748B";
    std::string grid_color = "#E2E8F0";
    std::string grid_dark = "#94A3B8";

    std::string c_target = "#028fc7";
    std::string c_truth = "#191882";
    std::string c_raw = "#8B5CF6";
    std::string c_err = "#E11D48";
    std::string c_pitch_err = "#0D9488";
    std::string c_dt = "#F59E0B";
    std::string c_score = "#DC2626";
    std::string c_flag_occluded = "#EF4444";
    std::string c_flag_stutter = "#F59E0B";

    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    std::ofstream out(path);
    if (!out.is_open()) {
        return;
    }

    std::vector<double> t, dt_ms, raw_yaw, pred_yaw, truth_yaw, pred_pitch, truth_pitch, yaw_err, pitch_err, score;
    t.reserve(rows.size());
    dt_ms.reserve(rows.size());
    raw_yaw.reserve(rows.size());
    pred_yaw.reserve(rows.size());
    truth_yaw.reserve(rows.size());
    pred_pitch.reserve(rows.size());
    truth_pitch.reserve(rows.size());
    yaw_err.reserve(rows.size());
    pitch_err.reserve(rows.size());
    score.reserve(rows.size());

    for (const auto& row: rows) {
        t.push_back(row.t);
        dt_ms.push_back(row.dt_ms);
        raw_yaw.push_back(row.raw_target_yaw_deg);
        pred_yaw.push_back(row.target_yaw_deg);
        truth_yaw.push_back(row.truth_target_yaw_deg);
        pred_pitch.push_back(row.target_pitch_deg);
        truth_pitch.push_back(row.truth_target_pitch_deg);
        yaw_err.push_back(
            std::isfinite(row.target_yaw_deg) && std::isfinite(row.truth_target_yaw_deg)
                ? angles::to_degrees(angles::shortest_angular_distance(
                    angles::from_degrees(row.truth_target_yaw_deg), angles::from_degrees(row.target_yaw_deg)))
                : nan_v());
        pitch_err.push_back(
            std::isfinite(row.target_pitch_deg) && std::isfinite(row.truth_target_pitch_deg)
                ? row.target_pitch_deg - row.truth_target_pitch_deg
                : nan_v());
        score.push_back(row.score);
    }

    auto range_of = [&](std::initializer_list<const std::vector<double>*> series, double fallback_min, double fallback_max) {
        double lo = std::numeric_limits<double>::infinity();
        double hi = -std::numeric_limits<double>::infinity();
        for (const auto* values: series) {
            for (double value: *values) {
                if (!std::isfinite(value)) {
                    continue;
                }
                lo = std::min(lo, value);
                hi = std::max(hi, value);
            }
        }
        if (!std::isfinite(lo) || !std::isfinite(hi)) {
            lo = fallback_min;
            hi = fallback_max;
        }
        if (std::abs(hi - lo) < 1e-6) {
            lo -= 1.0;
            hi += 1.0;
        } else {
            double pad = (hi - lo) * 0.1;
            lo -= pad;
            hi += pad;
        }
        return std::pair<double, double> {lo, hi};
    };

    double t_min = t.front();
    double t_max = t.back();
    if (!(t_max > t_min)) {
        t_max = t_min + 1.0;
    }

    auto yaw_range = range_of({&pred_yaw, &truth_yaw, &raw_yaw}, -1.0, 1.0);
    auto pitch_range = range_of({&pred_pitch, &truth_pitch}, -1.0, 1.0);
    auto err_range = range_of({&yaw_err, &pitch_err}, -1.0, 1.0);
    auto dt_range = range_of({&dt_ms}, 0.0, 1.0);
    double score_hi = 50.5;

    double px = cw * 0.025;
    double py = ch * 0.03;
    double gap_y = ch * 0.015;
    int num_panels = 5;
    double panel_w = cw - 2 * px;
    double panel_h = (ch - 2 * py - (num_panels - 1) * gap_y) / num_panels;
    double g_px = panel_w * 0.06;
    double graph_x = px + g_px;
    double graph_w = panel_w - 2 * g_px;
    double graph_y_offset = panel_h * 0.32;
    double graph_h = panel_h * 0.58;
    double f_title = panel_h * title_font_scale;
    double f_label = panel_h * label_font_scale;
    double f_tick = panel_h * tick_font_scale;

    out << "<svg xmlns='http://www.w3.org/2000/svg' width='" << cw << "' height='" << ch << "' viewBox='0 0 " << cw << " " << ch << "'>\n";
    out << "<defs>\n";
    out << "<filter id='shadow' x='-2%' y='-5%' width='104%' height='115%'>\n";
    out << "<feDropShadow dx='0' dy='" << ch * 0.003 << "' stdDeviation='" << ch * 0.004 << "' flood-color='#0F172A' flood-opacity='0.04'/>\n";
    out << "</filter>\n";
    out << "</defs>\n";
    out << "<rect width='100%' height='100%' fill='" << bg_color << "'/>\n";

    auto x_of = [&](double value) {
        return graph_x + (value - t_min) * graph_w / (t_max - t_min);
    };
    auto y_of = [&](double value, double lo, double hi, double p_top) {
        return p_top + graph_y_offset + (hi - value) * graph_h / (hi - lo);
    };

    auto draw_panel = [&](double p_top, const std::string& title, const std::string& ylabel, double lo, double hi) {
        out << "<rect x='" << px << "' y='" << p_top << "' width='" << panel_w << "' height='" << panel_h
            << "' rx='" << panel_h * 0.1 << "' fill='" << panel_bg << "' filter='url(#shadow)'/>\n";
        out << "<rect x='" << px + panel_w * 0.02 << "' y='" << p_top + panel_h * 0.1 << "' width='" << panel_w * 0.004
            << "' height='" << f_title * 1.2 << "' rx='" << panel_w * 0.002 << "' fill='" << c_target << "'/>\n";
        out << "<text x='" << px + panel_w * 0.03 << "' y='" << p_top + panel_h * 0.1 + f_title * 0.9
            << "' font-size='" << f_title << "' font-weight='700' font-family='system-ui, -apple-system, sans-serif' fill='" << text_main << "'>" << title << "</text>\n";
        double y_label_x = px + panel_w * 0.015;
        double y_label_y = p_top + graph_y_offset + graph_h / 2.0;
        out << "<text x='" << y_label_x << "' y='" << y_label_y << "' font-size='" << f_label
            << "' font-family='system-ui, -apple-system, sans-serif' fill='" << text_sub << "' text-anchor='middle' transform='rotate(-90 " << y_label_x << " " << y_label_y << ")'>" << ylabel << "</text>\n";

        for (int i = 0; i <= x_tick_count; i++) {
            double x = graph_x + graph_w * i / static_cast<double>(x_tick_count);
            out << "<line x1='" << x << "' y1='" << p_top + graph_y_offset << "' x2='" << x << "' y2='" << p_top + graph_y_offset + graph_h
                << "' stroke='" << grid_color << "' stroke-width='" << grid_width << "'/>\n";
            double tick = t_min + (t_max - t_min) * i / static_cast<double>(x_tick_count);
            out << "<text x='" << x << "' y='" << p_top + graph_y_offset + graph_h + f_tick * 1.5
                << "' text-anchor='middle' font-size='" << f_tick << "' font-family='system-ui, -apple-system, sans-serif' fill='" << text_sub << "'>"
                << std::fixed << std::setprecision(1) << tick << "</text>\n";
        }

        for (int i = 0; i <= 4; i++) {
            double y = p_top + graph_y_offset + graph_h * i / 4.0;
            double tick = hi - (hi - lo) * i / 4.0;
            out << "<line x1='" << graph_x << "' y1='" << y << "' x2='" << graph_x + graph_w << "' y2='" << y
                << "' stroke='" << grid_color << "' stroke-width='" << grid_width << "'/>\n";
            out << "<text x='" << graph_x - panel_w * 0.01 << "' y='" << y + f_tick * 0.35
                << "' text-anchor='end' font-size='" << f_tick << "' font-family='system-ui, -apple-system, sans-serif' fill='" << text_sub << "'>"
                << std::fixed << std::setprecision(1) << tick << "</text>\n";
        }
    };

    auto draw_zero_line = [&](double p_top, double lo, double hi) {
        if (lo >= 0.0 || hi <= 0.0) {
            return;
        }
        double y0 = y_of(0.0, lo, hi, p_top);
        out << "<line x1='" << graph_x << "' y1='" << y0 << "' x2='" << graph_x + graph_w << "' y2='" << y0
            << "' stroke='" << grid_dark << "' stroke-width='" << axis_line_width << "' stroke-dasharray='8 8'/>\n";
    };

    auto draw_series = [&](double p_top, double lo, double hi, const std::vector<double>& values, const std::string& color, double width_mult, double opacity) {
        std::ostringstream path_data;
        path_data << std::fixed << std::setprecision(2);
        bool move = true;
        for (size_t i = 0; i < values.size(); i++) {
            if (!std::isfinite(values[i])) {
                move = true;
                continue;
            }
            path_data << (move ? "M " : " L ") << x_of(t[i]) << " " << y_of(values[i], lo, hi, p_top);
            move = false;
        }
        if (path_data.str().empty()) {
            return;
        }
        out << "<path d='" << path_data.str() << "' fill='none' stroke='" << color << "' stroke-width='" << line_width * width_mult
            << "' stroke-opacity='" << opacity << "' stroke-linejoin='round' stroke-linecap='round'/>\n";
    };

    auto draw_flag_bands = [&](double p_top, bool stutter, const std::string& color, double opacity) {
        double y_start = p_top + graph_y_offset;
        for (size_t i = 0; i < rows.size(); ) {
            bool active = stutter ? rows[i].stutter : rows[i].occluded;
            if (!active) {
                i++;
                continue;
            }
            size_t begin = i;
            while (i + 1 < rows.size() && (stutter ? rows[i + 1].stutter : rows[i + 1].occluded)) {
                i++;
            }
            double left = std::max(graph_x, x_of(t[begin]) - cw * 0.001);
            double right = std::min(graph_x + graph_w, x_of(t[i]) + cw * 0.001);
            out << "<rect x='" << left << "' y='" << y_start << "' width='" << std::max(cw * 0.002, right - left)
                << "' height='" << graph_h << "' fill='" << color << "' fill-opacity='" << opacity << "'/>\n";
            i++;
        }
    };

    auto draw_legend = [&](double p_top, std::initializer_list<std::pair<std::string, std::string>> items) {
        double total_w = 0.0;
        double legend_h = f_tick * 2.0;
        double padding = f_tick * 0.8;
        for (const auto& item: items) {
            total_w += (item.first.size() * f_tick * 0.6) + legend_h * 1.5;
        }
        double x = graph_x + graph_w - total_w;
        double y = p_top + panel_h * 0.1;
        for (const auto& item: items) {
            double w = (item.first.size() * f_tick * 0.6) + legend_h * 1.5;
            out << "<rect x='" << x << "' y='" << y << "' width='" << w << "' height='" << legend_h
                << "' rx='" << legend_h * 0.5 << "' fill='" << grid_color << "'/>\n";
            out << "<circle cx='" << x + legend_h * 0.6 << "' cy='" << y + legend_h * 0.5 << "' r='" << legend_h * 0.25 << "' fill='" << item.second << "'/>\n";
            out << "<text x='" << x + legend_h * 1.1 << "' y='" << y + legend_h * 0.7
                << "' font-size='" << f_tick * 1.1 << "' font-family='system-ui, -apple-system, sans-serif' fill='" << text_main << "'>" << item.first << "</text>\n";
            x += w + padding;
        }
    };

    for (int i = 0; i < num_panels; i++) {
        double current_top = py + i * (panel_h + gap_y);
        if (i == 0) {
            draw_panel(current_top, "Yaw Tracking", "yaw (deg)", yaw_range.first, yaw_range.second);
            draw_series(current_top, yaw_range.first, yaw_range.second, raw_yaw, c_raw, 0.8, 0.6);
            draw_series(current_top, yaw_range.first, yaw_range.second, truth_yaw, c_truth, truth_line_mult, 0.9);
            draw_series(current_top, yaw_range.first, yaw_range.second, pred_yaw, c_target, target_line_mult, 1.0);
            draw_legend(current_top, {{"Target", c_target}, {"Truth", c_truth}, {"Raw", c_raw}});
        } else if (i == 1) {
            draw_panel(current_top, "Pitch Tracking", "pitch (deg)", pitch_range.first, pitch_range.second);
            draw_series(current_top, pitch_range.first, pitch_range.second, truth_pitch, c_truth, truth_line_mult, 0.9);
            draw_series(current_top, pitch_range.first, pitch_range.second, pred_pitch, c_target, target_line_mult, 1.0);
            draw_legend(current_top, {{"Target", c_target}, {"Truth", c_truth}});
        } else if (i == 2) {
            draw_panel(current_top, "Tracking Errors", "err (deg)", err_range.first, err_range.second);
            draw_zero_line(current_top, err_range.first, err_range.second);
            draw_series(current_top, err_range.first, err_range.second, yaw_err, c_err, 1.0, 0.85);
            draw_series(current_top, err_range.first, err_range.second, pitch_err, c_pitch_err, 1.0, 0.85);
            draw_legend(current_top, {{"Yaw Err", c_err}, {"Pitch Err", c_pitch_err}});
        } else if (i == 3) {
            draw_panel(current_top, "Score", "score", -0.5, score_hi);
            draw_series(current_top, -0.5, score_hi, score, c_score, 1.0, 0.95);
            draw_legend(current_top, {{"Score", c_score}});
        } else {
            draw_panel(current_top, "System Behavior", "dt (ms)", dt_range.first, dt_range.second);
            draw_flag_bands(current_top, false, c_flag_occluded, 0.15);
            draw_flag_bands(current_top, true, c_flag_stutter, 0.15);
            draw_series(current_top, dt_range.first, dt_range.second, dt_ms, c_dt, 1.0, 0.95);
            draw_legend(current_top, {{"Dt", c_dt}, {"Occluded", c_flag_occluded}, {"Stutter", c_flag_stutter}});
            out << "<text x='" << cw / 2.0 << "' y='" << current_top + panel_h + f_tick * 3.0
                << "' font-size='" << f_label << "' font-family='system-ui, -apple-system, sans-serif' font-weight='600' fill='" << text_sub
                << "' text-anchor='middle'>TIME (SECONDS)</text>\n";
        }
    }

    out << "</svg>\n";
}
*/

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
    std::cout << "  max current pitch err: " << report.max_cmd_pitch_err_deg << " deg\n";
    std::cout << "  max future pitch err: " << report.max_future_pitch_err_deg << " deg\n";
    std::cout << "  max center speed: " << report.max_center_speed << " m/s\n";
    std::cout << "  max |vyaw|: " << report.max_vyaw_deg_s << " deg/s\n";
    std::cout << "  csv: " << report.csv_path << "\n";
    if (!report.plot_path.empty()) {
        std::cout << "  plot: " << report.plot_path << "\n";
    }
    if (!report.details.empty()) {
        std::cout << "  issues:\n";
        for (const auto& detail: report.details) {
            std::cout << "    - " << detail << "\n";
        }
    }
}

} // namespace helios_cv
