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
    bool no_plot = false;
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

    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    std::ofstream out(path);
    if (!out.is_open()) {
        return;
    }

    std::vector<double> t;
    std::vector<double> dt_ms;
    std::vector<double> raw_yaw;
    std::vector<double> pred_yaw;
    std::vector<double> truth_yaw;
    std::vector<double> pred_pitch;
    std::vector<double> truth_pitch;
    std::vector<double> yaw_err;
    std::vector<double> pitch_err;
    t.reserve(rows.size());
    dt_ms.reserve(rows.size());
    raw_yaw.reserve(rows.size());
    pred_yaw.reserve(rows.size());
    truth_yaw.reserve(rows.size());
    pred_pitch.reserve(rows.size());
    truth_pitch.reserve(rows.size());
    yaw_err.reserve(rows.size());
    pitch_err.reserve(rows.size());

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
            double pad = (hi - lo) * 0.08;
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

    auto yaw_range = range_of({&pred_yaw, &truth_yaw}, -1.0, 1.0);
    auto raw_range = range_of({&raw_yaw}, -1.0, 1.0);
    auto yaw_err_range = range_of({&yaw_err}, -1.0, 1.0);
    auto dt_range = range_of({&dt_ms}, 0.0, 1.0);
    auto pitch_range = range_of({&pred_pitch, &truth_pitch}, -1.0, 1.0);
    auto pitch_err_range = range_of({&pitch_err}, -1.0, 1.0);

    out << "<svg xmlns='http://www.w3.org/2000/svg' width='1280' height='1572' viewBox='0 0 1280 1572'>\n";
    out << "<defs>\n";
    out << "<linearGradient id='bg' x1='0%' y1='0%' x2='100%' y2='100%'>\n";
    out << "<stop offset='0%' stop-color='#f7f3ec'/>\n";
    out << "<stop offset='56%' stop-color='#fffaf4'/>\n";
    out << "<stop offset='100%' stop-color='#f4efe8'/>\n";
    out << "</linearGradient>\n";
    out << "<linearGradient id='panel' x1='0%' y1='0%' x2='0%' y2='100%'>\n";
    out << "<stop offset='0%' stop-color='#fffefd'/>\n";
    out << "<stop offset='100%' stop-color='#fbf6f0'/>\n";
    out << "</linearGradient>\n";
    out << "<filter id='shadow' x='-10%' y='-10%' width='120%' height='120%'>\n";
    out << "<feDropShadow dx='0' dy='10' stdDeviation='14' flood-color='#6b5f56' flood-opacity='0.10'/>\n";
    out << "</filter>\n";
    out << "</defs>\n";
    out << "<rect width='1280' height='1572' fill='url(#bg)'/>\n";
    out << "<circle cx='1118' cy='98' r='184' fill='#d7e8ff' fill-opacity='0.48'/>\n";
    out << "<circle cx='186' cy='1480' r='236' fill='#fde3b4' fill-opacity='0.26'/>\n";
    out << "<circle cx='1238' cy='1084' r='152' fill='#cdeee5' fill-opacity='0.34'/>\n";

    auto x_of = [&](double value) {
        return 160.0 + (value - t_min) * 1000.0 / (t_max - t_min);
    };
    auto y_of = [&](double value, double lo, double hi, double top) {
        return top + 94.0 + (hi - value) * 118.0 / (hi - lo);
    };
    auto draw_panel = [&](double top, const std::string& title, const std::string& ylabel, double lo, double hi, bool show_x_ticks, const std::string& accent) {
        out << "<rect x='64' y='" << top << "' width='1152' height='236' rx='30' fill='url(#panel)' stroke='#e4d8cb' stroke-width='1.1' filter='url(#shadow)'/>\n";
        out << "<rect x='92' y='" << top + 18.0 << "' width='96' height='22' rx='11' fill='" << accent << "' fill-opacity='0.12'/>\n";
        out << "<text x='107' y='" << top + 33.0 << "' font-size='11' letter-spacing='1.4' font-family='Avenir Next, Helvetica Neue, Arial, sans-serif' fill='" << accent << "'>SIGNAL</text>\n";
        out << "<text x='92' y='" << top + 62.0 << "' font-size='20' font-weight='650' font-family='Avenir Next, Helvetica Neue, Arial, sans-serif' fill='#14212b'>" << title << "</text>\n";
        out << "<text x='48' y='" << top + 153.0 << "' font-size='12' font-family='Avenir Next, Helvetica Neue, Arial, sans-serif' fill='#62574f' transform='rotate(-90 48 "
            << top + 153.0 << ")'>" << ylabel << "</text>\n";
        out << "<rect x='160' y='" << top + 94.0 << "' width='1000' height='118' rx='18' fill='#fffdfb' stroke='#efe4d7' stroke-width='1'/>\n";
        for (int i = 0; i <= 4; i++) {
            double x = 160.0 + 1000.0 * i / 4.0;
            out << "<line x1='" << x << "' y1='" << top + 94.0 << "' x2='" << x << "' y2='" << top + 212.0
                << "' stroke='#ede3d7' stroke-width='1' stroke-dasharray='4 7'/>\n";
            if (show_x_ticks) {
                double tick = t_min + (t_max - t_min) * i / 4.0;
                out << "<text x='" << x << "' y='" << top + 228.0 << "' text-anchor='middle' font-size='11' font-family='Avenir Next, Helvetica Neue, Arial, sans-serif' fill='#6b5f56'>"
                    << std::fixed << std::setprecision(1) << tick << "</text>\n";
            }
        }
        for (int i = 0; i <= 4; i++) {
            double y = top + 94.0 + 118.0 * i / 4.0;
            double tick = hi - (hi - lo) * i / 4.0;
            out << "<line x1='160' y1='" << y << "' x2='1160' y2='" << y << "' stroke='#ede3d7' stroke-width='1' stroke-dasharray='4 7'/>\n";
            out << "<text x='146' y='" << y + 4.0 << "' text-anchor='end' font-size='11' font-family='Avenir Next, Helvetica Neue, Arial, sans-serif' fill='#6b5f56'>"
                << std::fixed << std::setprecision(1) << tick << "</text>\n";
        }
    };
    auto draw_zero_line = [&](double top, double lo, double hi) {
        if (lo >= 0.0 || hi <= 0.0) {
            return;
        }
        out << "<line x1='160' y1='" << y_of(0.0, lo, hi, top) << "' x2='1160' y2='" << y_of(0.0, lo, hi, top)
            << "' stroke='#9f9489' stroke-width='1.2' stroke-dasharray='6 6'/>\n";
    };
    auto draw_series = [&](double top, double lo, double hi, const std::vector<double>& values, const std::string& color, double width, double opacity) {
        std::ostringstream path_data;
        path_data << std::fixed << std::setprecision(2);
        bool has_point = false;
        bool move = true;
        for (size_t i = 0; i < values.size(); i++) {
            if (!std::isfinite(values[i])) {
                move = true;
                continue;
            }
            path_data << (move ? "M " : " L ") << x_of(t[i]) << " " << y_of(values[i], lo, hi, top);
            move = false;
            has_point = true;
        }
        if (!has_point) {
            return;
        }
        out << "<path d='" << path_data.str() << "' fill='none' stroke='" << color << "' stroke-width='" << width
            << "' stroke-opacity='" << opacity << "' stroke-linejoin='round' stroke-linecap='round'/>\n";
    };
    auto draw_points = [&](double top, double lo, double hi, const std::vector<double>& values, const std::string& color, double radius, double opacity) {
        for (size_t i = 0; i < values.size(); i++) {
            if (!std::isfinite(values[i])) {
                continue;
            }
            out << "<circle cx='" << x_of(t[i]) << "' cy='" << y_of(values[i], lo, hi, top)
                << "' r='" << radius << "' fill='" << color << "' fill-opacity='" << opacity << "'/>\n";
        }
    };
    auto draw_flag_bands = [&](double top, bool stutter, const std::string& color, double opacity) {
        size_t i = 0;
        while (i < rows.size()) {
            bool active = stutter ? rows[i].stutter : rows[i].occluded;
            if (!active) {
                i++;
                continue;
            }
            size_t begin = i;
            while (i + 1 < rows.size() && (stutter ? rows[i + 1].stutter : rows[i + 1].occluded)) {
                i++;
            }
            size_t end = i;
            double left = std::max(160.0, x_of(t[begin]) - 2.4);
            double right = std::min(1160.0, x_of(t[end]) + 2.4);
            out << "<rect x='" << left << "' y='" << top + 94.0 << "' width='" << std::max(4.8, right - left)
                << "' height='118' rx='7' fill='" << color << "' fill-opacity='" << opacity << "'/>\n";
            i++;
        }
    };
    auto draw_flags = [&](double top) {
        draw_flag_bands(top, false, "#b91c1c", 0.18);
        draw_flag_bands(top, true, "#f59e0b", 0.16);
    };
    auto draw_legend = [&](double top, std::initializer_list<std::pair<std::string, std::string>> items) {
        double total = 0.0;
        for (const auto& item: items) {
            total += std::max(82.0, 42.0 + static_cast<double>(item.first.size()) * 7.0) + 10.0;
        }
        double x = 1160.0 - total;
        for (const auto& item: items) {
            double width = std::max(82.0, 42.0 + static_cast<double>(item.first.size()) * 7.0);
            out << "<rect x='" << x << "' y='" << top + 16.0 << "' width='" << width << "' height='24' rx='12' fill='#fffaf4' stroke='#eadfd3'/>\n";
            out << "<circle cx='" << x + 17.0 << "' cy='" << top + 28.0 << "' r='5' fill='" << item.second << "'/>\n";
            out << "<text x='" << x + 30.0 << "' y='" << top + 32.0 << "' font-size='11' font-family='Avenir Next, Helvetica Neue, Arial, sans-serif' fill='#564b44'>"
                << item.first << "</text>\n";
            x += width + 10.0;
        }
    };

    draw_panel(36.0, "Target Yaw", "yaw (deg)", yaw_range.first, yaw_range.second, false, "#2563eb");
    draw_series(36.0, yaw_range.first, yaw_range.second, pred_yaw, "#2563eb", 2.3, 0.98);
    draw_series(36.0, yaw_range.first, yaw_range.second, truth_yaw, "#d97706", 2.0, 0.95);
    draw_legend(36.0, {{"target", "#2563eb"}, {"truth", "#d97706"}});

    draw_panel(290.0, "Raw Yaw", "yaw (deg)", raw_range.first, raw_range.second, false, "#334155");
    draw_series(290.0, raw_range.first, raw_range.second, raw_yaw, "#64748b", 1.8, 0.80);
    draw_points(290.0, raw_range.first, raw_range.second, raw_yaw, "#334155", 1.9, 0.72);
    draw_legend(290.0, {{"raw", "#334155"}});

    draw_panel(544.0, "Yaw Error", "err (deg)", yaw_err_range.first, yaw_err_range.second, false, "#be123c");
    draw_zero_line(544.0, yaw_err_range.first, yaw_err_range.second);
    draw_series(544.0, yaw_err_range.first, yaw_err_range.second, yaw_err, "#be123c", 2.0, 0.96);

    draw_panel(798.0, "Frame Dt + Events", "dt (ms)", dt_range.first, dt_range.second, false, "#0f766e");
    draw_flags(798.0);
    draw_series(798.0, dt_range.first, dt_range.second, dt_ms, "#0f766e", 2.1, 0.97);
    draw_legend(798.0, {{"dt", "#0f766e"}, {"occluded", "#b91c1c"}, {"stutter", "#f59e0b"}});

    draw_panel(1052.0, "Target Pitch", "pitch (deg)", pitch_range.first, pitch_range.second, false, "#2563eb");
    draw_series(1052.0, pitch_range.first, pitch_range.second, pred_pitch, "#2563eb", 2.3, 0.98);
    draw_series(1052.0, pitch_range.first, pitch_range.second, truth_pitch, "#d97706", 2.0, 0.95);
    draw_legend(1052.0, {{"target", "#2563eb"}, {"truth", "#d97706"}});

    draw_panel(1306.0, "Pitch Error", "err (deg)", pitch_err_range.first, pitch_err_range.second, true, "#be123c");
    draw_zero_line(1306.0, pitch_err_range.first, pitch_err_range.second);
    draw_series(1306.0, pitch_err_range.first, pitch_err_range.second, pitch_err, "#be123c", 2.0, 0.96);
    out << "<text x='640' y='1548' font-size='12' font-family='Avenir Next, Helvetica Neue, Arial, sans-serif' fill='#62574f'>time (s)</text>\n";
    out << "</svg>\n";
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
