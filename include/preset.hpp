#pragma once

#include "sim_utilities.hpp"

#include <array>

namespace helios_cv
{

struct SimPreset {
    const char* name;
    double fps;
    double duration;
    double image_delay_ms;
    double imu_delay_ms;
    double imu_yaw_bias_deg;
    double imu_pitch_bias_deg;
    double dt_jitter_ratio;
    double stutter_prob;
    double stutter_mult;
    double drop_prob;
    int drop_min_frames;
    int drop_max_frames;
    double pos_xy_sigma;
    double pos_z_sigma;
    double yaw_sigma_deg;
    double corner_sigma;
    double latency_ms;
    double ego_xy_amp;
    double ego_z_amp;
    double ego_yaw_amp_deg;
    double ego_pitch_amp_deg;
};

inline std::string canonical_preset(std::string preset)
{
    if (preset == "default" || preset == "clean" || preset == "light_disturb"
        || preset == "medium_disturb" || preset == "heavy_disturb" || preset == "extreme_disturb")
    {
        return preset;
    }
    if (preset == "field" || preset == "noisy_delay") return "medium_disturb";
    if (preset == "drop_stutter" || preset == "stress") return "heavy_disturb";
    if (preset == "extreme") return "extreme_disturb";
    return preset;
}

inline std::string canonical_standard_mode(std::string mode)
{
    if (mode == "translate_const" || mode == "translate_var") {
        return mode;
    }
    if (mode == "translate_accel" || mode == "translate_mixed") {
        return "translate_var";
    }
    return mode;
}

inline std::string canonical_top_mode(std::string mode)
{
    if (mode == "spin_const" || mode == "spin_var" || mode == "spin_const_translate_const"
        || mode == "spin_var_translate_var" || mode == "spin_const_height_var" || mode == "spin_var_height_var")
    {
        return mode;
    }
    if (mode == "top_stationary") return "spin_const";
    if (mode == "top_ramp") return "spin_var";
    if (mode == "top_translate") return "spin_var_translate_var";
    if (mode == "top_height_const") return "spin_const_height_var";
    if (mode == "top_height_var") return "spin_var_height_var";
    return mode;
}

inline std::string canonical_outpost_mode(std::string mode)
{
    if (mode == "outpost_standard") {
        return mode;
    }
    if (mode == "outpost_spin" || mode == "outpost_translate") {
        return "outpost_standard";
    }
    return mode;
}

inline const std::array<SimPreset, 5>& sim_presets()
{
    // 新增 preset 模板：
    // 1. 先复制下面一行，改 `name`
    // 2. 再按顺序填写：
    //    fps, duration, image_delay_ms, imu_delay_ms,
    //    imu_yaw_bias_deg, imu_pitch_bias_deg, dt_jitter_ratio,
    //    stutter_prob, stutter_mult, drop_prob, drop_min_frames, drop_max_frames,
    //    pos_xy_sigma, pos_z_sigma, yaw_sigma_deg, corner_sigma,
    //    latency_ms, ego_xy_amp, ego_z_amp, ego_yaw_amp_deg, ego_pitch_amp_deg
    // 3. 如果要兼容旧名字，再去 `canonical_preset(...)` 里补别名映射
    //
    // {"your_preset_name", 100.0, 20.0, 4.0, 1.0, 0.8, 0.4, 0.035, 0.015, 2.5, 0.02, 2, 6, 0.009, 0.0045, 1.6, 0.65, 12.0, 0.022, 0.006, 0.35, 0.28},
    static const std::array<SimPreset, 5> presets = {{
        {"clean", 100.0, 20.0, 1.5, 0.5, 0.15, 0.10, 0.005, 0.0, 2.0, 0.0, 2, 4, 0.002, 0.001, 0.35, 0.18, 8.0, 0.003, 0.001, 0.05, 0.04},
        {"light_disturb", 100.0, 20.0, 3.0, 1.0, 0.5, 0.25, 0.02, 0.005, 2.2, 0.01, 2, 4, 0.006, 0.003, 1.0, 0.45, 10.0, 0.012, 0.003, 0.18, 0.14},
        {"medium_disturb", 100.0, 20.0, 4.0, 1.0, 0.8, 0.4, 0.035, 0.015, 2.5, 0.02, 2, 6, 0.009, 0.0045, 1.6, 0.65, 12.0, 0.022, 0.006, 0.35, 0.28},
        {"heavy_disturb", 100.0, 20.0, 8.0, 3.0, 1.2, 0.7, 0.08, 0.04, 3.0, 0.045, 2, 8, 0.014, 0.007, 2.4, 1.1, 16.0, 0.045, 0.012, 0.75, 0.55},
        {"extreme_disturb", 100.0, 20.0, 12.0, 5.0, 1.8, 1.0, 0.12, 0.08, 4.0, 0.08, 4, 12, 0.020, 0.010, 3.2, 1.6, 20.0, 0.080, 0.020, 1.20, 0.90},
    }};
    return presets;
}

inline const SimPreset* find_sim_preset(const std::string& preset)
{
    const auto& presets = sim_presets();
    auto it = std::find_if(presets.begin(), presets.end(), [&](const SimPreset& item) {
        return item.name == preset;
    });
    return it == presets.end() ? nullptr : &*it;
}

inline void apply_sim_preset(SimConfig& cfg, const SimPreset& preset)
{
    cfg.fps = preset.fps;
    cfg.duration = preset.duration;
    cfg.image_delay_ms = preset.image_delay_ms;
    cfg.imu_delay_ms = preset.imu_delay_ms;
    cfg.imu_yaw_bias_deg = preset.imu_yaw_bias_deg;
    cfg.imu_pitch_bias_deg = preset.imu_pitch_bias_deg;
    cfg.dt_jitter_ratio = preset.dt_jitter_ratio;
    cfg.stutter_prob = preset.stutter_prob;
    cfg.stutter_mult = preset.stutter_mult;
    cfg.drop_prob = preset.drop_prob;
    cfg.drop_min_frames = preset.drop_min_frames;
    cfg.drop_max_frames = preset.drop_max_frames;
    cfg.pos_xy_sigma = preset.pos_xy_sigma;
    cfg.pos_z_sigma = preset.pos_z_sigma;
    cfg.yaw_sigma_deg = preset.yaw_sigma_deg;
    cfg.corner_sigma = preset.corner_sigma;
    cfg.latency_ms = preset.latency_ms;
    cfg.ego_xy_amp = preset.ego_xy_amp;
    cfg.ego_z_amp = preset.ego_z_amp;
    cfg.ego_yaw_amp_deg = preset.ego_yaw_amp_deg;
    cfg.ego_pitch_amp_deg = preset.ego_pitch_amp_deg;
}

} // namespace helios_cv
