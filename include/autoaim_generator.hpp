#pragma once

#include "node_params.hpp"
#include "real_generator.hpp"

namespace helios_cv
{

class AutoaimGenerator {
public:
    explicit AutoaimGenerator(const SimConfig& cfg);

    Report run_simple_tracker() const;
    Report run_singer_tracker() const;
    Report run_top_tracker() const;
    Report run_top3_tracker() const;
    // Report run_small_tracker() const;
    // Report run_big_tracker() const;

private:
    static ArmorAim select_standard_aim(const std::vector<ArmorAim>& aims, double vyaw, int top_level, double t, int armor_type, const Eigen::Vector3d& camera_pos, SelectorState& state, int forced_id = -1);
    static ArmorAim select_outpost_aim(const std::vector<ArmorAim>& aims, double vyaw, double t, const Eigen::Vector3d& camera_pos, SelectorState& state, int forced_id = -1);
    static void update_standard_lock(const ArmorAim& aim, int top_level, double t, SelectorState& state);
    static void update_outpost_lock(const ArmorAim& aim, double t, SelectorState& state);
    static int update_top_level(int level, double vyaw);
    // Report run_energy_tracker(bool big) const;

    SimConfig cfg_;
    RealGenerator real_generator_;
    TrackerParamSet tracker_params_;
};

} // namespace helios_cv
