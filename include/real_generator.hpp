#pragma once

#include "preset.hpp"

#include <tuple>

namespace helios_cv
{

class RealGenerator {
public:
    explicit RealGenerator(const SimConfig& cfg);

    static std::vector<std::tuple<int, double, double>> associate_rotating_armors(
        const std::vector<ArmorObservation>& obs_list, double pred_yaw_continuous, int armor_num);

    PoseSample camera_pose_at(double t, double gimbal_yaw, double gimbal_pitch) const;
    ArmorAim make_standard_truth(double t, bool singer) const;
    std::vector<ArmorAim> make_top_truth(double t, double& vyaw) const;
    std::vector<ArmorAim> make_top3_truth(double t, double& vyaw) const;
    ArmorObservation make_observation(
        const ArmorAim& truth, double t, const CameraParam& cam, const PoseSample& pose, std::mt19937& rng,
        double armor_pitch) const;
    std::vector<ArmorObservation> make_rotating_observations(
        const std::vector<ArmorAim>& truth_aims, double t, const CameraParam& cam, const PoseSample& pose,
        std::mt19937& rng, int max_count, double armor_pitch) const;

private:
    static std::vector<cv::Point2d> project_armor(
        const CameraParam& cam, const PoseSample& pose, const Eigen::Vector3d& pos, double yaw, int type, double armor_pitch);
    static double polygon_area(const std::vector<cv::Point2d>& corners);

    SimConfig cfg_;
};

} // namespace helios_cv
