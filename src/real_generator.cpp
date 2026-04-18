#include "../include/real_generator.hpp"

namespace helios_cv
{
namespace
{

std::pair<double, double> symmetric_const_translate(double t, double duration, double speed)
{
    double half = std::max(duration * 0.5, 1e-6);
    double edge = speed * half * 0.5;
    if (t < half) {
        return {-edge + speed * t, speed};
    }
    return {edge - speed * (t - half), -speed};
}

std::pair<double, double> symmetric_var_translate(double t, double duration, double speed)
{
    double mid = std::max(duration * 0.5, 1e-6);
    if (t < mid) {
        return {-speed * t + speed * t * t / mid, -speed + 2.0 * speed * t / mid};
    }
    double dt = t - mid;
    return {speed * dt - speed * dt * dt / mid, speed - 2.0 * speed * dt / mid};
}

std::pair<double, double> symmetric_height_offset(double t, double height)
{
    double omega = pi() / 5.0;
    return {height * std::sin(omega * t), height * omega * std::cos(omega * t)};
}

} // namespace

RealGenerator::RealGenerator(const SimConfig& cfg): cfg_(cfg) {}

std::vector<std::tuple<int, double, double>> RealGenerator::associate_rotating_armors(
    const std::vector<ArmorObservation>& obs_list, double pred_yaw_continuous, int armor_num)
{
    std::vector<std::tuple<int, double, double>> processed_obs;
    double angle_step = 2.0 * pi() / armor_num;

    for (const auto& raw_obs: obs_list) {
        double diff = raw_obs.orient_yaw - pred_yaw_continuous;
        while (diff > pi()) diff -= 2.0 * pi();
        while (diff < -pi()) diff += 2.0 * pi();

        int k = std::round(diff / angle_step);
        int id = (k % armor_num + armor_num) % armor_num;
        double yaw_offset = k * angle_step;

        double target = pred_yaw_continuous + yaw_offset;
        double cont_yaw = raw_obs.orient_yaw;
        while (cont_yaw - target > pi()) cont_yaw -= 2.0 * pi();
        while (cont_yaw - target < -pi()) cont_yaw += 2.0 * pi();

        processed_obs.emplace_back(id, yaw_offset, cont_yaw);
    }
    return processed_obs;
}

PoseSample RealGenerator::camera_pose_at(double t, double gimbal_yaw, double gimbal_pitch) const
{
    double ego_yaw = angles::from_degrees(cfg_.ego_yaw_amp_deg)
        * (0.80 * std::sin(0.37 * t) + 0.20 * std::sin(1.11 * t));
    double ego_pitch = angles::from_degrees(cfg_.ego_pitch_amp_deg)
        * (0.75 * std::sin(0.29 * t + 0.30) + 0.25 * std::sin(1.07 * t));
    double yaw = gimbal_yaw + ego_yaw;
    double pitch = gimbal_pitch + ego_pitch;
    PoseSample pose;
    pose.R_c_w = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix()
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix()
        * base_camera_pose();
    pose.t_c_w = Eigen::Vector3d(
        cfg_.ego_xy_amp * (0.75 * std::sin(0.17 * t) + 0.25 * std::sin(0.93 * t)),
        cfg_.ego_xy_amp * (0.60 * std::cos(0.21 * t) + 0.40 * std::sin(0.57 * t + 0.40)),
        cfg_.ego_z_amp * std::sin(0.31 * t + 0.20));
    return pose;
}

ArmorAim RealGenerator::make_standard_truth(double t, bool singer) const
{
    ArmorAim aim;
    aim.id = singer ? 17 : 11;
    aim.type = singer ? 1 : 0;

    double distance = resolved_distance(cfg_, "standard");
    double cx = distance;
    double cy = 0.0;
    double cz = singer ? 0.68 : 0.72;
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    double yaw = singer ? -0.3 : 0.6;

    if (cfg_.standard_mode == "translate_const") {
        cx = distance;
        std::tie(cy, vy) = symmetric_const_translate(t, cfg_.duration, 1.0);
        cz = singer ? 0.68 : 0.72;
        vx = 0.0;
        vz = 0.0;
        yaw = singer ? -0.35 : 0.55;
    } else if (cfg_.standard_mode == "translate_var") {
        cz = singer ? 0.68 : 0.72;
        std::tie(cy, vy) = symmetric_var_translate(t, cfg_.duration, 2.0);
        vz = 0.0;
        yaw = singer ? -0.35 : 0.55;
    } else {
        throw std::runtime_error("unknown standard-mode: " + cfg_.standard_mode);
    }

    double radius = aim.type == 0 ? 0.20 : 0.23;
    aim.center_pos = Eigen::Vector3d(cx, cy, cz);
    aim.center_vel = Eigen::Vector3d(vx, vy, vz);
    aim.orient_yaw = yaw;
    aim.radius = radius;
    aim.z = cz;
    aim.aim_pos = Eigen::Vector3d(cx + radius * std::cos(yaw), cy + radius * std::sin(yaw), cz);
    aim.last_seen_time = t;
    aim.trusted = true;
    return aim;
}

std::vector<ArmorAim> RealGenerator::make_top_truth(double t, double& vyaw) const
{
    double distance = resolved_distance(cfg_, "top");
    double base_yaw = 0.0;
    double seg_t = std::max(cfg_.duration / 3.0, 1e-6);
    if (cfg_.top_mode == "spin_const") {
        vyaw = 2.0 * pi();
        base_yaw = vyaw * t;
    } else if (cfg_.top_mode == "spin_var") {
        if (t < seg_t) {
            vyaw = 2.0 * pi() / 3.0;
            base_yaw = vyaw * t;
        } else if (t < 2.0 * seg_t) {
            vyaw = 4.0 * pi();
            base_yaw = 2.0 * pi() * seg_t / 3.0 + vyaw * (t - seg_t);
        } else {
            vyaw = 2.0 * pi();
            base_yaw = 2.0 * pi() * seg_t / 3.0 + 4.0 * pi() * seg_t + vyaw * (t - 2.0 * seg_t);
        }
    } else if (cfg_.top_mode == "spin_const_translate_const") {
        vyaw = 2.0 * pi();
        base_yaw = vyaw * t;
    } else if (cfg_.top_mode == "spin_var_translate_var") {
        if (t < seg_t) {
            vyaw = pi();
            base_yaw = vyaw * t;
        } else if (t < 2.0 * seg_t) {
            vyaw = 4.0 * pi();
            base_yaw = pi() * seg_t + vyaw * (t - seg_t);
        } else {
            vyaw = 2.0 * pi();
            base_yaw = pi() * seg_t + 4.0 * pi() * seg_t + vyaw * (t - 2.0 * seg_t);
        }
    } else if (cfg_.top_mode == "spin_const_height_var") {
        vyaw = 2.0 * pi();
        base_yaw = vyaw * t;
    } else if (cfg_.top_mode == "spin_var_height_var") {
        if (t < seg_t) {
            vyaw = 2.0 * pi() / 3.0;
            base_yaw = vyaw * t;
        } else if (t < 2.0 * seg_t) {
            vyaw = 4.0 * pi();
            base_yaw = 2.0 * pi() * seg_t / 3.0 + vyaw * (t - seg_t);
        } else {
            vyaw = 2.0 * pi();
            base_yaw = 2.0 * pi() * seg_t / 3.0 + 4.0 * pi() * seg_t + vyaw * (t - 2.0 * seg_t);
        }
    } else {
        throw std::runtime_error("unknown top-mode: " + cfg_.top_mode);
    }

    bool translate_const = cfg_.top_mode == "spin_const_translate_const";
    bool translate_var = cfg_.top_mode == "spin_var_translate_var";
    bool height_var_50 = cfg_.top_mode == "spin_const_height_var";
    bool height_var_40 = cfg_.top_mode == "spin_var_height_var";
    double cx = distance;
    double cy = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    if (translate_const) {
        std::tie(cy, vy) = symmetric_const_translate(t, cfg_.duration, 0.8);
    } else if (translate_var) {
        std::tie(cy, vy) = symmetric_var_translate(t, cfg_.duration, 1.5);
    }
    double z_offset = 0.0;
    double vz = 0.0;
    if (height_var_50) {
        std::tie(z_offset, vz) = symmetric_height_offset(t, 0.50);
    } else if (height_var_40) {
        std::tie(z_offset, vz) = symmetric_height_offset(t, 0.40);
    }
    double z0 = 0.52 + z_offset;
    double z1 = 0.67 + z_offset;
    double center_z = 0.5 * (z0 + z1);

    std::vector<ArmorAim> aims;
    for (int id = 0; id < 4; id++) {
        double radius = id % 2 == 0 ? 0.20 : 0.23;
        double z = id % 2 == 0 ? z0 : z1;
        double yaw = base_yaw + id * pi() * 0.5;
        ArmorAim aim;
        aim.id = id;
        aim.type = 1;
        aim.center_pos = Eigen::Vector3d(cx, cy, center_z);
        aim.center_vel = Eigen::Vector3d(vx, vy, vz);
        aim.orient_yaw = yaw;
        aim.radius = radius;
        aim.z = z;
        aim.aim_pos = Eigen::Vector3d(cx + radius * std::cos(yaw), cy + radius * std::sin(yaw), z);
        aim.last_seen_time = t;
        aim.trusted = true;
        aims.push_back(aim);
    }
    return aims;
}

std::vector<ArmorAim> RealGenerator::make_top3_truth(double t, double& vyaw) const
{
    double distance = resolved_distance(cfg_, "outpost");
    if (cfg_.outpost_mode != "outpost_standard") {
        throw std::runtime_error("unknown outpost-mode: " + cfg_.outpost_mode);
    }
    vyaw = 0.8 * pi() + 0.04 * pi() * std::cos(0.20 * t);
    double base_yaw = 0.8 * pi() * t + 0.2 * pi() * std::sin(0.20 * t);

    double cx = distance;
    double cy = -0.25;
    double cz = 0.76;
    double dz = 0.102;

    std::vector<ArmorAim> aims;
    for (int id = 0; id < 3; id++) {
        double scale = id == 0 ? 0.0 : (id == 1 ? 1.0 : -1.0);
        double yaw = base_yaw + id * 2.0 * pi() / 3.0;
        ArmorAim aim;
        aim.id = id;
        aim.type = 0;
        aim.center_pos = Eigen::Vector3d(cx, cy, cz);
        aim.center_vel = Eigen::Vector3d::Zero();
        aim.orient_yaw = yaw;
        aim.radius = 0.275;
        aim.z = cz + dz * scale;
        aim.aim_pos = Eigen::Vector3d(cx + 0.275 * std::cos(yaw), cy + 0.275 * std::sin(yaw), aim.z);
        aim.last_seen_time = t;
        aim.trusted = true;
        aims.push_back(aim);
    }
    return aims;
}

ArmorObservation RealGenerator::make_observation(
    const ArmorAim& truth, double t, const CameraParam& cam, const PoseSample& pose, std::mt19937& rng, double armor_pitch) const
{
    ArmorObservation obs;
    obs.t = t;
    obs.type = truth.type;
    obs.id = truth.id;
    obs.pos = truth.aim_pos + Eigen::Vector3d(
        gauss(rng, cfg_.pos_xy_sigma),
        gauss(rng, cfg_.pos_xy_sigma),
        gauss(rng, cfg_.pos_z_sigma));
    obs.orient_yaw = angles::normalize_angle(truth.orient_yaw + gauss(rng, angles::from_degrees(cfg_.yaw_sigma_deg)));
    obs.corners = project_armor(cam, pose, truth.aim_pos, truth.orient_yaw, truth.type, armor_pitch);
    for (auto& corner: obs.corners) {
        corner.x += gauss(rng, cfg_.corner_sigma);
        corner.y += gauss(rng, cfg_.corner_sigma);
    }
    obs.area = polygon_area(obs.corners);
    return obs;
}

std::vector<ArmorObservation> RealGenerator::make_rotating_observations(
    const std::vector<ArmorAim>& truth_aims, double t, const CameraParam& cam, const PoseSample& pose,
    std::mt19937& rng, int max_count, double armor_pitch) const
{
    std::vector<std::pair<double, ArmorObservation>> ranked;
    for (const auto& truth: truth_aims) {
        ArmorObservation obs = make_observation(truth, t, cam, pose, rng, armor_pitch);
        if (obs.corners.size() != 4) {
            continue;
        }
        bool inside = true;
        for (const auto& corner: obs.corners) {
            if (corner.x < -60.0 || corner.x > cam.width + 60.0 || corner.y < -60.0 || corner.y > cam.height + 60.0) {
                inside = false;
                break;
            }
        }
        if (!inside || obs.area < 20.0) {
            continue;
        }
        double center2cam_yaw = std::atan2(-truth.center_pos.y(), -truth.center_pos.x());
        double diff = std::abs(angles::normalize_angle(truth.orient_yaw - center2cam_yaw));
        if (diff > angles::from_degrees(85.0)) {
            continue;
        }
        ranked.emplace_back(diff, obs);
    }

    std::sort(ranked.begin(), ranked.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.first < rhs.first;
    });

    std::vector<ArmorObservation> obs_list;
    for (size_t i = 0; i < ranked.size() && static_cast<int>(i) < max_count; i++) {
        obs_list.push_back(ranked[i].second);
    }

    if (obs_list.size() > 1 && uniform01(rng) < cfg_.drop_prob * 0.8) {
        obs_list.resize(obs_list.size() - 1);
    }
    std::shuffle(obs_list.begin(), obs_list.end(), rng);
    return obs_list;
}

std::vector<cv::Point2d> RealGenerator::project_armor(
    const CameraParam& cam, const PoseSample& pose, const Eigen::Vector3d& pos, double yaw, int type, double armor_pitch)
{
    double half_width = (type == 0 ? 0.135 : 0.23) * 0.5;
    double half_height = 0.125 * 0.5;
    Eigen::Matrix3d armor_R =
        (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(armor_pitch, Eigen::Vector3d::UnitY())).toRotationMatrix();

    std::vector<Eigen::Vector3d> corners_l = {
        {0.0, half_width, half_height},
        {0.0, -half_width, half_height},
        {0.0, -half_width, -half_height},
        {0.0, half_width, -half_height},
    };

    double fx = cam.K.at<double>(0, 0);
    double fy = cam.K.at<double>(1, 1);
    double cx = cam.K.at<double>(0, 2);
    double cy = cam.K.at<double>(1, 2);

    std::vector<cv::Point2d> out;
    out.reserve(4);
    for (const auto& corner_l: corners_l) {
        Eigen::Vector3d corner_w = pos + armor_R * corner_l;
        Eigen::Vector3d corner_c = pose.R_c_w.transpose() * (corner_w - pose.t_c_w);
        if (corner_c.z() <= 1e-6) {
            return {};
        }
        out.emplace_back(
            fx * corner_c.x() / corner_c.z() + cx,
            fy * corner_c.y() / corner_c.z() + cy);
    }
    return out;
}

double RealGenerator::polygon_area(const std::vector<cv::Point2d>& corners)
{
    if (corners.size() < 3) {
        return 0.0;
    }
    double area = 0.0;
    for (size_t i = 0; i < corners.size(); i++) {
        const auto& p0 = corners[i];
        const auto& p1 = corners[(i + 1) % corners.size()];
        area += p0.x * p1.y - p1.x * p0.y;
    }
    return std::abs(area) * 0.5;
}

} // namespace helios_cv
