#include "../include/autoaim_generator.hpp"

#include <stdexcept>

namespace helios_cv
{
namespace
{

TrackerParamSet resolve_tracker_params(const SimConfig& cfg)
{
    std::string path = cfg.params_file.empty() ? find_default_params_file() : cfg.params_file;
    if (path.empty()) {
        throw std::runtime_error("failed to find node_params.yaml, please pass --params-file");
    }
    return load_tracker_params(path);
}

} // namespace

AutoaimGenerator::AutoaimGenerator(const SimConfig& cfg): cfg_(cfg), real_generator_(cfg), tracker_params_(resolve_tracker_params(cfg)) {}

Aim AutoaimGenerator::select_standard_aim(
    const std::vector<Aim>& aims, double vyaw, int top_level, double t, int armor_type, const Eigen::Vector3d& camera_pos, SelectorState& state, int forced_id)
{
    Aim result;
    if (aims.empty()) {
        return result;
    }

    auto armor_pos = [](const Aim& aim) {
        return Eigen::Vector3d(
            aim.center_pos.x() + aim.radius * std::cos(aim.orient_yaw),
            aim.center_pos.y() + aim.radius * std::sin(aim.orient_yaw),
            aim.z);
    };

    int last_id = -1;
    if (t - state.last_lock_t <= 0.2) {
        last_id = top_level == 0 ? state.top0_lock_id : state.lock_id;
    }

    auto in_view = [&](const Aim& aim) {
        double center2cam_yaw = std::atan2(camera_pos.y() - aim.center_pos.y(), camera_pos.x() - aim.center_pos.x());
        double diff = angles::normalize_angle(aim.orient_yaw - center2cam_yaw);
        double limit = 0.5 * 1.8 * (armor_type == 0 ? 0.135 : 0.23) / std::max(aim.radius, 1e-6);
        return std::abs(diff) <= limit;
    };

    double keep = angles::from_degrees(10.0);

    if (top_level == 0) {
        Aim best_armor;
        Aim lock_armor;
        double best_diff = std::numeric_limits<double>::infinity();
        double lock_diff = std::numeric_limits<double>::infinity();
        int visible_count = 0;

        for (const auto& aim: aims) {
            if (!in_view(aim)) {
                continue;
            }
            visible_count += 1;
            double center2cam_yaw = std::atan2(camera_pos.y() - aim.center_pos.y(), camera_pos.x() - aim.center_pos.x());
            double diff = std::abs(angles::normalize_angle(aim.orient_yaw - center2cam_yaw));
            if (forced_id >= 0 && aim.id == forced_id) {
                Aim selected = aim;
                selected.aim_pos = armor_pos(selected);
                return selected;
            }
            if (diff < best_diff) {
                best_diff = diff;
                best_armor = aim;
            }
            if (aim.id == last_id) {
                lock_diff = diff;
                lock_armor = aim;
            }
        }

        if (forced_id >= 0 || visible_count == 0) {
            return result;
        }
        if (visible_count == 1) {
            best_armor.aim_pos = armor_pos(best_armor);
            return best_armor;
        }
        if (last_id >= 0 && lock_diff < std::numeric_limits<double>::infinity() && best_diff + keep >= lock_diff) {
            lock_armor.aim_pos = armor_pos(lock_armor);
            return lock_armor;
        }
        best_armor.aim_pos = armor_pos(best_armor);
        return best_armor;
    }

    if (top_level == 2) {
        Aim best_armor = aims[0];
        double min_wait = std::numeric_limits<double>::infinity();
        double lock_wait = std::numeric_limits<double>::infinity();

        for (const auto& aim: aims) {
            double center2cam_yaw = std::atan2(camera_pos.y() - aim.center_pos.y(), camera_pos.x() - aim.center_pos.x());
            double diff = angles::normalize_angle(aim.orient_yaw - center2cam_yaw);
            double limit = 0.5 * 1.8 * (armor_type == 0 ? 0.135 : 0.23) / std::max(aim.radius, 1e-6);
            double wait = 0.0;
            if (std::abs(diff) > limit) {
                double enter = vyaw > 0.0 ? angles::normalize_angle(-limit - diff) : angles::normalize_angle(diff - limit);
                if (enter < 0.0) {
                    enter += 2.0 * pi();
                }
                wait = enter / std::abs(vyaw);
            }
            if (forced_id >= 0 && aim.id == forced_id) {
                Aim selected = aim;
                double aim_yaw = selected.orient_yaw + wait * vyaw;
                selected.orient_yaw = aim_yaw;
                selected.aim_pos = Eigen::Vector3d(
                    selected.center_pos.x() + selected.radius * std::cos(aim_yaw) + wait * selected.center_vel.x(),
                    selected.center_pos.y() + selected.radius * std::sin(aim_yaw) + wait * selected.center_vel.y(),
                    selected.z + wait * selected.center_vel.z());
                selected.z = selected.aim_pos.z();
                return selected;
            }
            if (wait < min_wait) {
                min_wait = wait;
                best_armor = aim;
            }
            if (aim.id == last_id) {
                lock_wait = wait;
            }
        }

        if (forced_id >= 0) {
            return result;
        }
        if (last_id >= 0 && lock_wait <= min_wait + keep) {
            min_wait = lock_wait;
            for (const auto& aim: aims) {
                if (aim.id == last_id) {
                    best_armor = aim;
                    break;
                }
            }
        }
        double aim_yaw = best_armor.orient_yaw + min_wait * vyaw;
        best_armor.orient_yaw = aim_yaw;
        best_armor.aim_pos = Eigen::Vector3d(
            best_armor.center_pos.x() + best_armor.radius * std::cos(aim_yaw) + min_wait * best_armor.center_vel.x(),
            best_armor.center_pos.y() + best_armor.radius * std::sin(aim_yaw) + min_wait * best_armor.center_vel.y(),
            best_armor.z + min_wait * best_armor.center_vel.z());
        best_armor.z = best_armor.aim_pos.z();
        return best_armor;
    }

    Aim best_armor;
    double best_diff = std::numeric_limits<double>::infinity();
    double best_wait = std::numeric_limits<double>::infinity();
    double direction = vyaw >= 0.0 ? 1.0 : -1.0;
    auto get_diff = [&](const Aim& aim) {
        double center2cam_yaw = std::atan2(camera_pos.y() - aim.center_pos.y(), camera_pos.x() - aim.center_pos.x());
        return direction * angles::normalize_angle(aim.orient_yaw - center2cam_yaw);
    };

    for (const auto& aim: aims) {
        if (forced_id >= 0 && aim.id == forced_id) {
            Aim selected = aim;
            selected.aim_pos = armor_pos(selected);
            return selected;
        }
        if (aim.id != last_id) {
            continue;
        }
        double diff = get_diff(aim);
        if (diff >= -angles::from_degrees(58.8888) && diff <= angles::from_degrees(20.0)) {
            Aim selected = aim;
            selected.aim_pos = armor_pos(selected);
            return selected;
        }
        break;
    }

    if (forced_id >= 0) {
        return result;
    }

    for (const auto& aim: aims) {
        double diff = get_diff(aim);
        if (diff < -angles::from_degrees(58.8888) || diff > angles::from_degrees(20.0)) {
            continue;
        }
        if (diff < best_diff) {
            best_diff = diff;
            best_armor = aim;
        }
    }
    if (best_armor.id >= 0) {
        best_armor.aim_pos = armor_pos(best_armor);
        return best_armor;
    }

    for (const auto& aim: aims) {
        double diff = get_diff(aim);
        double enter = angles::normalize_angle(-angles::from_degrees(58.8888) - diff);
        if (enter < 0.0) {
            enter += 2.0 * pi();
        }
        double wait = std::abs(vyaw) > 1e-6 ? enter / std::abs(vyaw) : enter;
        if (wait < best_wait) {
            best_wait = wait;
            best_armor = aim;
        }
    }
    best_armor.aim_pos = armor_pos(best_armor);
    return best_armor;
}

Aim AutoaimGenerator::select_outpost_aim(const std::vector<Aim>& aims, double vyaw, double t, const Eigen::Vector3d& camera_pos, SelectorState& state, int forced_id)
{
    Aim result;
    if (aims.empty()) {
        return result;
    }

    auto armor_pos = [](const Aim& aim) {
        return Eigen::Vector3d(
            aim.center_pos.x() + aim.radius * std::cos(aim.orient_yaw),
            aim.center_pos.y() + aim.radius * std::sin(aim.orient_yaw),
            aim.z);
    };

    int last_id = t - state.last_lock_t <= 0.2 ? state.lock_id : -1;
    Aim best_armor;
    double best_diff = std::numeric_limits<double>::infinity();
    double best_wait = std::numeric_limits<double>::infinity();
    double direction = vyaw >= 0.0 ? 1.0 : -1.0;
    auto get_diff = [&](const Aim& aim) {
        double center2cam_yaw = std::atan2(camera_pos.y() - aim.center_pos.y(), camera_pos.x() - aim.center_pos.x());
        return direction * angles::normalize_angle(aim.orient_yaw - center2cam_yaw);
    };

    for (const auto& aim: aims) {
        if (forced_id >= 0 && aim.id == forced_id) {
            Aim selected = aim;
            selected.aim_pos = armor_pos(selected);
            return selected;
        }
        if (aim.id != last_id) {
            continue;
        }
        double diff = get_diff(aim);
        if (diff >= -angles::from_degrees(58.8888) && diff <= angles::from_degrees(20.0)) {
            Aim selected = aim;
            selected.aim_pos = armor_pos(selected);
            return selected;
        }
        break;
    }

    if (forced_id >= 0) {
        return result;
    }

    for (const auto& aim: aims) {
        double diff = get_diff(aim);
        if (diff < -angles::from_degrees(58.8888) || diff > angles::from_degrees(20.0)) {
            continue;
        }
        if (diff < best_diff) {
            best_diff = diff;
            best_armor = aim;
        }
    }
    if (best_armor.id >= 0) {
        best_armor.aim_pos = armor_pos(best_armor);
        return best_armor;
    }

    for (const auto& aim: aims) {
        double diff = get_diff(aim);
        double enter = angles::normalize_angle(-angles::from_degrees(58.8888) - diff);
        if (enter < 0.0) {
            enter += 2.0 * pi();
        }
        double wait = std::abs(vyaw) > 1e-6 ? enter / std::abs(vyaw) : enter;
        if (wait < best_wait) {
            best_wait = wait;
            best_armor = aim;
        }
    }
    best_armor.aim_pos = armor_pos(best_armor);
    return best_armor;
}

void AutoaimGenerator::update_standard_lock(const Aim& aim, int top_level, double t, SelectorState& state)
{
    if (aim.id < 0) {
        return;
    }
    if (top_level == 0) {
        state.lock_id = -1;
        state.top0_lock_id = aim.id;
    } else {
        state.lock_id = aim.id;
        state.top0_lock_id = -1;
    }
    state.last_lock_t = t;
}

void AutoaimGenerator::update_outpost_lock(const Aim& aim, double t, SelectorState& state)
{
    if (aim.id < 0) {
        return;
    }
    state.lock_id = aim.id;
    state.last_lock_t = t;
}

int AutoaimGenerator::update_top_level(int level, double vyaw)
{
    double abs_vyaw = std::abs(vyaw);
    if (level == 0) {
        return abs_vyaw >= angles::from_degrees(120.0) ? 1 : 0;
    }
    if (level == 1) {
        if (abs_vyaw >= angles::from_degrees(900.0)) {
            return 2;
        }
        if (abs_vyaw < angles::from_degrees(80.0)) {
            return 0;
        }
        return 1;
    }
    return abs_vyaw < angles::from_degrees(850.0) ? 1 : 2;
}

Report AutoaimGenerator::run_simple_tracker() const
{
    Report report;
    report.tracker = "SimpleTracker";
    report.mode = describe_mode(cfg_.standard_mode, cfg_.preset);
    double bullet_speed = resolved_bullet_speed(cfg_, false);
    double distance = resolved_distance(cfg_, "standard");

    SimpleTracker tracker;
    tracker.set_params(tracker_params_.simple);

    std::vector<CommandRow> rows;
    std::mt19937 rng(cfg_.seed + 11);
    double t = 0.0;
    int occlusion_left = 0;
    bool has_prev = false;
    bool has_future_prev = false;
    double prev_t = 0.0;
    double prev_yaw = 0.0;
    double prev_pitch = 0.0;
    double prev_future_t = 0.0;
    double prev_future_yaw = 0.0;
    double prev_future_pitch = 0.0;

    while (t <= cfg_.duration + 1e-9) {
        bool stutter = false;
        double dt = compute_dt(rng, cfg_, stutter);
        bool occluded = is_occluded_frame(rng, cfg_, occlusion_left);
        double obs_t = std::max(0.0, t - cfg_.image_delay_ms * 0.001);
        Aim truth = real_generator_.make_standard_truth(obs_t, false);
        std::vector<Observation> obs_list;
        if (!occluded) {
            Observation obs;
            obs.t = t;
            obs.id = truth.id;
            obs.type = truth.type;
            obs.pos = truth.aim_pos + Eigen::Vector3d(
                gauss(rng, cfg_.pos_xy_sigma),
                gauss(rng, cfg_.pos_xy_sigma),
                gauss(rng, cfg_.pos_z_sigma));
            obs.orient_yaw = angles::normalize_angle(truth.orient_yaw + gauss(rng, angles::from_degrees(cfg_.yaw_sigma_deg)));
            obs_list.push_back(obs);
        }

        tracker.update(obs_list, t);
        double pred_gimbal_yaw = gimbal_yaw_at(std::max(0.0, t - cfg_.imu_delay_ms * 0.001), angles::from_degrees(cfg_.imu_yaw_bias_deg));
        double truth_gimbal_yaw = gimbal_yaw_at(t, 0.0);

        std::vector<Aim> aims = tracker.get_aims(t);
        Aim current_hit = aims.empty() ? Aim {} : aims.front();
        CommandResult predicted;
        predicted.hit = current_hit;
        if (predicted.hit.id >= 0) {
            Trajectory traj(predicted.hit.aim_pos, bullet_speed, pred_gimbal_yaw);
            predicted.fly_time = traj.solvable() ? traj.get_flyTime() : 0.0;
            predicted.solvable = traj.solvable();
            predicted.yaw_deg = angles::to_degrees(traj.solvable() ? traj.get_yaw() : command_yaw(predicted.hit.aim_pos));
            predicted.pitch_deg = angles::to_degrees(traj.solvable() ? traj.get_pitch() : command_pitch(predicted.hit.aim_pos));
        }

        Aim truth_now = real_generator_.make_standard_truth(t, false);
        CommandResult truth_cmd;
        truth_cmd.hit = truth_now;
        Trajectory truth_traj(truth_now.aim_pos, bullet_speed, truth_gimbal_yaw);
        truth_cmd.fly_time = truth_traj.solvable() ? truth_traj.get_flyTime() : 0.0;
        truth_cmd.solvable = truth_traj.solvable();
        truth_cmd.yaw_deg = angles::to_degrees(truth_traj.solvable() ? truth_traj.get_yaw() : command_yaw(truth_now.aim_pos));
        truth_cmd.pitch_deg = angles::to_degrees(truth_traj.solvable() ? truth_traj.get_pitch() : command_pitch(truth_now.aim_pos));

        update_current_metrics(report, predicted, truth_cmd, t, has_prev, prev_t, prev_yaw, prev_pitch);
        Aim predicted_future = predicted.hit.id >= 0 ? tracker.find_aim(0.08, predicted.hit.id) : Aim {};
        Aim truth_future = real_generator_.make_standard_truth(t + 0.08, false);
        update_future_metrics(report, predicted_future, truth_future, t, has_future_prev, prev_future_t, prev_future_yaw, prev_future_pitch);
        CommandResult raw_cmd = obs_list.empty() ? CommandResult {} : raw_observation_command(obs_list.front(), bullet_speed, pred_gimbal_yaw);

        CommandRow row;
        row.t = t;
        row.dt_ms = dt * 1000.0;
        row.stutter = stutter;
        row.occluded = occluded;
        row.raw_obs = static_cast<int>(obs_list.size());
        row.processed_obs = static_cast<int>(obs_list.size());
        row.selected_id = predicted.hit.id;
        row.raw_target_yaw_deg = raw_cmd.yaw_deg;
        row.target_yaw_deg = predicted.yaw_deg;
        row.truth_target_yaw_deg = truth_cmd.yaw_deg;
        row.raw_target_pitch_deg = raw_cmd.pitch_deg;
        row.target_pitch_deg = predicted.pitch_deg;
        row.truth_target_pitch_deg = truth_pitch_deg(truth_now, bullet_speed, truth_gimbal_yaw);
        row.center_speed = predicted.hit.center_vel.norm();
        rows.push_back(row);

        report.occluded_frames += occluded ? 1 : 0;
        report.stutter_frames += stutter ? 1 : 0;
        t += dt;
    }

    report.csv_path = build_csv_path(cfg_, "simple", cfg_.standard_mode, distance, bullet_speed);
    write_csv(report.csv_path, rows);
    return report;
}

Report AutoaimGenerator::run_singer_tracker() const
{
    Report report;
    report.tracker = "SingerTracker";
    report.mode = describe_mode(cfg_.standard_mode, cfg_.preset);
    double bullet_speed = resolved_bullet_speed(cfg_, false);
    double distance = resolved_distance(cfg_, "standard");

    SingerTracker tracker;
    tracker.set_params(tracker_params_.singer);

    std::vector<CommandRow> rows;
    std::mt19937 rng(cfg_.seed + 23);
    double t = 0.0;
    int occlusion_left = 0;
    bool has_prev = false;
    bool has_future_prev = false;
    double prev_t = 0.0;
    double prev_yaw = 0.0;
    double prev_pitch = 0.0;
    double prev_future_t = 0.0;
    double prev_future_yaw = 0.0;
    double prev_future_pitch = 0.0;

    while (t <= cfg_.duration + 1e-9) {
        bool stutter = false;
        double dt = compute_dt(rng, cfg_, stutter);
        bool occluded = is_occluded_frame(rng, cfg_, occlusion_left);
        double obs_t = std::max(0.0, t - cfg_.image_delay_ms * 0.001);
        Aim truth = real_generator_.make_standard_truth(obs_t, true);
        std::vector<Observation> obs_list;
        if (!occluded) {
            Observation obs;
            obs.t = t;
            obs.id = truth.id;
            obs.type = truth.type;
            obs.pos = truth.aim_pos + Eigen::Vector3d(
                gauss(rng, cfg_.pos_xy_sigma * 1.2),
                gauss(rng, cfg_.pos_xy_sigma * 1.2),
                gauss(rng, cfg_.pos_z_sigma * 1.2));
            obs.orient_yaw = angles::normalize_angle(truth.orient_yaw + gauss(rng, angles::from_degrees(cfg_.yaw_sigma_deg * 1.1)));
            obs_list.push_back(obs);
        }

        tracker.update(obs_list, t);
        double pred_gimbal_yaw = gimbal_yaw_at(std::max(0.0, t - cfg_.imu_delay_ms * 0.001), angles::from_degrees(cfg_.imu_yaw_bias_deg));
        double truth_gimbal_yaw = gimbal_yaw_at(t, 0.0);

        std::vector<Aim> aims = tracker.get_aims(t);
        Aim current_hit = aims.empty() ? Aim {} : aims.front();
        CommandResult predicted;
        predicted.hit = current_hit;
        if (predicted.hit.id >= 0) {
            Trajectory traj(predicted.hit.aim_pos, bullet_speed, pred_gimbal_yaw);
            predicted.fly_time = traj.solvable() ? traj.get_flyTime() : 0.0;
            predicted.solvable = traj.solvable();
            predicted.yaw_deg = angles::to_degrees(traj.solvable() ? traj.get_yaw() : command_yaw(predicted.hit.aim_pos));
            predicted.pitch_deg = angles::to_degrees(traj.solvable() ? traj.get_pitch() : command_pitch(predicted.hit.aim_pos));
        }

        Aim truth_now = real_generator_.make_standard_truth(t, true);
        CommandResult truth_cmd;
        truth_cmd.hit = truth_now;
        Trajectory truth_traj(truth_now.aim_pos, bullet_speed, truth_gimbal_yaw);
        truth_cmd.fly_time = truth_traj.solvable() ? truth_traj.get_flyTime() : 0.0;
        truth_cmd.solvable = truth_traj.solvable();
        truth_cmd.yaw_deg = angles::to_degrees(truth_traj.solvable() ? truth_traj.get_yaw() : command_yaw(truth_now.aim_pos));
        truth_cmd.pitch_deg = angles::to_degrees(truth_traj.solvable() ? truth_traj.get_pitch() : command_pitch(truth_now.aim_pos));

        update_current_metrics(report, predicted, truth_cmd, t, has_prev, prev_t, prev_yaw, prev_pitch);
        Aim predicted_future = predicted.hit.id >= 0 ? tracker.find_aim(0.08, predicted.hit.id) : Aim {};
        Aim truth_future = real_generator_.make_standard_truth(t + 0.08, true);
        update_future_metrics(report, predicted_future, truth_future, t, has_future_prev, prev_future_t, prev_future_yaw, prev_future_pitch);
        CommandResult raw_cmd = obs_list.empty() ? CommandResult {} : raw_observation_command(obs_list.front(), bullet_speed, pred_gimbal_yaw);

        CommandRow row;
        row.t = t;
        row.dt_ms = dt * 1000.0;
        row.stutter = stutter;
        row.occluded = occluded;
        row.raw_obs = static_cast<int>(obs_list.size());
        row.processed_obs = static_cast<int>(obs_list.size());
        row.selected_id = predicted.hit.id;
        row.raw_target_yaw_deg = raw_cmd.yaw_deg;
        row.target_yaw_deg = predicted.yaw_deg;
        row.truth_target_yaw_deg = truth_cmd.yaw_deg;
        row.raw_target_pitch_deg = raw_cmd.pitch_deg;
        row.target_pitch_deg = predicted.pitch_deg;
        row.truth_target_pitch_deg = truth_pitch_deg(truth_now, bullet_speed, truth_gimbal_yaw);
        row.center_speed = predicted.hit.center_vel.norm();
        rows.push_back(row);

        report.occluded_frames += occluded ? 1 : 0;
        report.stutter_frames += stutter ? 1 : 0;
        t += dt;
    }

    report.csv_path = build_csv_path(cfg_, "singer", cfg_.standard_mode, distance, bullet_speed);
    write_csv(report.csv_path, rows);
    return report;
}

Report AutoaimGenerator::run_top_tracker() const
{
    Report report;
    report.tracker = "TopTracker";
    report.mode = describe_mode(cfg_.top_mode, cfg_.preset);
    double bullet_speed = resolved_bullet_speed(cfg_, false);
    double distance = resolved_distance(cfg_, "top");

    TopTracker tracker;
    tracker.set_params(tracker_params_.top);
    tracker.set_tracking_type(1);

    CameraParam cam = make_camera();
    tracker.set_camera_info(cam);

    std::vector<CommandRow> rows;
    std::mt19937 rng(cfg_.seed + 37);
    double t = 0.0;
    int occlusion_left = 0;
    int truth_top_level = 0;
    SelectorState pred_state;
    SelectorState truth_state;
    bool has_prev = false;
    bool has_future_prev = false;
    double prev_t = 0.0;
    double prev_yaw = 0.0;
    double prev_pitch = 0.0;
    double prev_future_t = 0.0;
    double prev_future_yaw = 0.0;
    double prev_future_pitch = 0.0;
    double gimbal_yaw = 0.0;
    double gimbal_pitch = 0.0;
    double gimbal_cmd_yaw = 0.0;
    double gimbal_cmd_pitch = 0.0;
    std::vector<GimbalSample> gimbal_history = {{0.0, 0.0, 0.0}};

    while (t <= cfg_.duration + 1e-9) {
        gimbal_yaw = gimbal_cmd_yaw;
        gimbal_pitch = gimbal_cmd_pitch;
        record_gimbal_sample(gimbal_history, t, gimbal_yaw, gimbal_pitch);
        bool stutter = false;
        double dt = compute_dt(rng, cfg_, stutter);
        bool occluded = is_occluded_frame(rng, cfg_, occlusion_left);
        double obs_t = std::max(0.0, t - cfg_.image_delay_ms * 0.001);
        double truth_vyaw_obs = 0.0;
        std::vector<Aim> truth_aims_obs = real_generator_.make_top_truth(obs_t, truth_vyaw_obs);
        GimbalSample obs_gimbal = sample_gimbal(gimbal_history, obs_t);
        GimbalSample truth_gimbal = sample_gimbal(gimbal_history, t);
        GimbalSample imu_gimbal = sample_gimbal(gimbal_history, std::max(0.0, t - cfg_.imu_delay_ms * 0.001));
        PoseSample true_pose = real_generator_.camera_pose_at(obs_t, obs_gimbal.yaw, obs_gimbal.pitch);
        PoseSample truth_select_pose = real_generator_.camera_pose_at(t, truth_gimbal.yaw, truth_gimbal.pitch);
        double pred_gimbal_yaw = imu_gimbal.yaw + angles::from_degrees(cfg_.imu_yaw_bias_deg);
        double truth_gimbal_yaw = truth_gimbal.yaw;
        PoseSample measured_pose = real_generator_.camera_pose_at(
            std::max(0.0, t - cfg_.imu_delay_ms * 0.001),
            imu_gimbal.yaw + angles::from_degrees(cfg_.imu_yaw_bias_deg),
            imu_gimbal.pitch + angles::from_degrees(cfg_.imu_pitch_bias_deg));
        tracker.set_pose(measured_pose.R_c_w, measured_pose.t_c_w);

        std::vector<Observation> raw_obs;
        if (!occluded) {
            raw_obs = real_generator_.make_rotating_observations(truth_aims_obs, t, cam, true_pose, rng, 2, -0.26);
        }

        std::vector<Observation> processed_obs = raw_obs;
        if (tracker.get_credit(t) && !raw_obs.empty()) {
            processed_obs = tracker.fit(raw_obs, t);
            std::vector<Observation> assoc_obs = processed_obs;
            for (auto& obs: assoc_obs) {
                obs.orient_yaw = obs.fitted_yaw;
            }
            auto rotating_assoc = RealGenerator::associate_rotating_armors(assoc_obs, tracker.get_base_yaw(t), 4);
            for (size_t i = 0; i < rotating_assoc.size(); i++) {
                auto [id, yaw_offset, cont_yaw] = rotating_assoc[i];
                (void)yaw_offset;
                processed_obs[i].id = id;
                processed_obs[i].orient_yaw = cont_yaw;
                processed_obs[i].fitted_yaw = cont_yaw;
            }
        }
        if (!processed_obs.empty()) {
            tracker.update(processed_obs, t);
        }

        double truth_vyaw_now = 0.0;
        real_generator_.make_top_truth(t, truth_vyaw_now);
        truth_top_level = update_top_level(truth_top_level, truth_vyaw_now);
        auto pred_select_at = [&](double query_t) {
            return select_standard_aim(tracker.get_aims(query_t), tracker.get_vyaw(), tracker.get_top_level(), t, 1, measured_pose.t_c_w, pred_state);
        };
        auto truth_select_at = [&](double query_t) {
            double future_vyaw = 0.0;
            std::vector<Aim> future_truth = real_generator_.make_top_truth(query_t, future_vyaw);
            int future_top_level = update_top_level(truth_top_level, future_vyaw);
            return select_standard_aim(future_truth, future_vyaw, future_top_level, t, 1, truth_select_pose.t_c_w, truth_state);
        };

        CommandResult predicted = solve_command(t, bullet_speed, cfg_.latency_ms * 0.001, pred_gimbal_yaw, pred_select_at);
        CommandResult truth_cmd = solve_command(t, bullet_speed, cfg_.latency_ms * 0.001, truth_gimbal_yaw, truth_select_at);
        CommandResult truth_metric = truth_cmd;
        truth_metric.pitch_deg = truth_pitch_deg(truth_cmd.hit, bullet_speed, truth_gimbal_yaw);
        update_standard_lock(predicted.hit, tracker.get_top_level(), t, pred_state);
        update_standard_lock(truth_cmd.hit, truth_top_level, t, truth_state);
        latch_gimbal_command(predicted, gimbal_cmd_yaw, gimbal_cmd_pitch);
        update_current_metrics(report, predicted, truth_metric, t, has_prev, prev_t, prev_yaw, prev_pitch);

        Aim predicted_future = predicted.hit.id >= 0 ? tracker.find_aim(0.08, predicted.hit.id) : Aim {};
        double unused_vyaw = 0.0;
        Aim truth_future = predicted.hit.id >= 0 ? real_generator_.make_top_truth(t + 0.08, unused_vyaw)[static_cast<size_t>(predicted.hit.id)] : Aim {};
        update_future_metrics(report, predicted_future, truth_future, t, has_future_prev, prev_future_t, prev_future_yaw, prev_future_pitch);
        const Observation* raw_selected_obs = find_selected_observation(raw_obs, processed_obs, predicted.hit.id);
        CommandResult raw_cmd = raw_selected_obs == nullptr ? CommandResult {} : raw_observation_command(*raw_selected_obs, bullet_speed, pred_gimbal_yaw);

        CommandRow row;
        row.t = t;
        row.dt_ms = dt * 1000.0;
        row.stutter = stutter;
        row.occluded = occluded;
        row.raw_obs = static_cast<int>(raw_obs.size());
        row.processed_obs = static_cast<int>(processed_obs.size());
        row.selected_id = predicted.hit.id;
        row.top_level = tracker.get_top_level();
        row.raw_target_yaw_deg = raw_cmd.yaw_deg;
        row.target_yaw_deg = predicted.yaw_deg;
        row.truth_target_yaw_deg = truth_cmd.yaw_deg;
        row.raw_target_pitch_deg = raw_cmd.pitch_deg;
        row.target_pitch_deg = predicted.pitch_deg;
        row.truth_target_pitch_deg = truth_metric.pitch_deg;
        row.center_speed = predicted.hit.center_vel.norm();
        row.reported_vyaw_deg_s = angles::to_degrees(tracker.get_vyaw());
        rows.push_back(row);

        report.occluded_frames += occluded ? 1 : 0;
        report.stutter_frames += stutter ? 1 : 0;
        report.max_vyaw_deg_s = std::max(report.max_vyaw_deg_s, std::abs(angles::to_degrees(tracker.get_vyaw())));
        t += dt;
    }

    report.csv_path = build_csv_path(cfg_, "top", cfg_.top_mode, distance, bullet_speed);
    write_csv(report.csv_path, rows);
    return report;
}

Report AutoaimGenerator::run_top3_tracker() const
{
    Report report;
    report.tracker = "Top3Tracker";
    report.mode = describe_mode(cfg_.outpost_mode, cfg_.preset);
    double bullet_speed = resolved_bullet_speed(cfg_, true);
    double distance = resolved_distance(cfg_, "outpost");

    Top3Tracker tracker;
    tracker.set_params(tracker_params_.top3);

    CameraParam cam = make_camera();
    tracker.set_camera_info(cam);

    std::vector<CommandRow> rows;
    std::mt19937 rng(cfg_.seed + 53);
    double t = 0.0;
    int occlusion_left = 0;
    SelectorState pred_state;
    SelectorState truth_state;
    bool has_prev = false;
    bool has_future_prev = false;
    double prev_t = 0.0;
    double prev_yaw = 0.0;
    double prev_pitch = 0.0;
    double prev_future_t = 0.0;
    double prev_future_yaw = 0.0;
    double prev_future_pitch = 0.0;
    double gimbal_yaw = 0.0;
    double gimbal_pitch = 0.0;
    double gimbal_cmd_yaw = 0.0;
    double gimbal_cmd_pitch = 0.0;
    std::vector<GimbalSample> gimbal_history = {{0.0, 0.0, 0.0}};

    while (t <= cfg_.duration + 1e-9) {
        gimbal_yaw = gimbal_cmd_yaw;
        gimbal_pitch = gimbal_cmd_pitch;
        record_gimbal_sample(gimbal_history, t, gimbal_yaw, gimbal_pitch);
        bool stutter = false;
        double dt = compute_dt(rng, cfg_, stutter);
        bool occluded = is_occluded_frame(rng, cfg_, occlusion_left);
        double obs_t = std::max(0.0, t - cfg_.image_delay_ms * 0.001);
        double truth_vyaw_obs = 0.0;
        std::vector<Aim> truth_aims_obs = real_generator_.make_top3_truth(obs_t, truth_vyaw_obs);
        GimbalSample obs_gimbal = sample_gimbal(gimbal_history, obs_t);
        GimbalSample truth_gimbal = sample_gimbal(gimbal_history, t);
        GimbalSample imu_gimbal = sample_gimbal(gimbal_history, std::max(0.0, t - cfg_.imu_delay_ms * 0.001));
        PoseSample true_pose = real_generator_.camera_pose_at(obs_t, obs_gimbal.yaw, obs_gimbal.pitch);
        PoseSample truth_select_pose = real_generator_.camera_pose_at(t, truth_gimbal.yaw, truth_gimbal.pitch);
        double pred_gimbal_yaw = imu_gimbal.yaw + angles::from_degrees(cfg_.imu_yaw_bias_deg);
        double truth_gimbal_yaw = truth_gimbal.yaw;
        PoseSample measured_pose = real_generator_.camera_pose_at(
            std::max(0.0, t - cfg_.imu_delay_ms * 0.001),
            imu_gimbal.yaw + angles::from_degrees(cfg_.imu_yaw_bias_deg),
            imu_gimbal.pitch + angles::from_degrees(cfg_.imu_pitch_bias_deg));
        tracker.set_pose(measured_pose.R_c_w, measured_pose.t_c_w);

        std::vector<Observation> raw_obs;
        if (!occluded) {
            raw_obs = real_generator_.make_rotating_observations(truth_aims_obs, t, cam, true_pose, rng, 3, -0.2617993877991494);
        }

        std::vector<Observation> processed_obs = raw_obs;
        if (tracker.get_credit(t) && !raw_obs.empty()) {
            processed_obs = tracker.fit(raw_obs, t);
            std::vector<Observation> assoc_obs = processed_obs;
            for (auto& obs: assoc_obs) {
                obs.orient_yaw = obs.fitted_yaw;
            }
            auto rotating_assoc = RealGenerator::associate_rotating_armors(assoc_obs, tracker.get_base_yaw(t), 3);
            for (size_t i = 0; i < rotating_assoc.size(); i++) {
                auto [id, yaw_offset, cont_yaw] = rotating_assoc[i];
                (void)yaw_offset;
                processed_obs[i].id = id;
                processed_obs[i].orient_yaw = cont_yaw;
                processed_obs[i].fitted_yaw = cont_yaw;
            }
        }
        if (!processed_obs.empty()) {
            tracker.update(processed_obs, t);
        }

        auto pred_select_at = [&](double query_t) {
            return select_outpost_aim(tracker.get_aims(query_t), tracker.get_vyaw(), t, measured_pose.t_c_w, pred_state);
        };
        auto truth_select_at = [&](double query_t) {
            double future_vyaw = 0.0;
            return select_outpost_aim(real_generator_.make_top3_truth(query_t, future_vyaw), future_vyaw, t, truth_select_pose.t_c_w, truth_state);
        };

        CommandResult predicted = solve_command(t, bullet_speed, cfg_.latency_ms * 0.001, pred_gimbal_yaw, pred_select_at);
        CommandResult truth_cmd = solve_command(t, bullet_speed, cfg_.latency_ms * 0.001, truth_gimbal_yaw, truth_select_at);
        CommandResult truth_metric = truth_cmd;
        truth_metric.pitch_deg = truth_pitch_deg(truth_cmd.hit, bullet_speed, truth_gimbal_yaw);
        update_outpost_lock(predicted.hit, t, pred_state);
        update_outpost_lock(truth_cmd.hit, t, truth_state);
        latch_gimbal_command(predicted, gimbal_cmd_yaw, gimbal_cmd_pitch);
        update_current_metrics(report, predicted, truth_metric, t, has_prev, prev_t, prev_yaw, prev_pitch);

        Aim predicted_future = predicted.hit.id >= 0 ? tracker.find_aim(0.08, predicted.hit.id) : Aim {};
        double unused_vyaw = 0.0;
        Aim truth_future = predicted.hit.id >= 0 ? real_generator_.make_top3_truth(t + 0.08, unused_vyaw)[static_cast<size_t>(predicted.hit.id)] : Aim {};
        update_future_metrics(report, predicted_future, truth_future, t, has_future_prev, prev_future_t, prev_future_yaw, prev_future_pitch);
        const Observation* raw_selected_obs = find_selected_observation(raw_obs, processed_obs, predicted.hit.id);
        CommandResult raw_cmd = raw_selected_obs == nullptr ? CommandResult {} : raw_observation_command(*raw_selected_obs, bullet_speed, pred_gimbal_yaw);

        CommandRow row;
        row.t = t;
        row.dt_ms = dt * 1000.0;
        row.stutter = stutter;
        row.occluded = occluded;
        row.raw_obs = static_cast<int>(raw_obs.size());
        row.processed_obs = static_cast<int>(processed_obs.size());
        row.selected_id = predicted.hit.id;
        row.raw_target_yaw_deg = raw_cmd.yaw_deg;
        row.target_yaw_deg = predicted.yaw_deg;
        row.truth_target_yaw_deg = truth_cmd.yaw_deg;
        row.raw_target_pitch_deg = raw_cmd.pitch_deg;
        row.target_pitch_deg = predicted.pitch_deg;
        row.truth_target_pitch_deg = truth_metric.pitch_deg;
        row.center_speed = predicted.hit.center_vel.norm();
        row.reported_vyaw_deg_s = angles::to_degrees(tracker.get_vyaw());
        rows.push_back(row);

        report.occluded_frames += occluded ? 1 : 0;
        report.stutter_frames += stutter ? 1 : 0;
        report.max_vyaw_deg_s = std::max(report.max_vyaw_deg_s, std::abs(angles::to_degrees(tracker.get_vyaw())));
        t += dt;
    }

    report.csv_path = build_csv_path(cfg_, "top3", cfg_.outpost_mode, distance, bullet_speed);
    write_csv(report.csv_path, rows);
    return report;
}

} // namespace helios_cv
