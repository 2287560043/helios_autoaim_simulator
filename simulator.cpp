#include "./include/autoaim_generator.hpp"

namespace helios_cv
{

void print_help()
{
    std::cout
        << "Usage: simulator [options]\n"
        << "  --tracker all|simple|singer|top|top3\n"
        << "  --preset default|clean|light_disturb|medium_disturb|heavy_disturb|extreme_disturb\n"
        << "  --standard-mode translate_const|translate_var\n"
        << "  --top-mode spin_const|spin_var|spin_const_translate_const|spin_var_translate_var|spin_const_height_var|spin_var_height_var\n"
        << "  --outpost-mode outpost_standard\n"
        << "  --fps <value>\n"
        << "  --duration <seconds>\n"
        << "  --seed <int>\n"
        << "  --image-delay-ms <value>\n"
        << "  --imu-delay-ms <value>\n"
        << "  --imu-yaw-bias-deg <value>\n"
        << "  --imu-pitch-bias-deg <value>\n"
        << "  --drop-prob <value>\n"
        << "  --drop-min-frames <int>\n"
        << "  --drop-max-frames <int>\n"
        << "  --stutter-prob <value>\n"
        << "  --stutter-mult <value>\n"
        << "  --dt-jitter-ratio <value>\n"
        << "  --pos-xy-sigma <m>\n"
        << "  --pos-z-sigma <m>\n"
        << "  --yaw-sigma-deg <value>\n"
        << "  --corner-sigma <pixel>\n"
        << "  --distance-m <value>\n"
        << "  --bullet-speed <m/s>\n"
        << "  --latency-ms <value>\n"
        << "  --params-file <path>\n"
        << "  --output-dir <path>\n"
        << "  --no-plot\n"
        << "Target Presets:\n"
        << "  translate_const: 左右匀速平移 1.0m/s\n"
        << "  translate_var: 左右变速平移 -2.0 -> 2.0 -> -2.0m/s\n"
        << "  spin_const: 匀速陀螺 60rpm (360deg/s)\n"
        << "  spin_var: 变速陀螺 20 -> 120 -> 60rpm\n"
        << "  spin_const_translate_const: 匀速陀螺 60rpm + 左右匀速平移 0.8m/s\n"
        << "  spin_var_translate_var: 变速陀螺 30 -> 120 -> 60rpm + 左右变速平移 -1.5 -> 1.5 -> -1.5m/s\n"
        << "  spin_const_height_var: 匀速陀螺 60rpm + 原地高度变化 0 -> 50cm -> 0 (周期5s)\n"
        << "  spin_var_height_var: 变速陀螺 20 -> 120 -> 60rpm + 原地高度变化 0 -> 40cm -> 0 (周期5s)\n"
        << "  outpost_standard: 前哨站 0.8pi rad/s 附近轻微波动\n"
        << "Disturb Presets:\n"
        << "  clean: 低噪声、无丢帧、无卡顿、自车扰动极小\n"
        << "  light_disturb: 轻度噪声、轻度丢帧和小幅溜车\n"
        << "  medium_disturb: 常规噪声、遮挡卡顿和明显溜车\n"
        << "  heavy_disturb: 更重噪声、丢包遮挡卡顿和较大溜车\n"
        << "  extreme_disturb: 极端综合干扰\n";
}

void load_preset(SimConfig& cfg)
{
    cfg.preset = canonical_preset(cfg.preset);
    if (cfg.preset == "default") {
        return;
    }
    const SimPreset* preset = find_sim_preset(cfg.preset);
    if (preset == nullptr) {
        throw std::runtime_error("unknown preset: " + cfg.preset);
    }
    apply_sim_preset(cfg, *preset);
}

SimConfig parse_args(int argc, char** argv)
{
    SimConfig cfg;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            cfg.help = true;
            return cfg;
        }
        if (arg == "--preset") {
            if (i + 1 >= argc) {
                throw std::runtime_error("missing value for --preset");
            }
            cfg.preset = argv[++i];
        }
    }
    load_preset(cfg);

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        auto next = [&](const std::string& name) {
            if (i + 1 >= argc) {
                throw std::runtime_error("missing value for " + name);
            }
            return std::string(argv[++i]);
        };

        if (arg == "--help" || arg == "-h") continue;
        if (arg == "--preset") next(arg);
        else if (arg == "--tracker") cfg.tracker = next(arg);
        else if (arg == "--standard-mode") cfg.standard_mode = next(arg);
        else if (arg == "--top-mode") cfg.top_mode = next(arg);
        else if (arg == "--outpost-mode") cfg.outpost_mode = next(arg);
        else if (arg == "--output-dir") cfg.output_dir = next(arg);
        else if (arg == "--fps") cfg.fps = std::stod(next(arg));
        else if (arg == "--duration") cfg.duration = std::stod(next(arg));
        else if (arg == "--seed") cfg.seed = std::stoi(next(arg));
        else if (arg == "--image-delay-ms") cfg.image_delay_ms = std::stod(next(arg));
        else if (arg == "--imu-delay-ms") cfg.imu_delay_ms = std::stod(next(arg));
        else if (arg == "--imu-yaw-bias-deg") cfg.imu_yaw_bias_deg = std::stod(next(arg));
        else if (arg == "--imu-pitch-bias-deg") cfg.imu_pitch_bias_deg = std::stod(next(arg));
        else if (arg == "--drop-prob") cfg.drop_prob = std::stod(next(arg));
        else if (arg == "--drop-min-frames") cfg.drop_min_frames = std::stoi(next(arg));
        else if (arg == "--drop-max-frames") cfg.drop_max_frames = std::stoi(next(arg));
        else if (arg == "--stutter-prob") cfg.stutter_prob = std::stod(next(arg));
        else if (arg == "--stutter-mult") cfg.stutter_mult = std::stod(next(arg));
        else if (arg == "--dt-jitter-ratio") cfg.dt_jitter_ratio = std::stod(next(arg));
        else if (arg == "--pos-xy-sigma") cfg.pos_xy_sigma = std::stod(next(arg));
        else if (arg == "--pos-z-sigma") cfg.pos_z_sigma = std::stod(next(arg));
        else if (arg == "--yaw-sigma-deg") cfg.yaw_sigma_deg = std::stod(next(arg));
        else if (arg == "--corner-sigma") cfg.corner_sigma = std::stod(next(arg));
        else if (arg == "--distance-m") cfg.distance_m = std::stod(next(arg));
        else if (arg == "--bullet-speed") cfg.bullet_speed = std::stod(next(arg));
        else if (arg == "--latency-ms") cfg.latency_ms = std::stod(next(arg));
        else if (arg == "--params-file") cfg.params_file = next(arg);
        else if (arg == "--no-plot") cfg.no_plot = true;
        else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }

    cfg.fps = std::max(cfg.fps, 1.0);
    cfg.duration = std::max(cfg.duration, 0.1);
    cfg.drop_min_frames = std::max(cfg.drop_min_frames, 1);
    cfg.drop_max_frames = std::max(cfg.drop_max_frames, cfg.drop_min_frames);
    cfg.stutter_mult = std::max(cfg.stutter_mult, 1.0);
    cfg.preset = canonical_preset(cfg.preset);
    cfg.standard_mode = canonical_standard_mode(cfg.standard_mode);
    cfg.top_mode = canonical_top_mode(cfg.top_mode);
    cfg.outpost_mode = canonical_outpost_mode(cfg.outpost_mode);
    return cfg;
}

} // namespace helios_cv

int main(int argc, char** argv)
{
    using namespace helios_cv;

    try {
        SimConfig cfg = parse_args(argc, argv);
        if (cfg.help) {
            print_help();
            return 0;
        }

        std::filesystem::create_directories(cfg.output_dir);
        AutoaimGenerator generator(cfg);

        std::vector<Report> reports;
        if (cfg.tracker == "all" || cfg.tracker == "simple") {
            std::cerr << "running SimpleTracker..." << std::endl;
            reports.push_back(generator.run_simple_tracker());
        }
        if (cfg.tracker == "all" || cfg.tracker == "singer") {
            std::cerr << "running SingerTracker..." << std::endl;
            reports.push_back(generator.run_singer_tracker());
        }
        if (cfg.tracker == "all" || cfg.tracker == "top") {
            std::cerr << "running TopTracker..." << std::endl;
            reports.push_back(generator.run_top_tracker());
        }
        if (cfg.tracker == "all" || cfg.tracker == "top3") {
            std::cerr << "running Top3Tracker..." << std::endl;
            reports.push_back(generator.run_top3_tracker());
        }

        if (reports.empty()) {
            throw std::runtime_error("no tracker selected");
        }

        bool all_pass = true;
        for (const auto& report: reports) {
            print_report(report);
            all_pass = all_pass && report.pass();
        }
        std::cout << "overall: " << (all_pass ? "PASS" : "FAIL") << "\n";
        // return all_pass ? 0 : 1;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "simulator error: " << e.what() << std::endl;
        return 2;
    }
}
