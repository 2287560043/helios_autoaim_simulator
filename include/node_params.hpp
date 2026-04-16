#pragma once

#include "armor_predictor/tracker/SimpleTracker.hpp"
#include "armor_predictor/tracker/SingerTracker.hpp"
#include "autoaim_utilities/PredictorTypes.hpp"

#include <string>

namespace helios_cv
{

struct TrackerParamSet {
    SimpleTracker::Params simple;
    SingerTracker::Params singer;
    DDMParams top;
    DCMParams top3;
};

std::string find_default_params_file();
TrackerParamSet load_tracker_params(const std::string& path);

} // namespace helios_cv
