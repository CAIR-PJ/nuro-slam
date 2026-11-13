#pragma once

#include <string>

struct LMSettings
{
    // Maximum number of iterations allowed
    int max_num_iter;
    // Tolerance in the cost function to claim convergence
    double tol_fun;
    // % Number of consecutive times that tolerance must be satisfied to claim convergence
    int num_times_tol_fun_sat;

    // verb: 0 = no output
    //       1 = only print optimization process
    //       2 = print optimization process + show evolution
    //       3 = 2 + save evolution
    int verb;
};

// Measurement Model: LEGM, OEGM
enum MeasModel {
    LEGM_MODEL,
    OEGM_MODEL
};

struct BASettings
{
    // Contrast threshold
    double C_th;

    // If double-layer BA is enabled?
    bool double_layer;

    // If the initial map is not available? Start from sctrach?
    bool init_map_available;

    // Use the conjugate gradient (CG) solver?
    bool use_CG;

    // Use robust cost function (IRLS)?
    bool use_IRLS;

    // Cost funtion type: quadratic || huber || cauchy
    std::string cost_type;

    // The parameter of the Huber and Cauchy cost functions
    double a;

    // Share a common pose for a event batch
    int event_batch_size;

    // Event sample rate
    int event_sampling_rate;

    // Map size
    int pano_height, pano_width;

    // If groundtruth poses are available? For visualization.
    bool groundtruth_available;

    // Size of the time window (sec)
    double time_window_size;

    // Stride of sliding window (sec)
    double sliding_window_stride;

    // Time gap between two control points (knots)
    double dt_knots;
};
