#ifndef COIN_BIEVR_INTENSITY_PROCESSING_H_
#define COIN_BIEVR_INTENSITY_PROCESSING_H_

#include "coin_bievr/common.h"

namespace coin_bievr {

struct IntensityProcessingConfig {
  int image_width = 1024;
  int image_height = 128;
  double vertical_fov_deg = 90.0;
  int brightness_window_width = 41;
  int brightness_window_height = 7;
  double output_scale = 140.0;
};

// Sparse-safe adaptation of the COIN-LIO brightness normalization. The spherical
// image is only used to define local neighborhoods; registration samples the
// voxel-wise 3D intensity maps directly.
void normalizeIntensities(const Pointcloud& points_L, const IntensityView& raw_intensities,
                          const IntensityProcessingConfig& config,
                          Intensities& filtered_intensities);

}  // namespace coin_bievr

#endif  // COIN_BIEVR_INTENSITY_PROCESSING_H_
