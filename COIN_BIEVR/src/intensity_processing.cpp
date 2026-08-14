#include "coin_bievr/intensity_processing.h"

#include <tbb/parallel_sort.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <tuple>
#include <vector>

namespace coin_bievr {

namespace {

struct ProjectedIntensity {
  int pixel = -1;
  size_t point_index = 0;
  double intensity = 0.0;
};

int projectToPixel(const Point& point, const IntensityProcessingConfig& config) {
  const double range = point.norm();
  if (!std::isfinite(range) || range <= 0.0) return -1;

  const double azimuth = std::atan2(point.y(), point.x());
  const double elevation = std::asin(std::clamp(point.z() / range, -1.0, 1.0));
  const double vertical_fov_rad = config.vertical_fov_deg * M_PI / 180.0;

  int x = static_cast<int>(std::floor(
      -static_cast<double>(config.image_width) * azimuth / (2.0 * M_PI) +
      0.5 * static_cast<double>(config.image_width)));
  int y = static_cast<int>(std::floor(
      -static_cast<double>(config.image_height) * elevation / vertical_fov_rad +
      0.5 * static_cast<double>(config.image_height)));

  x %= config.image_width;
  if (x < 0) x += config.image_width;
  if (y < 0 || y >= config.image_height) return -1;
  return y * config.image_width + x;
}

double rectangleSum(const std::vector<double>& integral, int stride, int x0, int y0, int x1,
                    int y1) {
  return integral[(y1 + 1) * stride + (x1 + 1)] - integral[y0 * stride + (x1 + 1)] -
         integral[(y1 + 1) * stride + x0] + integral[y0 * stride + x0];
}

}  // namespace

void normalizeIntensities(const Pointcloud& points_L, const IntensityView& raw_intensities,
                          const IntensityProcessingConfig& config,
                          Intensities& filtered_intensities) {
  filtered_intensities.resize(1, points_L.size());
  filtered_intensities.setZero();
  if (points_L.empty()) return;

  std::vector<ProjectedIntensity> projected(points_L.size());
  for (size_t i = 0; i < points_L.size(); ++i) {
    const double raw = raw_intensities(i);
    projected[i].pixel = projectToPixel(points_L[i], config);
    projected[i].point_index = i;
    projected[i].intensity = std::max(0.0, std::isfinite(raw) ? raw : 0.0);
  }

  tbb::parallel_sort(projected.begin(), projected.end(),
                     [](const ProjectedIntensity& a, const ProjectedIntensity& b) {
                       return std::tie(a.pixel, a.point_index) < std::tie(b.pixel, b.point_index);
                     });

  const int width = config.image_width;
  const int height = config.image_height;
  const int pixel_count = width * height;
  std::vector<double> image(pixel_count, 0.0);
  std::vector<double> occupied(pixel_count, 0.0);

  size_t begin = 0;
  while (begin < projected.size()) {
    const int pixel = projected[begin].pixel;
    size_t end = begin + 1;
    while (end < projected.size() && projected[end].pixel == pixel) ++end;
    if (pixel >= 0) {
      const double sum = std::accumulate(
          projected.begin() + begin, projected.begin() + end, 0.0,
          [](double value, const ProjectedIntensity& point) { return value + point.intensity; });
      image[pixel] = sum / static_cast<double>(end - begin);
      occupied[pixel] = 1.0;
    }
    begin = end;
  }

  // Duplicate the image horizontally so box-filter windows cross the azimuth
  // seam without special cases. Integral images make each sparse neighborhood
  // query constant time regardless of the configured window size.
  const int extended_width = 3 * width;
  const int integral_stride = extended_width + 1;
  std::vector<double> intensity_integral((height + 1) * integral_stride, 0.0);
  std::vector<double> occupancy_integral((height + 1) * integral_stride, 0.0);
  for (int y = 0; y < height; ++y) {
    double intensity_row_sum = 0.0;
    double occupancy_row_sum = 0.0;
    for (int x = 0; x < extended_width; ++x) {
      const int source_pixel = y * width + (x % width);
      intensity_row_sum += image[source_pixel];
      occupancy_row_sum += occupied[source_pixel];
      intensity_integral[(y + 1) * integral_stride + (x + 1)] =
          intensity_integral[y * integral_stride + (x + 1)] + intensity_row_sum;
      occupancy_integral[(y + 1) * integral_stride + (x + 1)] =
          occupancy_integral[y * integral_stride + (x + 1)] + occupancy_row_sum;
    }
  }

  const int half_width = config.brightness_window_width / 2;
  const int half_height = config.brightness_window_height / 2;
  std::vector<double> normalized_image(pixel_count, 0.0);
  for (int y = 0; y < height; ++y) {
    const int y0 = std::max(0, y - half_height);
    const int y1 = std::min(height - 1, y + half_height);
    for (int x = 0; x < width; ++x) {
      const int pixel = y * width + x;
      if (occupied[pixel] == 0.0) continue;

      const int center_x = x + width;
      const int x0 = center_x - half_width;
      const int x1 = center_x + half_width;
      const double count =
          rectangleSum(occupancy_integral, integral_stride, x0, y0, x1, y1);
      const double brightness =
          rectangleSum(intensity_integral, integral_stride, x0, y0, x1, y1) /
          std::max(1.0, count);
      normalized_image[pixel] =
          std::clamp(config.output_scale * image[pixel] / (brightness + 1.0), 0.0, 255.0);
    }
  }

  for (const auto& point : projected) {
    if (point.pixel >= 0) {
      filtered_intensities(point.point_index) = normalized_image[point.pixel];
    }
  }
}

}  // namespace coin_bievr
