// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef OPENSEEKTHERMAL_CAMERA_CALIBRATION_HPP
#define OPENSEEKTHERMAL_CAMERA_CALIBRATION_HPP

#include "dead_pixel_mask.hpp"
#include "temperature_calibration.hpp"
#include "vignette_correction.hpp"

#include <filesystem>
#include <optional>
#include <string>

namespace openseekthermal
{

/*!
 * Bundled per-unit calibration: temperature mapping, vignette polynomial, and
 * dead-pixel list. All three are optional; omitted sections leave the
 * corresponding correction disabled.
 *
 * Loaded from / written to a single INI file with `[temperature]`,
 * `[vignette]`, `[dead_pixels]` sections via loadCameraCalibration() /
 * saveCameraCalibration().
 */
struct CameraCalibration {
  std::optional<TemperatureCalibration> temperature;
  std::optional<VignetteCorrection> vignette;
  std::optional<DeadPixelMask> dead_pixels;
};

/*!
 * Load a unified calibration file. Empty / missing sections become
 * `std::nullopt`. If `[vignette]` or `[dead_pixels]` is present, its width /
 * height must match the camera's frame size.
 *
 * @param path Path to the .ini file.
 * @param expected_width Frame width the vignette / dead-pixel sections must match.
 * @param expected_height Frame height the vignette / dead-pixel sections must match.
 * @throws std::runtime_error on I/O, parse, or dimension errors.
 */
CameraCalibration loadCameraCalibration( const std::filesystem::path &path, int expected_width,
                                         int expected_height );

/*!
 * Write a unified calibration file in the format consumed by
 * loadCameraCalibration. Omitted (nullopt) sections are skipped. Existing
 * files at `path` are overwritten.
 */
void saveCameraCalibration( const std::filesystem::path &path, const CameraCalibration &cal,
                            const std::string &header_comment = "" );

} // namespace openseekthermal

#endif // OPENSEEKTHERMAL_CAMERA_CALIBRATION_HPP
