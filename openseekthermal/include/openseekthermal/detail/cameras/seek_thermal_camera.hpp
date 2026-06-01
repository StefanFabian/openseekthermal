// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef OPENSEEKTHERMAL_SEEK_THERMAL_CAMERA_HPP
#define OPENSEEKTHERMAL_SEEK_THERMAL_CAMERA_HPP

#include "../../camera_calibration.hpp"
#include "../frame.hpp"
#include "../usb/seek_device.hpp"

#include <memory>
#include <mutex>
#include <vector>

struct libusb_context;
struct libusb_device_handle;

namespace openseekthermal
{

enum class GrabFrameResult {
  SUCCESS,
  DEVICE_NOT_OPEN,
  FAILED_TO_START_TRANSFER,
  TRANSFER_INCOMPLETE,
  BUFFER_TOO_SMALL,
  UNKNOWN_ERROR
};

std::string to_string( GrabFrameResult result );

class SeekThermalCamera
{
public:
  using SharedPtr = std::shared_ptr<SeekThermalCamera>;

  explicit SeekThermalCamera( SeekDevice device, libusb_context *usb_context = nullptr ) noexcept;
  virtual ~SeekThermalCamera();

  /*!
   * Opens the camera device in preparation for streaming.
   * @throws USBError if the device could not be opened.
   * @throws SeekSetupError if the camera setup failed.
   */
  void open();

  void close();

  /*!
   * Grabs a frame from the camera. Shutter (flat-field) correction, dead-pixel
   * inpaint, vignette, drift compensation and temperature mapping run only
   * when `header->getFrameType() == FrameType::THERMAL_FRAME`. For other frame
   * types `image_data` holds the extracted raw pixels (host-endian,
   * dead-pixel sentinels left in place); callers must branch on the frame type
   * before treating `image_data` as centi-Kelvin. Periodic shutter (ft=1)
   * frames still update the host-side flat-field reference internally even
   * though their image data is returned uncorrected.
   * @param image_data The image data. Can be provided or will be allocated if nullptr.
   *             Caller has ownership of the data.
   * @param size The size of the image data. Pass the current size of image_data if provided.
   *             If size is smaller than the required size, BUFFER_TOO_SMALL will be returned
   * without attempting to retrieve a frame.
   * @param header Optionally, the frame header can be provided to fill with the frame header data.
   * @returns GrabFrameResult indicating the result of the frame grab.
   * @throws USBError Could be thrown if an error occurred during frame transfer.
   */
  GrabFrameResult grabFrame( unsigned char **image_data, size_t &size, FrameHeader *header = nullptr );

  GrabFrameResult _grabRawFrame( unsigned char **frame_data, size_t &size );

  //! Send a TOGGLE_SHUTTER (0x37) command to the device, forcing a shutter
  //! calibration cycle. The next received transfers will typically be a ft=6
  //! / ft=1 / ft=20 sequence followed by the resumed ft=3 stream.
  bool triggerShutter();

  //! Returns the size of a frame in bytes.
  size_t getFrameSize() const;

  int getFrameWidth() const;

  int getFrameHeight() const;

  Framerate getMaxFramerate() const;

  std::string readFirmwareInfo();

  std::string readChipID();

  /*!
   * Install a unified CameraCalibration. Replaces any previously installed
   * sections. Temperature, vignette and dead-pixel sections are each
   * optional. The temperature section overrides the camera-derived two-point
   * anchor (`c0`, `c1`); leaving the temperature section empty reverts to
   * the camera-derived anchor.
   *
   * The in-band substrate-drift compensation (see
   * `setDriftCompensationEnabled`) runs on the raw counts regardless of
   * whether a user temperature calibration is installed. Calibrations should
   * therefore be fit against the drift-compensated stream.
   *
   * @throws std::invalid_argument if vignette or dead-pixel dimensions do not
   *         match the camera's frame.
   */
  void setCalibration( CameraCalibration cal );

  //! Remove any installed calibration. The camera reverts to emitting cK
  //! using the camera-derived two-point anchor (factory `(raw_at_T_ref,
  //! T_ref)` + per-unit firmware slope `c1 = 100/u16@row1+16`), or raw counts
  //! if the factory anchor was never read. Drift compensation is independent
  //! of this call; toggle it via `setDriftCompensationEnabled()`.
  void clearCalibration();

  /*!
   * Enable or disable host-side one-point flat-field correction using the
   * device's periodic shutter (ft=1) frames. When enabled (default), each
   * shutter frame is converted to a per-pixel deviation-from-mean array
   * which is subtracted from subsequent scene frames, suppressing
   * fixed-pattern noise while preserving the scene-frame mean.
   */
  void setShutterCorrectionEnabled( bool enabled );

  bool isShutterCorrectionEnabled() const { return shutter_correction_enabled_; }

  //! Compute housing temperature in °C from the per-frame `housing_adc`
  //! (frame header byte 10) using the factory-stored Beta-NTC anchors. Returns
  //! NaN when the factory anchors weren't read (e.g. older code paths or
  //! truncated dumps). Read-only — no side effects on streaming.
  double computeHousingTemperature( uint16_t housing_adc ) const noexcept;

  bool hasFactoryHousingNtc() const noexcept { return factory_housing_valid_; }

  //! Raw mean of the most recent shutter (ft=1) frame, in sensor counts.
  //! Returns 0 before the first shutter event.
  double getLastShutterMean() const noexcept { return last_shutter_mean_; }

  /*!
   * Enable / disable the in-band substrate-drift compensation. Applies to
   * every temperature mapping the driver performs, regardless of whether a
   * user-supplied `TemperatureCalibration` is installed — user calibrations
   * should be fit against the compensated stream so the c0/c1 they ship
   * encode only scene radiometry, not thermal drift.
   *
   * The compensation is anchored to camera-provided transfer data instead of a
   * host cold-start baseline:
   *
   * 1. The first non-visible body column (`x = getFrameWidth()`) is used as a
   *    per-frame detector/reference signal. It tracks the substrate-dependent
   *    raw drift seen by scene pixels.
   *
   * 2. The one-time `ft=4` transfer provides a stable per-unit startup
   *    reference for that same column before the thermal stream begins.
   *    Anchoring to this camera-emitted value keeps warm stream restarts on
   *    the same absolute level. The anchor is captured inside `open()`; if the
   *    device fails to emit `ft=4` after retries, `open()` throws rather than
   *    streaming with a substitute anchor.
   *
   * 3. `ft=1` and startup `ft=8` shutter-reference frames update the per-pixel
   *    flat-field offsets. They are not temperature-mapped.
   *
   * Combined:
   * ```
   *   raw_corrected = raw − K_pad·(pad − startup_pad_reference)
   *   T = c0 + c1·raw_corrected
   * ```
   *
   * Disabled when `substrate_drift_coefficient_` is 0. Defaults to ON.
   */
  void setDriftCompensationEnabled( bool enabled );

  bool isDriftCompensationEnabled() const noexcept { return drift_compensation_enabled_; }

  //! Per-product substrate-drift slope `K_pad` (scene-raw counts per
  //! pad-column count). 0 disables the intra-session drift term.
  double getDriftCompensationCoefficient() const noexcept { return substrate_drift_coefficient_; }

protected:
  bool write( SeekDeviceCommand command, const std::vector<unsigned char> &data );

  bool read( SeekDeviceCommand command, std::vector<unsigned char> &data );

  void openDevice();

  virtual void setupCamera() = 0;

  //! Per-product anchor read from factory page 0x40 by per-device setupCamera()
  //! implementations.
  //!   raw_at_T_ref @ rel offset 0x04 (f32 LE)
  //!   T_ref        @ rel offset 0x0c (f32 LE, universal 22.0 °C)
  double factory_raw_at_T_ref_ = 0.0;
  double factory_T_ref_ = 0.0;

  //! Beta-NTC housing thermometer constants, derived from factory anchors
  //! when read by setupCamera(). The housing temperature in °C for a given
  //! `housing_adc` (frame header byte 10) is then
  //!   1/T_K = 1/T_ref_K + (1/B) · ln((H − K)/(H_ref − K))
  //! where H_ref = factory_raw_at_T_ref_, T_ref_K = T_ref + 273.15.
  //!   K = H_ref − anchor[1].f[2] / T_ref  (factory page 0x20 rel-off 0x3C)
  //!   B = factory float32 @ factory byte 0x58 (factory page 0x40 rel-off 0x18)
  //! `factory_housing_valid_` indicates whether both anchors were read. NOTE:
  //! the K decode places the curve knee inside the streaming operating band, so
  //! computeHousingTemperature is only approximate; it is not used on the
  //! default temperature path. See reverse_engineering/housing-ntc/.
  double factory_housing_K_ = 0.0;
  double factory_housing_B_ = 0.0;
  bool factory_housing_valid_ = false;

  //! Per-product substrate-drift slope `K_pad`. Pad-column `getFrameWidth()`
  //! (one past the visible image) acts as a per-frame substrate-temperature
  //! proxy. Empirically the scene pixels drift `K_pad` raw counts per pad
  //! count (Nano 300 ≈ 1.15, Compact Pro ≈ 1.04). Set by the per-device
  //! `setupCamera()`; 0 disables the in-band drift compensation.
  double substrate_drift_coefficient_ = 0.0;

private:
  void extractFrame( const unsigned char *data, unsigned char *frame_data );

  //! Sentinel-excluded mean of the first pad column (`x = getFrameWidth()`)
  //! across the visible rows of a raw transfer buffer. Returns 0.0 if the
  //! buffer is too small or all sampled pixels are 0 / 0xFFFF.
  double computePadDriftSignal( const unsigned char *transfer_buffer,
                                size_t transfer_buffer_size ) const;

  //! Recompute per-pixel additive shutter offsets `mean - pixel` from an
  //! already-extracted shutter frame and update `last_shutter_mean_`. Logs a
  //! warning and leaves state untouched if no valid pixels are present.
  void applyShutterReference( const uint16_t *extracted_shutter, size_t pixel_count );

  //! Pumps frames from the freshly-restarted device looking for the one-shot
  //! boot frames ft=4 (`FIRST_FRAME`) and ft=8 (`STARTUP_CALIBRATION_FRAME`),
  //! using them to seed the drift anchor and host-side FFC reference. Returns
  //! false if neither is observed within a small budget; `open()` retries by
  //! re-running `setupCamera()`.
  bool tryConsumeStartupFrames();

  SeekDevice device_;
  std::recursive_mutex device_mutex_;
  std::mutex buffer_mutex_;
  std::vector<unsigned char> buffer_;
  //! Per-pixel additive shutter offset `mean(ft1) - ft1[i]`, computed from
  //! the latest shutter frame (dead-pixel sentinels excluded from the mean).
  //! Applied as `corrected[i] = raw[i] + shutter_offset_[i]`. Empty until the
  //! first shutter frame has been seen.
  std::vector<int32_t> shutter_offset_;
  double last_shutter_mean_ = 0.0;
  CameraCalibration calibration_;
  libusb_context *usb_context_ = nullptr;
  libusb_device_handle *usb_device_handle_ = nullptr;
  bool shutter_correction_enabled_ = true;
  bool own_usb_context_ = false;

  //! In-band drift-compensation per-session state. Captured during open() from
  //! the startup ft=4 transfer's first padding column; cleared on close().
  //! Toggling `drift_compensation_enabled_` does not touch the anchor.
  bool drift_compensation_enabled_ = true;
  bool drift_anchor_set_ = false;
  double drift_reference_anchor_ = 0.0;
};
} // namespace openseekthermal

#endif // OPENSEEKTHERMAL_SEEK_THERMAL_CAMERA_HPP
