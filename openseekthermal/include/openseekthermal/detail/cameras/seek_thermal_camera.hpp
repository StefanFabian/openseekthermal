// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef OPENSEEKTHERMAL_SEEK_THERMAL_CAMERA_HPP
#define OPENSEEKTHERMAL_SEEK_THERMAL_CAMERA_HPP

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
   * Grabs a frame from the camera.
   * @param image_data The image data. Can be provided or will be allocated if nullptr.
   *             Caller has ownership of the data.
   * @param size The size of the image data. Pass the current size of image_data if provided.
   *             If size is smaller than the required size, BUFFER_TOO_SMALL will be returned without
   *             attempting to retrieve a frame.
   * @param header Optionally, the frame header can be provided to fill with the frame header data.
   * @param info Optionally, the frame info can be provided to receive additional information about the frame.
   * @returns GrabFrameResult indicating the result of the frame grab.
   *          A calibration frame will also be returned but is probably useless and can be distinguished by the return value.
   * @throws USBError Could be thrown if an error occurred during frame transfer.
   */
  GrabFrameResult grabFrame( unsigned char **image_data, size_t &size, FrameHeader *header = nullptr );

  GrabFrameResult _grabRawFrame( unsigned char **frame_data, size_t &size );

  //! Returns the size of a frame in bytes.
  size_t getFrameSize() const;

  int getFrameWidth() const;

  int getFrameHeight() const;

  Framerate getMaxFramerate() const;

  std::string readFirmwareInfo();

  std::string readChipID();

protected:
  bool write( SeekDeviceCommand command, const std::vector<unsigned char> &data );

  bool read( SeekDeviceCommand command, std::vector<unsigned char> &data );

  void openDevice();

  virtual void setupCamera() = 0;

private:
  void extractFrame( const unsigned char *data, unsigned char *frame_data );
  void applyCalibration( unsigned char *frame_data );

  SeekDevice device_;
  std::recursive_mutex device_mutex_;
  std::mutex buffer_mutex_;
  std::vector<unsigned char> buffer_;
  std::vector<uint16_t> calibration_frame_;
  libusb_context *usb_context_ = nullptr;
  libusb_device_handle *usb_device_handle_ = nullptr;
  uint16_t offset_ = 0x4000;
  bool own_usb_context_ = false;
};
} // namespace openseekthermal

#endif // OPENSEEKTHERMAL_SEEK_THERMAL_CAMERA_HPP
