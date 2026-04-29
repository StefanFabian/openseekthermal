# OpenSeekThermal

This is an open library to interface with SeekThermal thermal cameras.  
It is in no way affiliated with the SeekThermal company or brand.

## Libraries

### openseekthermal

A C++ library to find, open and retrieve frames from the following
SeekThermal cameras:

* SeekThermal Compact
* SeekThermal Compact Pro (FF)
* SeekThermal Nano 300

Support for the Nano 200 can probably be added easily but I do not have
access to one to test.

#### What the runtime does to every frame

1. Bulk-transfer the raw frame from USB.
2. Inpaint any pixels that read exactly `0x0000` or `0xFFFF` with a 3×3
   Gaussian over their valid neighbours.
3. Apply the shutter flat-field correction (FFC) using the most recent
   in-camera calibration frame.
4. *(optional)* Inpaint pixels listed in a per-unit dead-pixel mask.
5. *(optional)* Subtract a per-unit radial vignette polynomial.

Steps 4 and 5 are produced by the calibration tools shipped with the
project — see [CALIBRATION.md](CALIBRATION.md).

#### Dependencies (openseekthermal)

* libusb

### openseekthermal_gstreamer

A GStreamer plugin built on top of openseekthermal. It exposes the
`openseekthermalsrc` element — a `GstPushSrc` that delivers live frames
from the SeekThermal camera.

#### Properties

| Property                | Type    | Description                                                                                              |
|-------------------------|---------|----------------------------------------------------------------------------------------------------------|
| `serial`                | string  | Identifies the device if available (Nano only). Takes precedence over `port`.                            |
| `port`                  | string  | USB port of the device, e.g. `1-3.2`. Use `list_seek_devices` to discover. Ignored if `serial` is set.   |
| `skip-invalid-frames`   | bool    | If true, only emit thermal frames; skip the calibration frames captured while the shutter is closed.     |
| `normalize`             | bool    | Stretch frame values to the full 16-bit range for visual contrast.                                       |
| `normalize-frame-count` | uint    | Sliding-window size used to compute the normalization scale and offset.                                  |
| `dead-pixel-mask`       | string  | Path to the PGM mask produced by `calibrate_dead_pixels`. Empty = disabled.                              |
| `vignette-correction`   | string  | Path to the INI file produced by `calibrate_vignette`. Empty = disabled.                                 |

If `dead-pixel-mask` or `vignette-correction` is set, load failures
(missing file, bad format, dimension mismatch) fail the pipeline
transition with a descriptive error rather than silently streaming
uncorrected frames.

#### Dependencies (openseekthermal_gstreamer)

* openseekthermal
* GStreamer
  * (Ubuntu) `libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev`

## Installation

### ROS 2 Workspace

Just clone the repo, build your workspace and source the environment
again. Please read the ROS 2 documentation on workspaces if you don't
know what this means.

### Manual Installation (No ROS 2 required)

Clone the repository.
Install `openseekthermal` (required by `openseekthermal_gstreamer`):

```bash
cd openseekthermal
mkdir build && cd build
cmake ..
make -j4
sudo make install
cd ../..
```

Install `openseekthermal_gstreamer`:

```bash
cd openseekthermal_gstreamer
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

To make the GStreamer plugin available, it has to be added to the
`GST_PLUGIN_PATH` environment variable. This is done automatically in a
ROS 2 workspace; if installing manually, add the path where the installed
library can be found to that environment variable.

## Usage Examples

### openseekthermal

Cameras can be found using:

```c++
std::vector<openseekthermal::SeekDevice> connected_devices = openseekthermal::listDevices();
```

A `SeekDevice` encodes the type of camera and the identifier.
The identifier is the serial if available (only for Nano) or alternatively
the USB bus + port (e.g. `1-1` for bus 1 port 1, or `1-3.2` for bus 1, hub
on port 3, port 2 on the hub). The serial always has precedence over the
bus + port.

Example to get the first Nano connected and stream frames with optional
per-unit corrections:

```c++
#include <openseekthermal/openseekthermal.hpp>
#include <openseekthermal/dead_pixel_mask.hpp>
#include <openseekthermal/vignette_correction.hpp>

using namespace openseekthermal;

std::vector<SeekDevice> connected =
    listDevices(SeekDevice::Type::Nano200 | SeekDevice::Type::Nano300);
if (connected.empty()) /* exit gracefully */;

SeekThermalCamera::SharedPtr camera = createCamera(connected[0]);
try {
  camera->open();
} catch (std::runtime_error &e) {
  /* exit gracefully */
}

// Optional: load per-unit calibration artifacts. Both setters validate
// dimensions against the open camera and throw on mismatch.
const int w = camera->getFrameWidth();
const int h = camera->getFrameHeight();
camera->setDeadPixelMask(loadDeadPixelMaskPgm("dead_pixels.pgm", w, h));
camera->setVignetteCorrection(loadVignetteCorrection("vignette.ini", w, h));

unsigned char *image_data = nullptr;
size_t image_data_size = 0;
FrameHeader header;
while (camera->grabFrame(&image_data, image_data_size, &header) == GrabFrameResult::SUCCESS) {
  // Skip the in-camera calibration frame (shutter closing/opening).
  if (header.getFrameType() != FrameType::THERMAL_FRAME) continue;
  /* Process image data — uint16 values, little-endian. */
}
```

See [CALIBRATION.md](CALIBRATION.md) for how to produce
`dead_pixels.pgm` and `vignette.ini` for a given camera.

### openseekthermal_gstreamer

Display the live feed from the first SeekThermal camera found:

```bash
gst-launch-1.0 openseekthermalsrc ! videoconvert ! autovideosink
```

By default the frames are normalized to give higher visual contrast; if
you require the raw values, use `normalize=false`.

Apply per-unit corrections:

```bash
gst-launch-1.0 openseekthermalsrc \
    dead-pixel-mask=/etc/openseekthermal/cam_A_dead.pgm \
    vignette-correction=/etc/openseekthermal/cam_A_vignette.ini \
  ! videoconvert ! autovideosink
```

## Per-camera calibration

The library ships two tools that produce per-unit calibration artifacts:

* `calibrate_dead_pixels` — detects pixels that survive the camera's
  built-in stuck-at-0/0xFFFF inpainting (partially-stuck, abnormally
  noisy, or fixed-DC-offset pixels) and writes an 8-bit PGM mask.
* `calibrate_vignette` — fits a radial polynomial to the lens-shading
  vignette and writes the coefficients to an INI file.

Both are run once per physical camera, and the resulting files are then
loaded at runtime via the C++ API or the GStreamer properties shown
above.

For capture procedures, recommended order, parameter tuning, and
troubleshooting tips see [CALIBRATION.md](CALIBRATION.md).
