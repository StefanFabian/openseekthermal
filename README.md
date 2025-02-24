# OpenSeekThermal

This is an open library to interface with SeekThermal thermal cameras.  
It is in no way affiliated with the SeekThermal company or brand.

## Libraries

### openseekthermal - Overview

Is a basic C++ library, to find, open and retrieve frames from the following SeekThermal cameras:

* SeekThermal Compact
* SeekThermal Compact Pro (FF)
* SeekThermal Nano 300

Support for the Nano 200 can probably be added easily but I do not have access to one.

#### Dependencies

* libusb

### openseekthermal_gstreamer - Overview

This is a GStreamer plugin for the Seek Thermal camera. It is based on the OpenSeekThermal library.  
The plugin provides the `openseekthermalsrc` element.
`openseekthermalsrc` is a `GstPushSrc` and will provide the live images from the SeekThermal camera.

Relevant options:

* `normalize`: (true|false) Normalize the frame values to use the full contrast range.
* `normalize-frame-count`: (int) How many frames to use for normalization.
* `skip-invalid-frames`: (true|false) If true, send only valid frames, if false, send all, which includes calibration frames where the shutter is closed and opens.
* `serial`: (string) Used to identify the device if available. Takes precedence over `port`.
* `port`: (string) The port of the device to use. E.g. 1-3.2 for bus 1, hub on port 3 and port 2 on the hub.
          Use the list_seek_devices example from openseekthermal to find the port.

#### Dependencies

* openseekthermal
* GStreamer
  * (Ubuntu) ``libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev``

## Installation

### ROS 2 Workspace

Just clone the repo, build your workspace and source the environment again.
Please read the ROS 2 documentation on workspaces if you don't know what this means.

### Manual Installation (No ROS2 required)

Clone the repository.
Install `openseekthermal` (required by `openseekthermal_gstreamer`)

```bash
cd openseekthermal
mkdir build && cd build
cmake ..
make -j4
sudo make install
cd ../..
```

Install `openseekthermal_gstreamer`

```bash
cd openseekthermal_gstreamer
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

To make the GStreamer plugin available, it has to be added to the `GST_PLUGIN_PATH` environment variable.
This is done automatically in a ROS 2 workspace but if installing manually, you have to add the path where the installed library can be found to this environment variable.

## Usage Examples

### openseekthermal - Example

Cameras can be found using:

```c++
std::vector<openseekthermal::SeekDevice> connected_devices = openseekthermal::listDevices();
```

A SeekDevice encodes the type of camera and the identifier.
The identifier is the serial if available (only for Nano) or alternatively the usb bus + port.
E.g. 1-1 for bus 1, port 1 or if using a hub 1-3.2 for bus 1, hub on port 3 and port 2 on hub.
The serial always has precedence over the bus + port.

Example to get the first Nano connected:

```c++
using namespace openseekthermal;
std::vector<SeekDevice> connected = listDevices(SeekDevice::Type::Nano200 | SeekDevice::Type::Nano300);
if (connected.empty()) /* exit gracefully */
SeekThermalCamera::SharedPtr camera = createCamera(connected[0]);
try {
  camera.open();
} catch (std::runtime_error &e) {
  /* exit gracefully */
}

unsigned char *image_data;
size_t image_data_size;
FrameHeader header;
int frame_width = camera->getFrameWidth();
int frame_height = camera->getFrameHeight();
while (camera->grabFrame(&image_data, image_data_size, &header) == GrabFrameResult::SUCCESS) {
  // Skip Calibration frame and pre/post calibration frame
  if (header.getFrameType() != FrameType::THERMAL_FRAME) continue; 
  /* Process image data which is uint16 values in little endian */
}

```

### openseekthermal_gstreamer - Example

```bash
gst-launch-1.0 openseekthermalsrc ! videoconvert ! autovideosink
```

This will open the first SeekThermal camera that is found and open a window displaying the live feed.
Note that by default the frames are normalized to give higher visual contrast, if you require the raw values,
use the option `normalize=false`.
