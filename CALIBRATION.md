# Per-camera calibration

Two artifacts the runtime can correct given a per-unit calibration file:

- **Dead pixels** that survive the drivers built-in stuck-at-0/0xFFFF
  inpainting (partially stuck, abnormally noisy, or fixed-DC-offset).
- **Lens-shading vignette** — edges typically read slightly warmer than
  the centre because the lens equilibrates to a different temperature
  than the shutter.

Run each calibration once per physical camera, save the output files,
then load them at runtime.

## Build the tools

The calibration tools live under `openseekthermal/tools/` and are built
by default with the `openseekthermal` library (`BUILD_TOOLS=ON`):

```bash
cd openseekthermal
mkdir -p build && cd build
cmake ..
make -j4
```

The two binaries you need land in the build directory:

- `./calibrate_dead_pixels`
- `./calibrate_vignette`

The commands below assume you run them from there. They are not
installed by `make install` — use them in place.

## 1. Dead pixels

```bash
./calibrate_dead_pixels --out dead_pixels.pgm
```

> **Pan / sweep the camera across a varied scene** during the whole
> capture (~200 frames). A good pixel sees changing scene content and
> builds temporal stddev; a stuck pixel doesn't. If you point at a
> uniform field the detector cannot tell stuck pixels from healthy ones.

Output: `dead_pixels.pgm` — 8-bit PGM, 255 = dead, 0 = good.

## 2. Vignette

```bash
./calibrate_vignette --out vignette.ini --dead-pixels dead_pixels.pgm
```

> Point at any roughly-flat scene (a wall is fine). Hold reasonably
> still. Let the camera run ~60 s before capturing so it reaches
> thermal equilibrium with ambient.

Passing `--dead-pixels` excludes the bad pixels found in step 1 from
the polynomial fit, which is why this step runs second. The vignette
effect is small enough that letting dead pixels into the fit measurably
biases the coefficients.

Output: `vignette.ini` — radial polynomial fit (a handful of floats).

## 3. Apply at runtime

### GStreamer

```bash
gst-launch-1.0 openseekthermalsrc \
    dead-pixel-mask=dead_pixels.pgm \
    vignette-correction=vignette.ini \
  ! videoconvert ! autovideosink
```

Either property can be omitted to disable that correction. Load
failures (missing file, dimension mismatch, etc.) fail the pipeline
transition with a descriptive error.

### C++

```cpp
#include <openseekthermal/dead_pixel_mask.hpp>
#include <openseekthermal/vignette_correction.hpp>

const int w = camera->getFrameWidth();
const int h = camera->getFrameHeight();
camera->setDeadPixelMask(
    openseekthermal::loadDeadPixelMaskPgm("dead_pixels.pgm", w, h));
camera->setVignetteCorrection(
    openseekthermal::loadVignetteCorrection("vignette.ini", w, h));
```

`clearDeadPixelMask()` / `clearVignetteCorrection()` remove either
correction at runtime.

## Notes

- **Calibrations are per-unit** — don't share files between physical
  cameras.
- Both tools accept `--serial S` or `--port P` to pick a specific
  camera, and `--write-diagnostics` to dump intermediate PGMs (useful
  if you want to sanity-check the result).
- `calibrate_dead_pixels` warns if the captured scene was too static
  (low median stddev) — re-run with more motion if you see that.
- Run `<tool> --help` for the full set of tuning knobs.
