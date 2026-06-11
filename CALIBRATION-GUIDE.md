# Calibrating a Seek Thermal camera

The driver returns absolute temperatures out of the box: the per-unit gain `c1`
is read from the camera firmware and the offset `c0` is anchored at boot to the
factory reference temperature. Per pixel:

```text
T_°C = c0 + c1 * raw      (frames are emitted as centi-Kelvin: cK = 100*T_°C + 27315)
```

Calibration refines the offset for a specific unit and corrects two optical
artifacts. It is all done with the interactive `calibration_tool` and saved to a
single `.ini` file.

## Run the tool

```shell
ros2 run openseekthermal calibration_tool        # or ./build/openseekthermal/calibration_tool
```

Options: `--serial S` / `--port P` to pick a device, `--calibration FILE` to
load and update an existing `.ini`. The tool opens a menu; each step is optional
and writes its own section of the `.ini`:

- `[1]` Vignette — flat-field the lens shading
- `[2]` Dead pixels — mask stuck pixels
- `[3]` Temperature — correct the absolute offset (`c0`)
- `[5]` Save — write `calibration_<serial>.ini`

## [1] Vignette

Aim at a flat, uniform surface that fills the frame (a blank wall or a sheet of
cardboard); keep your hand and breath out of view. The tool averages frames and
fits a radial correction. A small edge-vs-centre swing just means the lens is
already flat.

## [2] Dead pixels

Pan the camera slowly across a varied scene for the whole capture — stuck pixels
only stand out when the scene around them changes. The tool flags pixels that
stay static or read `0` / `0xFFFF`, and warns if parts of the sensor saw too
little motion to be judged.

## [3] Temperature offset

Choose "known temperature", fill the centre of the frame with a stable target of
known temperature and high emissivity (~0.95), and enter that temperature. The
tool measures the centre ROI through the full runtime pipeline and shifts `c0`
so the reading matches; the firmware gain `c1` is kept.

Zero-thermometer references:

- Ice-water slush, stirred, with ice still present: **0 °C**.
- Boiling water: **100 °C** at sea level; subtract ~0.34 °C per 100 m of altitude.
- A matte-black-taped printer bed, or a stirred water bath read with a thermometer.

Aim at the water surface (emissivity ~0.96), not at shiny metal (emissivity
~0.1, which reads far too cold). "Manual offset" instead nudges `c0` by a fixed
°C amount against a trusted thermometer, with no capture.

## Save and use

"Save" writes `calibration_<serial>.ini` with the temperature, vignette and
dead-pixel sections together. Load it at runtime:

```cpp
auto cal = openseekthermal::loadCameraCalibration(path, cam->getFrameWidth(),
                                                  cam->getFrameHeight());
cam->setCalibration(std::move(cal));
```

The `stream_calibrated_centikelvin` example does exactly this — pass the `.ini`
as its first argument.

## Verify

Stream with the calibration loaded and aim at:

- Ice water → 0 °C ± 1 °C (if far off, re-run the temperature offset).
- Boiling water → your altitude-corrected boiling point ± 1 °C.

Skin is **not** a usable verification target: its temperature varies from ~25 °C
to ~35 °C with ambient temperature, blood flow and the body part, so it cannot
confirm a reading to within a degree. Use stable fixed points instead.
