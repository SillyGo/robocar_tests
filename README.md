## Robocar Vision Prototypes

This repository is where I keep ROS 2 Python prototypes for vision-based navigation.

Most files are experiment snapshots. Some are pure vision tests, some are control tests, and some are steps toward map-based navigation.

## Repository Structure

```text
src/
  pointcloud_mapping_prototypes/
    camFramePointcloud.py
  pure_control_tests/
    VHScontrol1.py
  pure_vision_tests/
    FishEye.py
    KMandCanny.py
    kmeansSeg.py
  short_sight_prototypes/
    FirstWorkingPrototype.py
    SecondPrototypeVersion.py
```

## Method Families

### Pointcloud Mapping Prototypes

These are the prototypes I use for map-based vehicle control, using either:
- a short map (for example, integrating approximately the last 10 seconds), or
- a larger/full road map.

The point cloud extracted by `camFramePointcloud.py` is published in camera frame and then transformed (using TF frames + odometry) into a `map` frame, where mapping and map-based control happen.

### Short Sight Methods

I call these "short sight" because they use only the immediate camera feed to compute control actions.

They do not use previous frames in the control computation, unlike the mapping approach.

## File Overview

### `src/short_sight_prototypes/FirstWorkingPrototype.py`
- Main short-sight obstacle-avoidance prototype using edge extraction plus a radial occupancy histogram.
- Pipeline summary:
  - Discards top part of the frame (region likely to contain less relevant road info).
  - Converts to grayscale and runs Canny edge detection.
  - Projects edge pixels into angular bins (histogram sectors) based on pixel geometry.
  - Chooses steering from a weighted average of sector "free-space" scores.
  - Publishes `/cmd_vel` through a ROS 2 timer.
- Also renders a debug window for the radial histogram.

### `src/short_sight_prototypes/SecondPrototypeVersion.py`
- Cleaner and faster variant of the same radial histogram idea.
- Main differences from the first prototype:
  - Simpler angle calculation with `atan2`.
  - Normalization and clipping strategy adjusted for stability.
  - More direct steering command generation.
  - Similar ROS 2 integration (`/lane_camera/image` subscriber + `/cmd_vel` publisher).

### `src/pure_control_tests/VHScontrol1.py`
- Earlier version of the histogram control logic.
- Combines edge extraction and full-image per-pixel sector scoring.
- Includes MiniBatch K-Means setup, but control is mainly driven by edge-based histogram scoring.
- Useful as a reference for the evolution of the controller logic.

### `src/pure_vision_tests/KMandCanny.py`
- Combined experiment: color clustering (MiniBatch K-Means) plus Canny edges.
- Masks upper third of the image, clusters remaining pixels, then computes edges.
- Returns edge output for debug visualization and timing inspection.

### `src/pure_vision_tests/kmeansSeg.py`
- Pure segmentation test using K-Means color quantization.
- Flattens image to pixel vectors, clusters into `k` colors, reconstructs segmented frame.
- Useful to evaluate segmentation quality and runtime independently of navigation logic.

### `src/pure_vision_tests/FishEye.py`
- Fisheye undistortion/remapping test.
- Builds intrinsic matrix `K` and distortion coefficients `d`.
- Uses OpenCV fisheye map generation + remap to inspect corrected output.
- Includes a rectangular region mask before remapping.

### `src/pointcloud_mapping_prototypes/camFramePointcloud.py`
- Converts RGB edge pixels and aligned depth image into a point cloud in camera frame.
- ROS 2 behavior:
  - Subscribes to RGB and depth topics.
  - Performs timestamp consistency check.
  - Detects edges in RGB image.
  - Uses camera intrinsics (`fx`, `fy`, `cx`, `cy`) to project pixels + depth into 3D coordinates.
  - Publishes `PointCloud2` on `/camFramePC`.

## Core OpenCV Methods Used

The main OpenCV building blocks I use in these prototypes are:

- `cv2.cvtColor(..., cv2.COLOR_BGR2GRAY)`
  - Converts RGB/BGR frames to grayscale to simplify edge detection.

- `cv2.Canny(gray, threshold1, threshold2)`
  - Extracts strong intensity transitions; used as the core obstacle contour signal.

- ROI masking/cropping
  - `frame[0:int(h/3), :] = 0` and similar slicing to ignore less useful regions.
  - `cv2.rectangle` + `cv2.bitwise_and` for explicit mask-based region selection.

- `cv2.resize(..., interpolation=cv2.INTER_AREA)`
  - Reduces resolution to improve speed before geometric histogram processing.

- Fisheye correction
  - `cv2.fisheye.initUndistortRectifyMap(...)`
  - `cv2.remap(...)`

- Visualization helpers
  - `cv2.fillPoly`, `cv2.polylines`, `cv2.applyColorMap` for histogram debug views.
  - `cv2.imshow` and `cv2.waitKey(1)` for live display during runtime.

- K-Means for feature simplification
  - K-Means was tested to reduce visual noise before edge extraction.
  - On real roads, a lot of texture/details can create noisy edges.
  - By clustering colors into a small number of dominant regions, the goal was to preserve broad road structure and suppress small irelevant variations.
  - In practice, I stopped leaning on it and focused more on tuning Canny thresholds, which gave a better speed/benefit tradeoff for these prototypes.

## ROS 2 Structure Pattern

Most scripts follow roughly the same architecture:

- A `Node` class with:
  - camera subscription (`/lane_camera/image`)
  - optional depth subscription (`/lane_camera/depth_image`)
  - `/cmd_vel` publisher (or point cloud publisher)
- A periodic timer callback publishing control commands.
- An image callback that runs `openCV_main(...)` and updates behavior.

## Notes on Maturity

- These are prototypes and experiment snapshots, not a polished package.
- Parameters are mostly hardcoded (camera intrinsics, thresholds, gains).
- Computational cost varies significanlty between scripts (especially full K-Means and dense per-pixel loops).

This README is mainly a map of what each experiment does. I will attach videos/images and benchmark notes as this evolves.