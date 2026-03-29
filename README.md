## Robocar Vision Prototypes

This repository contains ROS 2 Python prototypes used to test computer vision pipelines for navigation with a forward camera (and, in one case, depth data).

The code is organized as iterative experiments. Some scripts focus on segmentation, some on edge-based obstacle cues, and others on converting image observations into control commands.

## File Overview

### `FirstWorkingPrototype.py`
- Main obstacle-avoidance prototype using edge extraction plus a radial occupancy histogram.
- Pipeline summary:
  - Discards top part of the frame (region likely to contain less relevant road info).
  - Converts to grayscale and runs Canny edge detection.
  - Projects edge pixels into angular bins (histogram sectors) based on pixel geometry.
  - Chooses steering from a weighted average of sector "free-space" scores.
  - Publishes `/cmd_vel` through a ROS 2 timer.
- Also renders a visualization window for the radial histogram.

### `SecondPrototypeVersion.py`
- Cleaner and faster variant of the radial histogram approach.
- Main differences from the first prototype:
  - Simpler angle calculation with `atan2`.
  - Normalization and clipping strategy adjusted for stability.
  - More direct steering command generation.
  - Similar ROS 2 integration (`/lane_camera/image` subscriber + `/cmd_vel` publisher).

### `VHScontrol1.py`
- Early version of the histogram control logic.
- Combines edge extraction and full-image per-pixel sector scoring.
- Includes MiniBatch K-Means setup, but control is mainly driven by edge-based histogram mapping.
- Useful as a reference for the evolution of the controller logic.

### `KMandCanny.py`
- Combined experiment: color clustering (MiniBatch K-Means) plus Canny edges.
- Masks upper third of the image, clusters remaining pixels, then computes edges.
- Returns edge output for visualization and timing inspection.

### `kmeansSeg.py`
- Pure segmentation benchmark using K-Means color quantization.
- Flattens image to pixel vectors, clusters into `k` colors, reconstructs segmented frame.
- Useful to evaluate segmentation quality and runtime independently of navigation logic.

### `FishEye.py`
- Fisheye undistortion/remapping test.
- Builds intrinsic matrix `K` and distortion coefficients `d`.
- Uses OpenCV fisheye map generation + remap to inspect corrected output.
- Includes a rectangular region mask before remapping.

### `camFramePointcloud.py`
- Converts RGB edge pixels and aligned depth image into a point cloud in camera frame.
- ROS 2 behavior:
  - Subscribes to RGB and depth topics.
  - Performs timestamp consistency check.
  - Detects edges in RGB image.
  - Uses camera intrinsics (`fx`, `fy`, `cx`, `cy`) to project pixels + depth into 3D coordinates.
  - Publishes `PointCloud2` on `/camFramePC`.

## Core OpenCV Methods Used

The main OpenCV building blocks across the repository are:

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
  - On real roads, many texture/details can create noisy edges.
  - By clustering colors into a small number of dominant regions, the goal was to preserve broad road structure and suppress small irelevant variations.
  - In practice, this was later de-emphasized in favor of tuning Canny thresholds, which gave a better speed/benefit tradeoff for these prototypes.

## ROS 2 Structure Pattern

Most scripts use the same architecture:

- A `Node` class with:
  - camera subscription (`/lane_camera/image`)
  - optional depth subscription (`/lane_camera/depth_image`)
  - `/cmd_vel` publisher (or point cloud publisher)
- A periodic timer callback publishing control commands.
- An image callback that runs `openCV_main(...)` and updates behavior.

## Notes on Maturity

- These are prototypes and experiment snapshots, not a single polished package.
- Parameters are mostly hardcoded (camera intrinsics - partially hardcoded, as this info is published by the Gazebo 'camera_info' topic -, thresholds, gains).
- Computational cost varies significanlty between scripts (especially full K-Means and dense per-pixel loops).

This README is intended as a map of what each experiment does, so media and benchmark results can be attached later.