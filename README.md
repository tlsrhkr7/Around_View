# Around-View Based Parking Assist (ROS + MORAI)

An around-view (bird’s-eye / top-view) parking assist prototype built with **ROS** and the **MORAI** simulator.  
This project generates a **360° surround view** by transforming 4 camera streams (front/rear/left/right) into a unified top-view image using homography and image stitching.

## Motivation
I was inspired by autonomous parking solutions that leverage around-view cameras to solve parking missions.  
The goal of this project is to improve open-source/ROS proficiency and implement a practical around-view pipeline end-to-end.

## What it does
- Subscribes to **4 camera images** (front/rear/left/right)
- Applies **top-view transformation** per camera using homography
- Stitches the transformed images into a single **around-view canvas**
- Overlays a **vehicle top image** in the center
- Publishes the final around-view image for visualization (e.g., `rqt_image_view`)

## Key Features
- Top-view conversion using OpenCV:
  - `cv::findHomography`
  - `cv::warpPerspective`
- Simple image composition pipeline:
  - `transformToTopView()`
  - `CreateAroundView()`
  - `attachVehicleTopView()`
  - `CropSides()` (reduce distortion on side boundaries)
- Supports quick calibration iteration via configurable source/destination points

## System Overview
**Inputs**
- `front_img_`, `rear_img_`, `left_img_`, `right_img_`: camera frames
- `src_points_`, `dst_points_`: homography points for front/rear
- `side_src_points_`, `side_dst_points_`: homography points for left/right

**Outputs**
- A single stitched **around-view image** (top-down)

## Architecture
### Nodes / Classes
- `AroundViewNode`
  - Receives images from each camera
  - Publishes the final stitched around-view output
- `AroundViewProcessor`
  - Performs homography transforms and stitching
  - Handles vehicle overlay and final cropping

## Tech Stack
- ROS (image subscribe/publish)
- MORAI simulator (camera sources, testing)
- OpenCV (homography, warping, stitching utilities)
- C++ (main implementation)

## Demo / Results
- Intermediate results: per-camera top-view transform
- Final result: stitched around-view image with vehicle overlay  
(See the attached report slides for screenshots and outputs.)

## How to Run (Typical Flow)
> Exact commands and topics may differ depending on your repo layout.  
> The usual flow is:

1. Launch MORAI and enable/attach 4 camera feeds to the vehicle.
2. Run this ROS node/package to subscribe to the 4 camera topics.
3. Visualize output:
   - `rqt_image_view` (recommended)  
   - or any ROS image viewer

## Calibration Notes
- Homography accuracy is highly dependent on camera placement and FOV.
- Start by tuning `src_points` / `dst_points` per camera until the lane/ground plane aligns.
- Side cameras often need extra care due to perspective distortion and overlap boundary artifacts.

## Known Issues / Limitations
- **Latency / delay** can occur due to compute load and camera publish rates.
- Visible seams on overlapping regions (simple stitching without blending).
- Around-view range and coverage depend on camera configuration (FOV/position).

## Future Work
- Reduce delay by optimizing processing and adjusting camera frequency
- Apply blending / seam optimization on overlap areas
- Expand the around-view coverage range
- Extend to driving functions:
  - lane-based vehicle position analysis (lane departure warning)
  - parking slot & obstacle detection + path planning
  - mapping/localization using around-view features

## Development Timeline (Summary)
- Environment setup (ROS, MORAI)
- Camera placement & calibration in MORAI
- Top-view transform + stitching implementation
- Final composition & performance tuning
- Visualization and presentation material preparation
