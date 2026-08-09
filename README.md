# LDWS — Computer Vision Based Lane Departure Warning System

An embedded computer vision system for real-time lane detection and departure warning, running entirely on an ESP32-CAM.

## Overview

LDWS detects lane boundaries from a live camera feed and warns when a vehicle drifts out of its lane — a simplified, embedded-hardware take on the lane-keeping systems used in modern ADAS (Advanced Driver Assistance Systems). The full pipeline — image capture, edge detection, lane-line fitting, and control output — runs on-device on an ESP32-CAM, optimized for low-latency inference at the edge rather than offloading to a more powerful host machine.

## Features

- **Gradient-based edge detection** — extracts candidate lane-line features from each frame
- **Iterative RANSAC lane fitting** — robustly fits lane lines even in high-noise environments (shadows, worn lane markings, uneven lighting), by iteratively discarding outlier points
- **Closed-loop control** — feeds detected lane position into a control loop for steering actuation and safety alerts
- **Low-latency, edge-optimized execution** — designed to run within the ESP32-CAM's limited compute/memory budget, rather than requiring a paired external processor

## Tech Stack

`ESP32-CAM` · `Embedded C++` · RANSAC Algorithm · Gradient-based edge detection

## Architecture

```
Camera frame → Preprocessing (grayscale / gradient) → Edge detection
            → RANSAC line fitting (outlier-robust) → Lane position estimate
            → Control logic → Steering actuation + safety alert
```

Each captured frame is reduced to gradient information to highlight likely lane-marking edges. Because real-world footage is noisy (shadows, cracks, faded paint), a naive line fit would be thrown off by outlier edge points — RANSAC addresses this by repeatedly fitting a line to random subsets of points and keeping the fit with the most inliers, discarding noise robustly. The resulting lane estimate feeds a closed-loop controller that triggers a steering correction signal and/or a driver-facing safety alert.

## Getting Started

### Prerequisites
- ESP32-CAM board
- Arduino IDE or PlatformIO with ESP32 board support installed
- USB-to-serial programmer (for flashing, since ESP32-CAM has no onboard USB)

### Build & Flash
```bash
git clone https://github.com/avaneeshkakade/ldws.git
cd ldws
# Open in Arduino IDE / PlatformIO, select the ESP32-CAM board profile,
# then build and flash over the serial programmer.
```


## Known Limitations / Future Work

- Performance depends heavily on lighting/road-marking quality — extreme conditions (heavy rain, faded markings) reduce detection reliability
- Currently tuned for a specific camera mounting angle/height — would need recalibration for a different setup
- No sensor fusion (e.g. combining with IMU data) for more robust lane-position estimation

## What I Learned

This project required optimizing a real computer-vision pipeline to run within the tight compute and memory constraints of embedded hardware, and gave hands-on experience with RANSAC as a general-purpose tool for robust estimation in the presence of noisy, real-world data — not just a textbook algorithm.
