# GRADUATION PROJECT: LASER PEN SYSTEM

* **Student:** Nguyen Huu Luan (Student ID: 20200253)
* **Advisor:** MSc. Do Quoc Minh Dang
* **Department:** Electronics and Telecommunications, VNUHCM - University of Science

This repository contains the complete distributed embedded firmware for the Laser Pen system. The system consists of two microcontroller modules that process and synchronize data in real-time via wireless communication protocols (ESP-NOW) and Bluetooth Human Interface Device (BLE Mouse).

---

## 🗺️ System Architecture

```text
       ┌───────────────────────────┐         ESP-NOW Protocol (2-way)            ┌───────────────────────────┐
       │ ESP32-S3 Laser Pen (Main) │<───────────────────────────────────────────>│     ESP32-CAM Vision      │
       │ - OLED UI & Tuner         │   [Coordinates x, y] <-------------------   │ - OV2640 Image Processing │
       │ - Buttons & Encoder       │   -----------------> [Config, BBox, Laser]  │ - Laser Cross Control     │
       │ - Homography Calculation  │                                             │ - Sub-pixel Tracking      │
       └─────────────┬─────────────┘                                             └───────────────────────────┘
                     │
                     │ Bluetooth HID (BLE Mouse)
                     │ (Absolute & Relative Movement)
                     ▼
               ┌───────────┐
               │  PC / OS  │
               └───────────┘
```

---

## 🧠 Core Algorithms & Technical Implementations

### 1. `ESP32CAM_Vision` Module: Bounded ROI & Dynamic Kinematic Tracking

This module is responsible for capturing high-speed Grayscale frames at QVGA resolution ($320 \times 240$) from the OV2640 sensor, applying digital noise filtering, and accurately locating the laser point at a sub-pixel level.

#### A. Bounded ROI Recovery Algorithm (New Update)
* **Previous Limitation:** When the user moved the pen too fast or moved it out of the interaction space, tracking was lost (`has_tracking = false`). The camera had to scan the entire frame ($320 \times 240 = 76,800$ pixels) to relocate the laser, consuming excessive CPU cycles, causing FPS drops, and creating high latency.
* **Optimized Solution:** The algorithm now leverages the actual physical workspace boundaries (`Bounding Box`) calculated during the Calibration phase. The ESP32-S3 saves these coordinates in NVS and transmits them to the CAM module via ESP-NOW.
  * The maximum scan area is strictly constrained by the calibrated box: `[bound_min_x, bound_max_x, bound_min_y, bound_max_y]`.
  * To ensure ergonomic safety and edge tolerance, the algorithm automatically expands this boundary by a **+10 pixel** margin on all sides:

$$
\begin{cases} 
s_x = \max(0, x_{\min} - 10) \\ 
e_x = \min(W_{\text{frame}} - 1, x_{\max} + 10) 
\end{cases}
$$

  * Upon losing tracking, the camera **only searches within this localized boundary** rather than the full frame. This reduces the processing area by $50\%$ to $70\%$, allowing for near-instantaneous recovery speed while maintaining maximum FPS stability.

#### B. Dynamic Kinematic ROI
* When the system is in stable tracking mode (`has_tracking = true`), the Region of Interest (ROI) shrinks to a micro-window around the current laser point to optimize processing time.
* The velocity vector is calculated in real-time between two consecutive frames $t-1$ and $t-2$:

$$
v_x = x_{t-1} - x_{t-2}, \quad v_y = y_{t-1} - y_{t-2}
$$

* The dynamic search radius adjusts linearly based on the velocity magnitude to keep up with rapid hand movements. It utilizes a `v_factor` (tunable via the Pen's OLED) and a `damping = 0.8f` coefficient to compensate for hardware latency:

$$
R_{\text{dynamic}} = \text{clamp}\left(15 + v_{\text{elocity}} \times \frac{v_{\text{factor}}}{10}, 15, 100\right)
$$

$$
x_{\text{predict}} = x_{t-1} + v_x \times \text{damping}
$$

#### C. Sub-pixel Centroid Positioning
* After locating the brightest pixel (Peak Detection) that exceeds the `bright_threshold`, an independent $31 \times 31$ window (`SEARCH_RADIUS = 15`) is established.
* The algorithm calculates the geometric center of mass based on light intensity distribution to achieve sub-pixel precision, eliminating jagged cursor movements:

$$
X_{\text{out}} = \frac{\sum (x \cdot I_{(x,y)})}{\sum I_{(x,y)}}, \quad Y_{\text{out}} = \frac{\sum (y \cdot I_{(x,y)})}{\sum I_{(x,y)}} \quad (\text{where } I_{(x,y)} > T_{\text{bright}})
$$

#### D. Remote Laser Synchronization
* To conserve power and ensure safety, the laser diode (Pin 13 on the CAM module) is explicitly managed by the ESP32-S3 over ESP-NOW. It turns on automatically during the 4-point calibration phase and turns off immediately upon entering tracking mode.

---

### 2. `ESP32S3_SmartPen` Module: Spatial Transformation & HID Coordination

This module acts as the central brain. It handles local I/O (buttons, encoder, OLED UI), coordinates the data loop, calculates the planar geometry matrix, and interfaces with the PC via Bluetooth HID.

#### A. Planar Homography Transformation
* A camera placed at an arbitrary angle relative to the projection screen causes Perspective Distortion. The Homography algorithm establishes a periodic $3 \times 3$ transformation matrix to non-linearly map the planar Camera coordinates $(x, y)$ to the Display coordinates $(X_{\text{screen}}, Y_{\text{screen}})$ at a $1920 \times 1080$ resolution.
* **Mathematical Optimization:** The 8-variable linear equation system (derived from the 4 corner calibration points) is solved directly using **Gauss-Jordan elimination**. All computations have been upgraded to double-precision floating-point format (`double`), completely eliminating cumulative floating-point division errors and preventing cursor drift at the edges of the screen.

#### B. Absolute Mouse Synchronization Hack
* Standard Bluetooth mice transmit data using relative displacement ($\Delta x, \Delta y$). Over time, this causes cumulative drift compared to the absolute physical position of the laser.
* **Correction Solution:** During initialization (`!mouse_initialized`), the S3 fires a rapid sequence of 20 maximum-boundary commands `bleMouse.move(-127, -127)` to force the host OS cursor to the absolute origin $(0,0)$. The system then calculates the vector distance to the mapped physical point and iteratively moves the cursor there, establishing a seamless absolute synchronization baseline.

#### C. Low-pass Filter (Exponential Moving Average) & Tunable UI
* To eliminate high-frequency jitter caused by physiological hand tremors, the output coordinates are passed through a digital low-pass filter:

$$
X_{\text{smooth}} = \alpha \cdot X_{\text{new}} + (1 - \alpha) \cdot X_{\text{old}}
$$

* **OLED Real-time UI:** The S3 module now features an I2C OLED display (SH1106G) that provides a real-time system dashboard. It displays battery levels, system states, FPS, and allows the user to dynamically tune the $\alpha$ coefficient, camera exposure, brightness threshold, and velocity factor using onboard physical buttons.

---

## 📂 Repository Structure

```text
Laser-Pen-ESP32/
├── ESP32CAM_Vision/
│   └── main.cpp          <-- Bounded ROI, Dynamic Kinematic ROI, Centroid logic, Laser Sync
├── ESP32S3_LaserPen/
│   └── main.cpp          <-- Gauss-Jordan Homography, BLE HID, OLED UI, On-the-fly Tuning
└── README.md             <-- System architecture & algorithmic documentation
```

```