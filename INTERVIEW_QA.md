# VIO-Lite — explaining the project

## 1. What is visual odometry?

Visual odometry estimates how a camera moved by comparing consecutive images. In this project I detect corners, track them with optical flow, remove bad correspondences, estimate an essential matrix and recover rotation and translation direction. I accumulate those relative transforms into a trajectory. I do not build a map.

## 2. Why is monocular scale unknown?

A nearby small scene and a larger scene farther away can produce the same image. A single camera therefore cannot distinguish a movement of 10 cm from 1 m without another source of information. Also, this project's pairwise essential-matrix output has unit translation on every step: relative step lengths across frames are unknown too. A single global scale is only a simplifying experiment and can fail under changing speed.

## 3. What does an IMU measure?

A gyroscope measures angular velocity in rad/s. An accelerometer measures specific force in m/s² in its sensor frame. It does not directly return velocity or world-frame acceleration. A stationary accelerometer normally has a nonzero gravity-related reading. I must account for its orientation, bias and gravity before integrating it.

## 4. How does IMU integration work here?

I interpolate measurements to camera timestamps and integrate each short interval. Midpoint angular velocity updates orientation with a rotation exponential. I rotate midpoint accelerometer readings into the world frame and add the downward gravity vector. I then update velocity and position. A quiet initialization prefix supplies a gravity direction and gyro bias under a stationary assumption. Small attitude or bias errors accumulate quickly; this is not a complete inertial navigation filter.

## 5. What is an essential matrix?

It describes the relative geometry of two calibrated camera views. It combines rotation with translation direction and constrains where corresponding image points can lie. OpenCV fits it from undistorted normalized coordinates, then recovers a relative pose. It does not give translation distance in metres.

## 6. Why use RANSAC?

Some optical-flow matches are wrong. RANSAC repeatedly tries a small subset and looks for a model supported by many points. That makes the essential-matrix estimate less sensitive to bad matches. I still need sufficient inliers and parallax; RANSAC cannot create information in a blank image or solve every geometric degeneracy.

## 7. What causes tracking failure, and what does the app do?

Few corners, motion blur, repeated texture, low parallax and failed pose recovery can cause rejection. The CSV records the exact status. After any failed pair, a continuous global trajectory cannot be trusted, so the app permanently marks the run disconnected and withholds global metric coordinates. It may continue collecting local tracking/window diagnostics. Held positions in raw output are not successful motion estimates.

## 8. How does sliding-window scale fitting work?

I fit `s*q_i - v0*t_i = d_i + lever_i` over 20 poses. Here `s` is scale, `q` is visual displacement, `v0` is unknown starting velocity and `d` is integrated acceleration displacement. The lever term accounts for the camera being offset from the IMU. I solve four unknowns with Eigen SVD. I reject weak excitation, poor conditioning, excessive residual, nonpositive scale and outliers. Straight constant-velocity motion cannot distinguish scale from initial velocity.

For a numerical test, I construct visual motion and inertial displacement with a known scale of 2.5 and a nonzero starting velocity. The test checks that the solver recovers 2.5, including after one large injected outlier. That checks the mathematics on synthetic inputs; it is not evidence of real-world scale recovery.

## 9. What did evaluation show?

The included evidence records 12 passing CTest cases and eight passing Python evaluator tests. A synthetic EuRoC-shaped command-line run checks loading, image motion, synchronization, CSV generation and plotting. Its summary is in `evidence/smoke/RESULTS.csv`. EuRoC MH_01_easy could not be downloaded in the build environment, so EuRoC trajectory accuracy and metric-scale recovery remain unverified. I would not quote a EuRoC error or claim navigation performance from these tests.

The offline evaluator associates timestamps and aligns metric positions with rotation and translation only. If scale was not established, metric ATE stays unavailable. An optional Sim(3) plot is explicitly a shape diagnostic that fits scale from ground truth.

## 10. What are the main limitations?

The largest algorithmic limitation is unit-step visual translation: one scale cannot generally correct varying step lengths. The inertial side depends on stationary initialization, known gravity and configured accelerometer bias. Noise, calibration errors and synchronization offsets cause drift. There is no bias estimator, EKF, map, bundle adjustment, loop closure or repair after tracking loss. Passing implementation tests does not establish accurate recorded-sensor VIO.

## 30-second explanation

“VIO-Lite is a C++17 camera-motion prototype with an IMU scale experiment. I use OpenCV to track image corners and recover relative motion, synchronize the IMU to image intervals, and use Eigen to fit scale and initial velocity over a short window. The application records why tracking or scale fitting fails. I verified numerical and image-processing tests and a synthetic command-line demonstration; recorded EuRoC accuracy is still unverified.”

## 2-minute explanation

“The problem is that one camera does not provide metric distance. I built a small command-line pipeline to understand what information images and an IMU provide, and where a simple method breaks.

“The loader reads EuRoC-style timestamps as 64-bit nanoseconds, checks calibration and missing files, and interpolates IMU values to image boundaries. On the camera side, I detect corners, track them forward and backward, reject poor matches, undistort their coordinates and estimate an essential matrix with RANSAC. I accumulate camera-to-world transforms with the correct inverse convention.

“For the IMU, I assume a short stationary prefix, align gravity and estimate a constant gyro bias. I integrate orientation and acceleration with midpoint measurements. Camera-to-IMU extrinsics are explicit, including the translation lever arm.

“The scale experiment fits one scale and three starting-velocity components over a sliding window. Constant-velocity straight motion is not observable for this fit, so I check excitation and SVD conditioning and reject inconsistent windows. Ground truth is only used in a separate Python evaluator, with rigid alignment for metric error.

“My tests cover parsing, timing, calibration, failures, transforms, integration and known scale examples. The synthetic CLI demonstration exercises the application and produces CSVs and a plot. I cannot yet claim EuRoC accuracy. The biggest limitation is that the visual baseline gives unit translation per pair, so changing speed is not represented by one shared scale. This project demonstrates a transparent experiment and its failure modes, rather than a complete VIO solution.”

## 5-minute demonstration

**0:00–0:45 — State the problem.** Explain monocular ambiguity and show the architecture diagram. Point out the separate ground-truth arrow into offline evaluation only.

**0:45–1:30 — Show calibration and timing.** Open `config/default.yaml`, then explain cam0 `T_BS`, identity imu0 `T_BS`, nanoseconds and interpolated interval endpoints. Explain the quiet-prefix assumption.

**1:30–2:15 — Verify the implementation.** Run `ctest --test-dir build --output-on-failure`, then `python scripts/test_evaluation.py -v`. Show the synthetic translation test and scale example. Explain that tests check behavior, not EuRoC accuracy.

**2:15–3:15 — Run the demonstration.** Run `python scripts/run_project.py --skip-build --output output-demo`. Open the resulting trajectory CSV and point to matches, inliers, tracking flags and scale rejection statuses. The generated data is synthetic; do not call it a drone flight.

**3:15–4:00 — Explain scale.** Write `s*q - v0*t = d + lever`. Explain why initial velocity must be fitted and why constant velocity is degenerate. Show the empty metric fields when scale is unavailable.

**4:00–5:00 — Show plots and limits.** Open `output-demo/trajectory.png`. Explain the two unit systems, the metric-unavailable panel, and why a Sim(3) diagnostic cannot prove recovered scale. Finish with what remains to measure: actual MH_01_easy run, tracking success and rigid-aligned metric ATE only if the scale experiment succeeds.
