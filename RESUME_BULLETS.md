# VIO-Lite — resume wording

**VIO-Lite — Camera and IMU-Based Motion Estimation | C++17, OpenCV, Eigen, CMake/CTest**

- Built a C++17 motion-estimation prototype using OpenCV feature tracking, RANSAC essential-matrix estimation and explicit camera–IMU calibration handling.
- Implemented sensor synchronization with nanosecond timestamps, interpolated IMU intervals, midpoint integration and an Eigen least-squares experiment for scale and unknown initial velocity.
- Verified 12 deterministic CMake/CTest cases and eight Python evaluation tests covering parsing, numerical methods, tracking failures, synthetic motion, scale degeneracy and alignment without fitted metric scale.

**Claim boundaries:** Describe this as an educational prototype with synthetic verification. Do not claim EuRoC accuracy, recovered metric scale, SLAM, ROS2, deployment on a drone or real-world navigation performance. The recorded EuRoC evaluation is not executed. Review and understand the generated implementation before presenting it as work you can explain.
