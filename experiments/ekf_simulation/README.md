# Synthetic IMU + visual-position EKF demonstration

**Scope:** A self-contained Python/NumPy learning demo of a 2D extended Kalman filter. Does **not** consume real visual odometry, IMU, EuRoC, LiDAR, stereo camera, ROS 2, or physical hardware. All metric units reflect simulated ground-truth coordinates.

- Five-state nonlinear dynamics `[x,y,vx,vy,yaw]`; noisy, biased body-frame acceleration and gyro.
- VO-like position samples (simulated) at 2 Hz, IMU prediction at 20 Hz; random measurement dropouts.
- Joseph covariance update; tests for PSD covariance, invalid input, dropout and repeatability.
- Compare held-last-VO estimate, unaided IMU dead reckoning, and fused EKF against generated truth.

Run using your existing AeroDetect NumPy environment:

```bash
python -m pytest test_ekf_fusion.py -q
python ekf_fusion.py
```

`ekf_results.csv` contains measurements generated locally. Do not claim these are EuRoC trajectory errors or proven improvement in a real system.
