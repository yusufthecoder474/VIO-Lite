"""Synthetic 2D EKF state estimation for internship learning and verification.

State: [x, y, vx, vy, yaw]. Body-frame accelerometer + gyro are noisy and biased.
VO-like sensor: intermittent noisy 2D position (NOT real visual odometry).
No real IMU, camera, EuRoC, ROS integration or metric-scale estimation.
"""
from __future__ import annotations
import csv
import argparse
import numpy as np

class PoseEKF:
    def __init__(self, dt: float = 0.05):
        self.dt = dt
        self.x = np.zeros(5, dtype=float)
        self.P = np.diag([2., 2., 1., 1., 0.2])
        self.Q = np.diag([0.002, 0.002, 0.012, 0.012, 0.00015])
        self.R = np.diag([0.8**2, 0.8**2])

    def predict(self, ax_body: float, ay_body: float, gyro: float):
        if not np.all(np.isfinite([ax_body, ay_body, gyro])):
            raise ValueError('Non-finite IMU measurement')
        dt = self.dt
        px, py, vx, vy, th = self.x
        c, s = np.cos(th), np.sin(th)
        ax = c * ax_body - s * ay_body
        ay = s * ax_body + c * ay_body
        nxt = np.array([px + vx*dt + 0.5*ax*dt*dt,
                        py + vy*dt + 0.5*ay*dt*dt,
                        vx + ax*dt, vy + ay*dt,
                        (th + gyro*dt + np.pi) % (2*np.pi) - np.pi])
        dax_dth = -s*ax_body - c*ay_body
        day_dth =  c*ax_body - s*ay_body
        F = np.eye(5)
        F[0,2] = dt; F[1,3] = dt
        F[0,4] = 0.5*dt*dt*dax_dth; F[1,4] = 0.5*dt*dt*day_dth
        F[2,4] = dt*dax_dth; F[3,4] = dt*day_dth
        self.x = nxt
        self.P = F @ self.P @ F.T + self.Q
        self.P = (self.P + self.P.T) / 2
        return self.x.copy()

    def update(self, position):
        if position is None:
            return False
        z = np.asarray(position, dtype=float)
        if z.shape != (2,) or not np.all(np.isfinite(z)):
            raise ValueError('Position measurement must contain two finite values')
        H = np.zeros((2,5)); H[0,0] = 1; H[1,1] = 1
        innov = z - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = np.linalg.solve(S, H @ self.P).T
        self.x = self.x + K @ innov
        # Joseph form protects positive semi-definiteness.
        I_KH = np.eye(5) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T
        self.P = (self.P + self.P.T) / 2
        return True

def simulate(seed: int = 42, seconds: int = 60, dt: float = 0.05):
    rng = np.random.default_rng(seed)
    n = round(seconds/dt)
    filter_ = PoseEKF(dt)
    truth = np.zeros(5)
    raw_vo = np.zeros(2)
    dead = np.zeros(5)
    histories = {'truth': [], 'vo_hold': [], 'imu_only': [], 'ekf': []}
    vo_updates = 0; vo_drops = 0
    for k in range(n):
        t = k*dt
        # Smooth changing orientation; acceleration given in body frame.
        accel_b = np.array([0.16*np.sin(t/7), 0.11*np.cos(t/9)])
        turn_rate = 0.05*np.sin(t/11)
        c,s=np.cos(truth[4]),np.sin(truth[4])
        world_a=np.array([c*accel_b[0]-s*accel_b[1], s*accel_b[0]+c*accel_b[1]])
        truth[:2] += truth[2:4]*dt + 0.5*world_a*dt*dt
        truth[2:4] += world_a*dt
        truth[4] = (truth[4]+turn_rate*dt+np.pi)%(2*np.pi)-np.pi
        # Deliberately biased sensors; bias NOT estimated by the EKF.
        imu = accel_b + np.array([0.018, -0.013]) + rng.normal(0, 0.065, size=2)
        gyro = turn_rate + 0.0015 + rng.normal(0, 0.008)
        filter_.predict(float(imu[0]),float(imu[1]),float(gyro))
        cd,sd=np.cos(dead[4]),np.sin(dead[4])
        da=np.array([cd*imu[0]-sd*imu[1],sd*imu[0]+cd*imu[1]])
        dead[:2] += dead[2:4]*dt+0.5*da*dt*dt
        dead[2:4] += da*dt
        dead[4] += gyro*dt
        if k % 10 == 0:  # VO-like measurements at 2 Hz; IMU at 20 Hz.
            if rng.random() < 0.2:
                vo_drops += 1
                filter_.update(None)
            else:
                raw_vo = truth[:2] + rng.normal(0,0.8,size=2)
                filter_.update(raw_vo)
                vo_updates += 1
        histories['truth'].append(truth[:2].copy())
        histories['vo_hold'].append(raw_vo.copy())
        histories['imu_only'].append(dead[:2].copy())
        histories['ekf'].append(filter_.x[:2].copy())
    data={k:np.array(v) for k,v in histories.items()}
    metrics={}
    for kind in ('vo_hold','imu_only','ekf'):
        errors=np.linalg.norm(data[kind]-data['truth'],axis=1)
        metrics[kind]={'rmse_m':float(np.sqrt(np.mean(errors**2))),
                       'final_error_m':float(errors[-1])}
    return data, metrics, {'steps':n,'vo_updates':vo_updates,'vo_drops':vo_drops}

def main():
    p=argparse.ArgumentParser(); p.add_argument('--output',default='ekf_results.csv')
    args=p.parse_args()
    _,metrics,info=simulate()
    print('SCOPE,SYNTHETIC 2D (not EuRoC; not physical hardware; not ROS-integrated)')
    print(f"STEPS,{info['steps']},VO_UPDATES,{info['vo_updates']},VO_DROPS,{info['vo_drops']}")
    print('METHOD,POSITION_RMSE_M,FINAL_ERROR_M')
    rows=[]
    for name in ('vo_hold','imu_only','ekf'):
        m=metrics[name]
        print(f"{name},{m['rmse_m']:.4f},{m['final_error_m']:.4f}")
        rows.append((name,m['rmse_m'],m['final_error_m']))
    with open(args.output,'w',newline='') as f:
        wr=csv.writer(f);wr.writerow(('method','position_rmse_m','final_position_error_m'));wr.writerows(rows)
    print(f'CSV_SAVED,{args.output}')

if __name__=='__main__':
    main()
