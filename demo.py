#!/usr/bin/env python3
"""
Vision-Based Head Controller Demo (Hardware-Free)

Expected runtime: 5–25 seconds (always < 1 minute)

Compares:
1) Baseline: direct pixel->angle mapping (with rate limit)
2) Proposed: α–β filter (with robust gating) + stable dynamical system controller

Saves to ./outputs/ :
- yaw_pitch_tracking.png
- pixel_tracking.png
- summary.txt
"""

import math
import os
from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt


# # # # # # # # # # # # # # # # # # # # # # # #
# Parameters and Utilities                    #
# # # # # # # # # # # # # # # # # # # # # # # #

@dataclass
class CameraIntrinsics:
    fx: float = 1064.31468
    fy: float = 1068.32207
    cx: float = 308.1238
    cy: float = 253.60508


@dataclass
class Params:
    W: int = 640
    H: int = 480
    fps: float = 30.0
    T: float = 18.0

    # α–β (g–h) filter
    g: float = 0.25
    h: float = 0.05
    max_innov_px: float = 35.0  # robust clamp for outliers

    # DS controller + rate limit
    tau: float = 0.45
    max_rate: float = 2.0       # rad/s

    # synthetic measurement corruption
    noise_px: float = 6.0
    outlier_prob: float = 0.02
    outlier_mag: float = 80.0


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def px_to_angles(x, y, K: CameraIntrinsics):
    """
    Intrinsics-based mapping:
      yaw   = -atan((x - cx)/fx)
      pitch =  atan((y - cy)/fy)
    """
    yaw = -math.atan((x - K.cx) / K.fx)
    pitch = math.atan((y - K.cy) / K.fy)
    return yaw, pitch


def rmse(a, b):
    a = np.asarray(a)
    b = np.asarray(b)
    return float(np.sqrt(np.mean((a - b) ** 2)))


def jitter_metric(signal):
    """Std of frame-to-frame increments (rad/frame). Lower = smoother."""
    s = np.asarray(signal)
    return float(np.std(np.diff(s)))


# # # # # # # # # # # # # # # # # # # # # # # #
# α–β Filter (1D)                             #
# # # # # # # # # # # # # # # # # # # # # # # #

class GHFilter:
    """
    1D constant-velocity α–β filter:
      x_pred = x + v*dt
      r      = z - x_pred
      x      = x_pred + g*r
      v      = v + (h/dt)*r

    Includes robust innovation clamp to limit outlier effect.
    """
    def __init__(self, g, h, max_innov):
        self.g = float(g)
        self.h = float(h)
        self.max_innov = float(max_innov)
        self.x = None
        self.v = 0.0

    def update_and_predict(self, z, dt):
        """
        Returns:
          x_pred_next: one-step-ahead predicted position
          r_raw:       innovation before clamping
          r_used:      innovation after clamping (what actually influenced the filter)
          x_pred:      prediction prior to update (useful for debugging)
        """
        dt = max(1e-3, float(dt))
        '''
        At initialization, the filter state is set directly from the first measurement, as no prior prediction exists; in this case the innovation is 
        defined as zero and the predicted state coincides with the initialized state.
        '''
        if self.x is None:
            self.x = float(z)
            self.v = 0.0
            return self.x, 0.0, 0.0, self.x

        # predict (prior)
        x_pred = self.x + self.v * dt

        # innovation
        r_raw = float(z) - x_pred
        r_used = clamp(r_raw, -self.max_innov, self.max_innov)

        # update
        self.x = x_pred + self.g * r_used
        self.v = self.v + (self.h * r_used / dt)

        # one-step-ahead prediction (post)
        x_pred_next = self.x + self.v * dt
        return x_pred_next, r_raw, r_used, x_pred



 

# # # # # # # # # # 
# Demo            #       
# # # # # # # # # # 

def main():
    print("Expected runtime: 5–25 seconds")
    os.makedirs("outputs", exist_ok=True)

    p = Params()
    K = CameraIntrinsics()

    dt = 1.0 / p.fps
    t = np.arange(0, p.T, dt)

    innov_x_raw = np.zeros_like(t)
    innov_x_used = np.zeros_like(t)
    innov_y_raw = np.zeros_like(t)
    innov_y_used = np.zeros_like(t)

    # True face motion (pixels): sinusoids + a step to make it more interesting
    x_true = 320 + 120*np.sin(0.6*t) + 25*(t > 9.0)
    y_true = 240 + 70*np.sin(0.4*t)  - 15*(t > 12.0)

    # clamp to image
    x_true = np.clip(x_true, 0, p.W - 1)
    y_true = np.clip(y_true, 0, p.H - 1)

    # true angles (ground truth)
    yaw_true = np.zeros_like(t)
    pitch_true = np.zeros_like(t)
    for i in range(len(t)):
        yaw_true[i], pitch_true[i] = px_to_angles(float(x_true[i]), float(y_true[i]), K)

    # Noisy measurements + occasional outliers
    rng = np.random.default_rng(4)
    x_meas = x_true + rng.normal(0, p.noise_px, len(t))
    y_meas = y_true + rng.normal(0, p.noise_px, len(t))

    outliers = rng.random(len(t)) < p.outlier_prob
    print(f"Injected outliers: {int(np.count_nonzero(outliers))} / {len(t)} frames")
    x_meas[outliers] += rng.choice([-1, 1], size=np.count_nonzero(outliers)) * p.outlier_mag
    y_meas[outliers] += rng.choice([-1, 1], size=np.count_nonzero(outliers)) * p.outlier_mag

    x_meas = np.clip(x_meas, 0, p.W - 1)
    y_meas = np.clip(y_meas, 0, p.H - 1)

    # Filters (x and y)
    fx = GHFilter(p.g, p.h, p.max_innov_px)
    fy = GHFilter(p.g, p.h, p.max_innov_px)

    # Baseline and Proposed head states
    yaw_b = 0.0
    pitch_b = 0.0
    yaw_p = 0.0
    pitch_p = 0.0

    yaw_base = np.zeros_like(t)
    pitch_base = np.zeros_like(t)
    yaw_prop = np.zeros_like(t)
    pitch_prop = np.zeros_like(t)

    x_filt = np.zeros_like(t)
    y_filt = np.zeros_like(t)

    for i in range(len(t)):
        ########### Baseline: direct mapping + rate limit #######
        y_t_b, p_t_b = px_to_angles(float(x_meas[i]), float(y_meas[i]), K)

        max_step = p.max_rate * dt
        yaw_b += clamp(y_t_b - yaw_b, -max_step, max_step)
        pitch_b += clamp(p_t_b - pitch_b, -max_step, max_step)

        yaw_base[i] = yaw_b
        pitch_base[i] = pitch_b

        # Proposed: α–β filtered pixels -> DS controller  
        xf, rx_raw, rx_used, _ = fx.update_and_predict(float(x_meas[i]), dt)
        yf, ry_raw, ry_used, _ = fy.update_and_predict(float(y_meas[i]), dt)

        x_filt[i] = xf
        y_filt[i] = yf

        innov_x_raw[i] = rx_raw
        innov_x_used[i] = rx_used
        innov_y_raw[i] = ry_raw
        innov_y_used[i] = ry_used


        y_t, p_t = px_to_angles(xf, yf, K)

        # DS update: theta <- theta + (-(theta - theta_target)/tau)*dt, then rate limit
        dy = (-(yaw_p - y_t) / max(p.tau, 1e-3)) * dt
        dp_ = (-(pitch_p - p_t) / max(p.tau, 1e-3)) * dt

        yaw_p += clamp(dy, -max_step, max_step)
        pitch_p += clamp(dp_, -max_step, max_step)

        yaw_prop[i] = yaw_p
        pitch_prop[i] = pitch_p

    ##########################
    # Metrics vs ground truth 
    ##########################
    yaw_rmse_base = rmse(yaw_base, yaw_true)
    yaw_rmse_prop = rmse(yaw_prop, yaw_true)
    pitch_rmse_base = rmse(pitch_base, pitch_true)
    pitch_rmse_prop = rmse(pitch_prop, pitch_true)

    yaw_jit_base = jitter_metric(yaw_base)
    yaw_jit_prop = jitter_metric(yaw_prop)
    pitch_jit_base = jitter_metric(pitch_base)
    pitch_jit_prop = jitter_metric(pitch_prop)

    summary = []
    summary.append("=== Summary Metrics (lower is better) ===")
    summary.append(f"Yaw RMSE (rad):   baseline={yaw_rmse_base:.4f}, proposed={yaw_rmse_prop:.4f}")
    summary.append(f"Yaw jitter (rad/frame): baseline={yaw_jit_base:.5f}, proposed={yaw_jit_prop:.5f}")
    summary.append(f"Pitch RMSE (rad): baseline={pitch_rmse_base:.4f}, proposed={pitch_rmse_prop:.4f}")
    summary.append(f"Pitch jitter (rad/frame): baseline={pitch_jit_base:.5f}, proposed={pitch_jit_prop:.5f}")

    print("\n".join(summary))

    with open("outputs/summary.txt", "w") as f:
        f.write("\n".join(summary) + "\n")

    #############################################
    # Plots (saved to outputs/)
    #############################################

    # 1) Yaw + Pitch comparison
    fig, ax = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    ax[0].plot(t, yaw_true, label="yaw true")
    ax[0].plot(t, yaw_base, label="yaw baseline")
    ax[0].plot(t, yaw_prop, label="yaw proposed")
    ax[0].set_ylabel("Yaw (rad)")
    ax[0].legend(loc="upper right")
    ax[0].set_title("Yaw and Pitch Tracking vs Ground Truth")

    ax[1].plot(t, pitch_true, label="pitch true")
    ax[1].plot(t, pitch_base, label="pitch baseline")
    ax[1].plot(t, pitch_prop, label="pitch proposed")
    ax[1].set_ylabel("Pitch (rad)")
    ax[1].set_xlabel("Time (s)")
    ax[1].legend(loc="upper right")

    fig.tight_layout()
    fig.savefig("outputs/yaw_pitch_tracking.png", dpi=200)
    plt.close(fig)

    # 2) Pixel tracking (x shown)
    fig2, bx = plt.subplots(1, 1, figsize=(10, 5))
    bx.plot(t, x_true, label="x true")
    bx.plot(t, x_meas, label="x measured", alpha=0.6)
    bx.plot(t, x_filt, label="x filtered/predicted")
    bx.set_xlabel("Time (s)")
    bx.set_ylabel("Pixel x")
    bx.set_title("Pixel Tracking (x): Noise + Outliers + α–β Filtering")
    bx.legend(loc="upper right")
    fig2.tight_layout()
    fig2.savefig("outputs/pixel_tracking.png", dpi=200)
    plt.close(fig2)

  
    # 3) Outliers + gating visualization (innovation vs clamp)
    # Show measured vs filtered x, and highlight outlier frames in red.
    fig3, cx = plt.subplots(1, 1, figsize=(10, 5))
    cx.plot(t, x_meas, label="x measured", alpha=0.6)
    cx.plot(t, x_filt, label="x filtered/predicted", linewidth=2)

    # highlight outlier frames
    cx.scatter(t[outliers], x_meas[outliers], s=18, label="outlier measurements", color="red")

    cx.set_xlabel("Time (s)")
    cx.set_ylabel("Pixel x")
    cx.set_title("Outliers (red) and α–β Robust Gating Effect on x")
    cx.legend(loc="upper right")
    fig3.tight_layout()
    fig3.savefig("outputs/outliers_gating.png", dpi=200)
    plt.close(fig3)
    # 4) Innovation plot (shows gating/clamp)
    fig3, ax3 = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    # X innovation
    ax3[0].plot(t, innov_x_raw, label="x innovation (raw)", alpha=0.6)
    ax3[0].plot(t, innov_x_used, label="x innovation (clamped)", linewidth=2)
    ax3[0].axhline(p.max_innov_px, linestyle="--", linewidth=1, label="+/- clamp")
    ax3[0].axhline(-p.max_innov_px, linestyle="--", linewidth=1)
    ax3[0].set_ylabel("Pixels")
    ax3[0].set_title("Innovation r = z - x_pred (raw vs clamped)")
    ax3[0].legend(loc="upper right")

    # Y innovation
    ax3[1].plot(t, innov_y_raw, label="y innovation (raw)", alpha=0.6)
    ax3[1].plot(t, innov_y_used, label="y innovation (clamped)", linewidth=2)
    ax3[1].axhline(p.max_innov_px, linestyle="--", linewidth=1, label="+/- clamp")
    ax3[1].axhline(-p.max_innov_px, linestyle="--", linewidth=1)
    ax3[1].set_ylabel("Pixels")
    ax3[1].set_xlabel("Time (s)")
    ax3[1].legend(loc="upper right")

    fig3.tight_layout()
    fig3.savefig("outputs/innovation_gating.png", dpi=200)
    plt.close(fig3)

    print("Demo complete. Outputs saved to ./outputs/")

if __name__ == "__main__":
    main()
