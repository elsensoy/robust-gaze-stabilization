#!/usr/bin/env python3
"""
Hardware Experiment Runner for Jetson Orin (Visualized Comparison)
- Runs the servos based on user selection
- Computes BOTH trajectories in the background for comparison
- Saves 'servo_trajectory.png' showing Baseline vs Proposed vs Truth
"""

import time
import math
import board
import busio
import numpy as np
import matplotlib.pyplot as plt
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Import your algorithms
import demo  

# ==========================================
# 1. Hardware Setup
# ==========================================
def setup_servos():
    print("Initializing I2C and Servos...")
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        pca = PCA9685(i2c, address=0x40)
        pca.frequency = 50
        pan = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2500)
        tilt = servo.Servo(pca.channels[1], min_pulse=500, max_pulse=2500)
        pan.angle = 90
        tilt.angle = 90
        time.sleep(1)
        return pca, pan, tilt
    except Exception as e:
        print(f"Error initializing hardware: {e}")
        return None, None, None

def drive_servo(s, angle_degrees):
    if s is None or angle_degrees is None or math.isnan(angle_degrees):
        return
    val = max(0, min(180, angle_degrees))
    s.angle = val

# ==========================================
# 2. Main Experiment Loop
# ==========================================
def run_experiment(mode="proposed"):
    pca, pan_servo, tilt_servo = setup_servos()
    
    if pca is None:
        print("Hardware setup failed. Exiting.")
        return

    p = demo.Params()
    K = demo.CameraIntrinsics()
    
    dt = 1.0 / p.fps
    T = 20.0  
    t_steps = np.arange(0, T, dt)
    
    #  WIDE FIGURE 8 MOTION
    omega = 0.8  
    x_true = 320 + 280 * np.sin(omega * t_steps)
    y_true = 240 + 150 * np.sin(2 * omega * t_steps)
    
    # Noise Generation
    rng = np.random.default_rng(4)
    x_meas = x_true + rng.normal(0, p.noise_px, len(t_steps))
    y_meas = y_true + rng.normal(0, p.noise_px, len(t_steps))
    
    outliers = rng.random(len(t_steps)) < p.outlier_prob
    x_meas[outliers] += rng.choice([-1, 1], size=np.count_nonzero(outliers)) * p.outlier_mag
    y_meas[outliers] += rng.choice([-1, 1], size=np.count_nonzero(outliers)) * p.outlier_mag

    #  INITIALIZE STATE FOR BOTH ALGORITHMS ---
    # We track both simultaneously so we can plot them together
    
    # Baseline State
    yaw_base = 0.0
    pitch_base = 0.0
    
    # Proposed State (Filters + Control)
    fx = demo.GHFilter(p.g, p.h, p.max_innov_px)
    fy = demo.GHFilter(p.g, p.h, p.max_innov_px)
    yaw_prop = 0.0
    pitch_prop = 0.0
    
    #  DATA RECORDING 
    rec_yaw_base = []
    rec_pitch_base = []
    rec_yaw_prop = []
    rec_pitch_prop = []
    rec_yaw_true = []
    rec_pitch_true = []
    
    print(f"\n--- STARTING RUN: DRIVING SERVOS WITH [{mode.upper()}] ---")
    print("Computing both trajectories for plot...")
    print("Press Ctrl+C to stop early.\n")
    
    try:
        for i, t in enumerate(t_steps):
            loop_start = time.time()
            
        
            # 1. GROUND TRUTH (For Ref)
        
            y_t, p_t = demo.px_to_angles(x_true[i], y_true[i], K)
            # Convert to degrees for plotting
            rec_yaw_true.append(90 - math.degrees(y_t))
            rec_pitch_true.append(90 + math.degrees(p_t))

       
            # 2. CALCULATE BASELINE
           
            y_raw, p_raw = demo.px_to_angles(x_meas[i], y_meas[i], K)
            max_step = p.max_rate * dt
            yaw_base += demo.clamp(y_raw - yaw_base, -max_step, max_step)
            pitch_base += demo.clamp(p_raw - pitch_base, -max_step, max_step)
            
            # Record Baseline Degrees
            base_pan = 90 - math.degrees(yaw_base)
            base_tilt = 90 + math.degrees(pitch_base)
            rec_yaw_base.append(base_pan)
            rec_pitch_base.append(base_tilt)

            # 3. CALCULATE PROPOSED
       
            xf, _, _, _ = fx.update_and_predict(x_meas[i], dt)
            yf, _, _, _ = fy.update_and_predict(y_meas[i], dt)
            y_target, p_target = demo.px_to_angles(xf, yf, K)
            
            dy = (-(yaw_prop - y_target) / p.tau) * dt
            dp = (-(pitch_prop - p_target) / p.tau) * dt
            
            yaw_prop += demo.clamp(dy, -max_step, max_step)
            pitch_prop += demo.clamp(dp, -max_step, max_step)

            # Record Proposed Degrees
            prop_pan = 90 - math.degrees(yaw_prop)
            prop_tilt = 90 + math.degrees(pitch_prop)
            rec_yaw_prop.append(prop_pan)
            rec_pitch_prop.append(prop_tilt)

            # 4. DRIVE MOTORS (User Choice)
           
            if mode == "baseline":
                drive_servo(pan_servo, base_pan)
                drive_servo(tilt_servo, base_tilt)
            else:
                drive_servo(pan_servo, prop_pan)
                drive_servo(tilt_servo, prop_tilt)
            
            # Pacing
            elapsed = time.time() - loop_start
            if dt - elapsed > 0:
                time.sleep(dt - elapsed)
                
    except KeyboardInterrupt:
        print("\nStopping...")
    
    # CLEANUP 
    drive_servo(pan_servo, 90)
    drive_servo(tilt_servo, 90)
    time.sleep(0.5)
    pca.deinit()
    
    #  PLOTTING 
    print("Generating comparison plot...")
    plt.figure(figsize=(10, 8))
    
    # 1. Plot Ground Truth (Ideal Figure 8)
    plt.plot(rec_yaw_true, rec_pitch_true, 'g--', linewidth=2, label="Ground Truth", alpha=0.7)
    
    # 2. Plot Baseline (Noisy)
    # We make it thinner/lighter so it doesn't hide the proposed line
    plt.plot(rec_yaw_base, rec_pitch_base, 'b-', linewidth=1, label="Baseline (Direct)", alpha=0.5)
    
    # 3. Plot Proposed (Smooth)
    plt.plot(rec_yaw_prop, rec_pitch_prop, 'orange', linewidth=2.5, label="Proposed (Filtered + DS)")
    
    plt.title(f"Servo Trajectory: Baseline vs Proposed (Driven: {mode.upper()})")
    plt.xlabel("Pan Angle (Degrees)")
    plt.ylabel("Tilt Angle (Degrees)")
    plt.axis('equal') 
    plt.grid(True)
    plt.legend()
    
    filename = "servo_trajectory_comparison.png"
    plt.savefig(filename)
    print(f"Done. Comparison saved to {filename}")

if __name__ == "__main__":
    print("Select Driving Mode (Plot will show BOTH):")
    print("1. Drive with Baseline (Noisy)")
    print("2. Drive with Proposed (Smooth)")
    
    choice = input("Enter 1 or 2: ")
    
    if choice == "1":
        run_experiment("baseline")
    elif choice == "2":
        run_experiment("proposed")
    else:
        print("Invalid selection.")