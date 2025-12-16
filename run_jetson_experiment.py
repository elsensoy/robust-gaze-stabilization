#!/usr/bin/env python3
"""
Hardware Experiment Runner for Jetson Orin
- Imports algorithm logic from demo.py
- Generates the SAME synthetic noise
- Drives REAL servos to visualize the difference
"""

import time
import math
import board
import busio
import numpy as np
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Import your algorithms from the existing demo file
import demo  

# ==========================================
# 1. Hardware Setup (Your Manual Control Code)
# ==========================================
def setup_servos():
    print("Initializing I2C and Servos...")
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c, address=0x40)
    pca.frequency = 50
    
    # Init servos (assuming channels 0 and 1)
    pan = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2500)
    tilt = servo.Servo(pca.channels[1], min_pulse=500, max_pulse=2500)
    
    # Center them start
    pan.angle = 90
    tilt.angle = 90
    time.sleep(1)
    return pca, pan, tilt

def drive_servo(s, angle_degrees):
    """Safely drive servo within 0-180 range"""
    if angle_degrees is None or math.isnan(angle_degrees):
        return
    # Clamp to physical limits
    val = max(0, min(180, angle_degrees))
    s.angle = val

# ==========================================
# 2. Simulation Loop (Real-Time)
# ==========================================
def run_experiment(mode="proposed"):
    pca, pan_servo, tilt_servo = setup_servos()
    
    # Load parameters from your demo.py class
    p = demo.Params()
    K = demo.CameraIntrinsics()
    
    # Generate the EXACT same synthetic data as the software demo
    dt = 1.0 / p.fps
    T = 15.0  # Run for 15 seconds
    t_steps = np.arange(0, T, dt)
    
    # Synthetic Face Motion (Sinusoids)
    x_true = 320 + 120*np.sin(0.6*t_steps) + 25*(t_steps > 9.0)
    y_true = 240 + 70*np.sin(0.4*t_steps)  - 15*(t_steps > 12.0)
    
    # Inject Noise & Outliers
    rng = np.random.default_rng(4) # Same seed = reproducible noise
    x_meas = x_true + rng.normal(0, p.noise_px, len(t_steps))
    y_meas = y_true + rng.normal(0, p.noise_px, len(t_steps))
    
    # Outliers
    outliers = rng.random(len(t_steps)) < p.outlier_prob
    x_meas[outliers] += rng.choice([-1, 1], size=np.count_nonzero(outliers)) * p.outlier_mag
    y_meas[outliers] += rng.choice([-1, 1], size=np.count_nonzero(outliers)) * p.outlier_mag

    # Initialize Filters (for Proposed Mode)
    fx = demo.GHFilter(p.g, p.h, p.max_innov_px)
    fy = demo.GHFilter(p.g, p.h, p.max_innov_px)
    
    # Initialize State Variables
    yaw_current = 0.0
    pitch_current = 0.0
    
    print(f"\n--- STARTING HARDWARE RUN: {mode.upper()} ---")
    print("Press Ctrl+C to stop early.\n")
    
    try:
        start_time = time.time()
        
        for i, t in enumerate(t_steps):
            loop_start = time.time()
            
            #################################
            # A) BASELINE MODE (Jerky)
            #################################
            if mode == "baseline":
                # Direct mapping from noisy measurement
                y_raw, p_raw = demo.px_to_angles(x_meas[i], y_meas[i], K)
                
                # Simple rate limit (simulating a basic servo speed cap)
                max_step = p.max_rate * dt
                yaw_current += demo.clamp(y_raw - yaw_current, -max_step, max_step)
                pitch_current += demo.clamp(p_raw - pitch_current, -max_step, max_step)

            #################################
            # B) PROPOSED MODE (Smooth)
            #################################
            elif mode == "proposed":
                # 1. Filter the noisy pixels
                xf, _, _, _ = fx.update_and_predict(x_meas[i], dt)
                yf, _, _, _ = fy.update_and_predict(y_meas[i], dt)
                
                # 2. Convert filtered pixels to target angles
                y_target, p_target = demo.px_to_angles(xf, yf, K)
                
                # 3. Dynamical System Control (Low-pass filter the angles)
                # theta_next = theta + (-1/tau * error) * dt
                dy = (-(yaw_current - y_target) / p.tau) * dt
                dp = (-(pitch_current - p_target) / p.tau) * dt
                
                # Apply rate limit
                max_step = p.max_rate * dt
                yaw_current += demo.clamp(dy, -max_step, max_step)
                pitch_current += demo.clamp(dp, -max_step, max_step)

            #################################
            # C) Convert to Hardware Angles
            #################################
            # demo.py outputs Radians (0 is center). 
            # Servos need Degrees (90 is center).
            # yaw is usually horizontal (Pan), pitch is vertical (Tilt)
            
            # Negate yaw if the servo moves the wrong way!
            pan_deg = 90 - math.degrees(yaw_current)  
            tilt_deg = 90 + math.degrees(pitch_current)

            drive_servo(pan_servo, pan_deg)
            drive_servo(tilt_servo, tilt_deg)

            #################################
            # D) Real-time Pacing
            #################################
            # The simulation step is dt (e.g., 0.033s). 
            # We sleep to make the physical robot match simulation time.
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    except KeyboardInterrupt:
        print("\nStopping...")
    
    # Cleanup
    print("Recentering servos...")
    pan_servo.angle = 90
    tilt_servo.angle = 90
    time.sleep(0.5)
    pca.deinit()
    print("Done.")

# ==========================================
# Main Menu
# ==========================================
if __name__ == "__main__":
    print("Select Mode:")
    print("1. Baseline (Noisy/Direct)")
    print("2. Proposed (Filtered/Smooth)")
    
    choice = input("Enter 1 or 2: ")
    
    if choice == "1":
        run_experiment("baseline")
    elif choice == "2":
        run_experiment("proposed")
    else:
        print("Invalid selection.")