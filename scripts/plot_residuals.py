#!/usr/bin/env python3
import argparse
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_residuals(csv_file):
    print(f"Loading data from {csv_file}...")
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: Could not find {csv_file}. Make sure you've run the data recorder node first.")
        return

    # We know the system is running at 100Hz (dt = 0.01s)
    dt = 0.01 
    time = np.arange(len(df)) * dt
    
    # Restrict to first 1000 seconds
    mask = time <= 500.0
    time = time[mask]
    df = df[mask]
    
    num_joints = 5
    
    # Plot 1: Position Residuals (Error in rad)
    fig1, axes1 = plt.subplots(num_joints, 1, figsize=(10, 12), sharex=True)
    fig1.suptitle('Healthy EKF Position Residuals', fontsize=16)
    
    for i in range(1, num_joints + 1):
        ax = axes1[i-1]
        # Position residuals are named res_q1, res_q2, etc.
        ax.plot(time, df[f'res_q{i}'], label=f'Joint {i} Pos Error', color='blue', linewidth=1)
        ax.set_ylim(-0.4, 0.4)
        ax.set_ylabel('Error (rad)')
        ax.grid(True)
        ax.legend(loc='upper right')
        
    axes1[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    
    # Plot 2: Velocity Residuals (Error in rad/s)
    fig2, axes2 = plt.subplots(num_joints, 1, figsize=(10, 12), sharex=True)
    fig2.suptitle('Healthy EKF Velocity Residuals', fontsize=16)
    
    for i in range(1, num_joints + 1):
        ax = axes2[i-1]
        # Velocity residuals are named res_dq1, res_dq2, etc.
        ax.plot(time, df[f'res_dq{i}'], label=f'Joint {i} Vel Error', color='orange', linewidth=1)
        ax.set_ylim(-0.4, 0.4)
        ax.set_ylabel('Error (rad/s)')
        ax.grid(True)
        ax.legend(loc='upper right')
        
    axes2[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    
    print("Displaying plots. Close the plot windows to exit.")
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot EKF Residuals from CSV')
    parser.add_argument('--file', type=str, default='healthy_residuals.csv', help='Path to the residuals CSV file')
    args = parser.parse_args()
    
    plot_residuals(args.file)