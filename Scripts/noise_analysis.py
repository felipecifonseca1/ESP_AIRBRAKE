import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq
import os

# ==========================================
# CONFIGURATION
# ==========================================
FILE_NEW = 'Data_read/REG_VOO/VOO_060.csv' # 20Hz DLPF
FILE_OLD = 'Data_read/REG_VOO/VOO_061.csv' # 41/45Hz DLPF
LOOP_HZ = 50.0           # Your system loop frequency

# Columns to analyze
accel_cols = ['AccX[g]', 'AccY[g]', 'AccZ[g]']
gyro_cols = ['GyroX[°/s]', 'GyroY[°/s]', 'GyroZ[°/s]']

def main():
    if not os.path.exists(FILE_NEW) or not os.path.exists(FILE_OLD):
        print(f"Error: Make sure {FILE_NEW} and {FILE_OLD} are in the same directory.")
        return

    print("Loading datasets...\n")
    df_new = pd.read_csv(FILE_NEW)
    df_old = pd.read_csv(FILE_OLD)

    # ==========================================
    # METHOD 1: Time-Domain Variance (Std Dev)
    # ==========================================
    print("--- METHOD 1: STANDARD DEVIATION (TIME DOMAIN) ---")
    std_new_accel = df_new[accel_cols].std()
    std_old_accel = df_old[accel_cols].std()
    
    std_new_gyro = df_new[gyro_cols].std()
    std_old_gyro = df_old[gyro_cols].std()

    print("Accelerometer Noise [g]:")
    for col in accel_cols:
        print(f"  {col}: {std_old_accel[col]:.6f} (Old) -> {std_new_accel[col]:.6f} (New)")

    print("\nGyroscope Noise [°/s]:")
    for col in gyro_cols:
        print(f"  {col}: {std_old_gyro[col]:.6f} (Old) -> {std_new_gyro[col]:.6f} (New)")

    # ==========================================
    # METHOD 2: FFT Numerical Metrics
    # ==========================================
    print("\n--- METHOD 2: SPECTRAL METRICS (FREQUENCY DOMAIN) ---")
    print("Metrics exclude 0 Hz (DC offset) to isolate pure vibration noise.\n")
    
    print("Average Spectral Amplitude (Noise Floor):")
    for col in accel_cols + gyro_cols:
        _, amp_old = compute_fft(df_old[col], LOOP_HZ)
        _, amp_new = compute_fft(df_new[col], LOOP_HZ)
        
        # Slice [1:] to remove the 0 Hz DC bias
        mean_old = np.mean(amp_old[1:])
        mean_new = np.mean(amp_new[1:])
        
        unit = "[g]" if "Acc" in col else "[°/s]"
        print(f"  {col}: {mean_old:.6f} (Old) -> {mean_new:.6f} (New) {unit}")

    print("\nPeak Spectral Noise (Worst-Case Spike):")
    for col in accel_cols + gyro_cols:
        _, amp_old = compute_fft(df_old[col], LOOP_HZ)
        _, amp_new = compute_fft(df_new[col], LOOP_HZ)
        
        # Slice [1:] to remove the 0 Hz DC bias
        max_old = np.max(amp_old[1:])
        max_new = np.max(amp_new[1:])
        
        unit = "[g]" if "Acc" in col else "[°/s]"
        print(f"  {col}: {max_old:.6f} (Old) -> {max_new:.6f} (New) {unit}")

    # ==========================================
    # GENERATE PLOTS
    # ==========================================
    print("\nGenerating visual plots...")
    plot_method_1(std_old_accel, std_new_accel, std_old_gyro, std_new_gyro)
    plot_method_2(df_old, df_new, 'AccX[g]', 'GyroX[°/s]', LOOP_HZ)


def compute_fft(data, fs):
    """Computes the one-dimensional discrete Fourier Transform."""
    N = len(data)
    yf = fft(data.values)
    xf = fftfreq(N, 1 / fs)[:N//2]
    amp = 2.0 / N * np.abs(yf[0:N//2])
    return xf, amp

def plot_method_1(std_old_acc, std_new_acc, std_old_gyro, std_new_gyro):
    """Generates bar charts comparing the Standard Deviations."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    width = 0.35
    
    # Accel Plot
    x_acc = np.arange(len(std_old_acc))
    ax1.bar(x_acc - width/2, std_old_acc, width, label='Old (45Hz DLPF)', color='indianred')
    ax1.bar(x_acc + width/2, std_new_acc, width, label='New (20Hz DLPF)', color='steelblue')
    ax1.set_title('Method 1: Accelerometer Baseline Noise (Std Dev)')
    ax1.set_xticks(x_acc)
    ax1.set_xticklabels(std_old_acc.index)
    ax1.set_ylabel('Standard Deviation [g]')
    ax1.legend()
    ax1.grid(axis='y', linestyle='--', alpha=0.7)

    # Gyro Plot
    x_gyro = np.arange(len(std_old_gyro))
    ax2.bar(x_gyro - width/2, std_old_gyro, width, label='Old (41Hz DLPF)', color='indianred')
    ax2.bar(x_gyro + width/2, std_new_gyro, width, label='New (20Hz DLPF)', color='steelblue')
    ax2.set_title('Method 1: Gyroscope Baseline Noise (Std Dev)')
    ax2.set_xticks(x_gyro)
    ax2.set_xticklabels(std_old_gyro.index)
    ax2.set_ylabel('Standard Deviation [°/s]')
    ax2.legend()
    ax2.grid(axis='y', linestyle='--', alpha=0.7)

    plt.tight_layout()
    plt.show()

def plot_method_2(df_old, df_new, acc_col, gyro_col, fs):
    """Generates FFT frequency spectrum plots."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
    nyquist = fs / 2.0

    # Calculate FFTs
    xf_old_acc, amp_old_acc = compute_fft(df_old[acc_col], fs)
    xf_new_acc, amp_new_acc = compute_fft(df_new[acc_col], fs)
    
    xf_old_gyro, amp_old_gyro = compute_fft(df_old[gyro_col], fs)
    xf_new_gyro, amp_new_gyro = compute_fft(df_new[gyro_col], fs)

    # Accel Plot
    ax1.plot(xf_old_acc, amp_old_acc, label=f'Old - {acc_col}', color='indianred', alpha=0.8)
    ax1.plot(xf_new_acc, amp_new_acc, label=f'New - {acc_col}', color='steelblue', alpha=0.8)
    ax1.axvline(x=20, color='green', linestyle=':', label='20Hz DLPF Cutoff')
    ax1.set_title('Method 2: Frequency Spectrum (Aliasing Check) - Accelerometer')
    ax1.set_xlabel('Frequency [Hz]')
    ax1.set_ylabel('Amplitude [g]')
    ax1.set_xlim(0, nyquist)
    ax1.legend()
    ax1.grid(True, linestyle='--', alpha=0.7)

    # Gyro Plot
    ax2.plot(xf_old_gyro, amp_old_gyro, label=f'Old - {gyro_col}', color='indianred', alpha=0.8)
    ax2.plot(xf_new_gyro, amp_new_gyro, label=f'New - {gyro_col}', color='steelblue', alpha=0.8)
    ax2.axvline(x=20, color='green', linestyle=':', label='20Hz DLPF Cutoff')
    ax2.set_title('Method 2: Frequency Spectrum (Aliasing Check) - Gyroscope')
    ax2.set_xlabel('Frequency [Hz]')
    ax2.set_ylabel('Amplitude [°/s]')
    ax2.set_xlim(0, nyquist)
    ax2.legend()
    ax2.grid(True, linestyle='--', alpha=0.7)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()