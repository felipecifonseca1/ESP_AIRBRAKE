import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq
import os

# ==========================================
# CONFIGURATION
# ==========================================
FILE_BASE = 'Data_read/REG_VOO/VOO_061.csv' # 41/45Hz DLPF, No Ceramic Caps
FILE_DLPF = 'Data_read/REG_VOO/VOO_060.csv' # 20Hz DLPF, No Ceramic Caps
FILE_CAPS = 'Data_read/REG_VOO/VOO_078.csv' # 20Hz DLPF, WITH 0.1uF Ceramic Caps
LOOP_HZ = 50.0                              # System loop frequency

# Columns to analyze
accel_cols = ['AccX[g]', 'AccY[g]', 'AccZ[g]']
gyro_cols = ['GyroX[°/s]', 'GyroY[°/s]', 'GyroZ[°/s]']

def main():
    files = [FILE_BASE, FILE_DLPF, FILE_CAPS]
    for f in files:
        if not os.path.exists(f):
            print(f"Error: Could not find {f}. Please check the path.")
            return

    print("Loading datasets...\n")
    df_base = pd.read_csv(FILE_BASE)
    df_dlpf = pd.read_csv(FILE_DLPF)
    df_caps = pd.read_csv(FILE_CAPS)

    # ==========================================
    # METHOD 1: Time-Domain Variance (Std Dev)
    # ==========================================
    print("--- METHOD 1: STANDARD DEVIATION (TIME DOMAIN) ---")
    std_base_accel = df_base[accel_cols].std()
    std_dlpf_accel = df_dlpf[accel_cols].std()
    std_caps_accel = df_caps[accel_cols].std()
    
    std_base_gyro = df_base[gyro_cols].std()
    std_dlpf_gyro = df_dlpf[gyro_cols].std()
    std_caps_gyro = df_caps[gyro_cols].std()

    print("Accelerometer Noise [g]:")
    for col in accel_cols:
        print(f"  {col}: {std_base_accel[col]:.6f} (Base) -> {std_dlpf_accel[col]:.6f} (DLPF) -> {std_caps_accel[col]:.6f} (Caps)")

    print("\nGyroscope Noise [°/s]:")
    for col in gyro_cols:
        print(f"  {col}: {std_base_gyro[col]:.6f} (Base) -> {std_dlpf_gyro[col]:.6f} (DLPF) -> {std_caps_gyro[col]:.6f} (Caps)")

    # ==========================================
    # METHOD 2: FFT Numerical Metrics
    # ==========================================
    print("\n--- METHOD 2: SPECTRAL METRICS (FREQUENCY DOMAIN) ---")
    print("Metrics exclude 0 Hz (DC offset) to isolate pure vibration/electrical noise.\n")
    
    print("Average Spectral Amplitude (Noise Floor):")
    for col in accel_cols + gyro_cols:
        _, amp_base = compute_fft(df_base[col], LOOP_HZ)
        _, amp_dlpf = compute_fft(df_dlpf[col], LOOP_HZ)
        _, amp_caps = compute_fft(df_caps[col], LOOP_HZ)
        
        mean_base = np.mean(amp_base)
        mean_dlpf = np.mean(amp_dlpf)
        mean_caps = np.mean(amp_caps)
        
        unit = "[g]" if "Acc" in col else "[°/s]"
        print(f"  {col}: {mean_base:.6f} (Base) -> {mean_dlpf:.6f} (DLPF) -> {mean_caps:.6f} (Caps) {unit}")

    print("\nPeak Spectral Noise (Worst-Case Spike):")
    for col in accel_cols + gyro_cols:
        _, amp_base = compute_fft(df_base[col], LOOP_HZ)
        _, amp_dlpf = compute_fft(df_dlpf[col], LOOP_HZ)
        _, amp_caps = compute_fft(df_caps[col], LOOP_HZ)
        
        max_base = np.max(amp_base)
        max_dlpf = np.max(amp_dlpf)
        max_caps = np.max(amp_caps)
        
        unit = "[g]" if "Acc" in col else "[°/s]"
        print(f"  {col}: {max_base:.6f} (Base) -> {max_dlpf:.6f} (DLPF) -> {max_caps:.6f} (Caps) {unit}")

    # ==========================================
    # GENERATE PLOTS
    # ==========================================
    print("\nGenerating visual plots...")
    plot_method_1(std_base_accel, std_dlpf_accel, std_caps_accel, 
                  std_base_gyro, std_dlpf_gyro, std_caps_gyro)
    plot_method_2(df_base, df_dlpf, df_caps, 'AccZ[g]', 'GyroY[°/s]', LOOP_HZ)


def compute_fft(data, fs):
    """Computes the one-dimensional discrete Fourier Transform and aggressively strips the DC offset."""
    N = len(data)
    yf = fft(data.values)
    xf = fftfreq(N, 1 / fs)[:N//2]
    amp = 2.0 / N * np.abs(yf[0:N//2])
    
    # Strip the first frequency bin (0 Hz / DC offset) to prevent scale blowout
    return xf[1:], amp[1:]

def plot_method_1(std_b_acc, std_d_acc, std_c_acc, std_b_gyro, std_d_gyro, std_c_gyro):
    """Generates bar charts comparing the Standard Deviations."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    width = 0.25  
    
    x_acc = np.arange(len(std_b_acc))
    ax1.bar(x_acc - width, std_b_acc, width, label='Base (45Hz, No Caps)', color='indianred')
    ax1.bar(x_acc, std_d_acc, width, label='DLPF (20Hz, No Caps)', color='steelblue')
    ax1.bar(x_acc + width, std_c_acc, width, label='Caps (20Hz + 0.1uF)', color='mediumseagreen')
    
    ax1.set_title('Method 1: Accelerometer Baseline Noise (Std Dev)')
    ax1.set_xticks(x_acc)
    ax1.set_xticklabels(std_b_acc.index)
    ax1.set_ylabel('Standard Deviation [g]')
    ax1.legend()
    ax1.grid(axis='y', linestyle='--', alpha=0.7)

    x_gyro = np.arange(len(std_b_gyro))
    ax2.bar(x_gyro - width, std_b_gyro, width, label='Base (41Hz, No Caps)', color='indianred')
    ax2.bar(x_gyro, std_d_gyro, width, label='DLPF (20Hz, No Caps)', color='steelblue')
    ax2.bar(x_gyro + width, std_c_gyro, width, label='Caps (20Hz + 0.1uF)', color='mediumseagreen')
    
    ax2.set_title('Method 1: Gyroscope Baseline Noise (Std Dev)')
    ax2.set_xticks(x_gyro)
    ax2.set_xticklabels(std_b_gyro.index)
    ax2.set_ylabel('Standard Deviation [°/s]')
    ax2.legend()
    ax2.grid(axis='y', linestyle='--', alpha=0.7)

    plt.tight_layout()
    plt.show()

def plot_method_2(df_base, df_dlpf, df_caps, acc_col, gyro_col, fs):
    """Generates FFT frequency spectrum plots with proper scaling."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
    nyquist = fs / 2.0

    xf_b_acc, amp_b_acc = compute_fft(df_base[acc_col], fs)
    xf_d_acc, amp_d_acc = compute_fft(df_dlpf[acc_col], fs)
    xf_c_acc, amp_c_acc = compute_fft(df_caps[acc_col], fs)
    
    xf_b_gyro, amp_b_gyro = compute_fft(df_base[gyro_col], fs)
    xf_d_gyro, amp_d_gyro = compute_fft(df_dlpf[gyro_col], fs)
    xf_c_gyro, amp_c_gyro = compute_fft(df_caps[gyro_col], fs)

    # Accel Plot
    ax1.plot(xf_b_acc, amp_b_acc, label=f'Base - {acc_col}', color='indianred', alpha=0.6)
    ax1.plot(xf_d_acc, amp_d_acc, label=f'DLPF - {acc_col}', color='steelblue', alpha=0.7)
    ax1.plot(xf_c_acc, amp_c_acc, label=f'Caps - {acc_col}', color='mediumseagreen', alpha=0.9, linewidth=2)
    ax1.axvline(x=20, color='gray', linestyle=':', label='20Hz DLPF Cutoff')
    ax1.set_title('Method 2: Frequency Spectrum - Accelerometer (DC Bias Removed)')
    ax1.set_xlabel('Frequency [Hz]')
    ax1.set_ylabel('Amplitude [g]')
    
    # Dynamically scale Y-axis to 110% of the max noise spike (ignoring DC)
    max_acc_noise = max(np.max(amp_b_acc), np.max(amp_d_acc), np.max(amp_c_acc))
    ax1.set_ylim(0, max_acc_noise * 1.1)
    ax1.set_xlim(0, nyquist)
    ax1.legend()
    ax1.grid(True, linestyle='--', alpha=0.7)

    # Gyro Plot
    ax2.plot(xf_b_gyro, amp_b_gyro, label=f'Base - {gyro_col}', color='indianred', alpha=0.6)
    ax2.plot(xf_d_gyro, amp_d_gyro, label=f'DLPF - {gyro_col}', color='steelblue', alpha=0.7)
    ax2.plot(xf_c_gyro, amp_c_gyro, label=f'Caps - {gyro_col}', color='mediumseagreen', alpha=0.9, linewidth=2)
    ax2.axvline(x=20, color='gray', linestyle=':', label='20Hz DLPF Cutoff')
    ax2.set_title('Method 2: Frequency Spectrum - Gyroscope (DC Bias Removed)')
    ax2.set_xlabel('Frequency [Hz]')
    ax2.set_ylabel('Amplitude [°/s]')
    
    # Dynamically scale Y-axis to 110% of the max noise spike (ignoring DC)
    max_gyro_noise = max(np.max(amp_b_gyro), np.max(amp_d_gyro), np.max(amp_c_gyro))
    ax2.set_ylim(0, max_gyro_noise * 1.1)
    ax2.set_xlim(0, nyquist)
    ax2.legend()
    ax2.grid(True, linestyle='--', alpha=0.7)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()