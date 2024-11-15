import numpy as np
import pandas as pd
from scipy.signal import bilinear, lfilter, group_delay
import matplotlib.pyplot as plt

# Set the figure size and layout
plt.rcParams["figure.figsize"] = [15.00, 7.00]
plt.rcParams["figure.autolayout"] = True

# Define columns and load the CSV file, dropping any rows with NaN values
columns = ["Seconds", "Nanoseconds", "Accel_X", "Accel_Y", "Accel_Z"]
df = pd.read_csv("imu_data.csv", usecols=columns)

# Remove rows with NaN values
df = df.dropna()

df['Time'] = df['Seconds'] + df['Nanoseconds'] * 1e-9

# Convert the relevant columns to NumPy arrays
time = df["Time"].to_numpy()
accel_x = df["Accel_X"].to_numpy()
accel_y = df["Accel_Y"].to_numpy()
accel_z = df["Accel_Z"].to_numpy()

# Function to apply a second-order low-pass filter
def second_order_lowpass(data, cutoff, fs, damping=0.5):
    # Calculate the natural frequency and damping
    omega_n = 2 * np.pi * cutoff
    # Define the s-domain coefficients for the second-order filter
    b = [omega_n**2]
    a = [1, 2 * damping * omega_n, omega_n**2]
    
    # Convert to z-domain coefficients using bilinear transformation
    bz, az = bilinear(b, a, fs)
    
    # Apply the filter
    y = lfilter(bz, az, data)
    return y

# Function to apply a moving average filter
def moving_average(data, window_size=5):
    return np.convolve(data, np.ones(window_size) / window_size, mode='same')

# Function to calculate the group delay for the low-pass filter at the cutoff frequency
def calculate_lowpass_group_delay(cutoff, fs, damping=0.5):
    omega_n = 2 * np.pi * cutoff
    b = [omega_n**2]
    a = [1, 2 * damping * omega_n, omega_n**2]
    
    bz, az = bilinear(b, a, fs)
    w, gd = group_delay((bz, az), fs=fs)
    
    cutoff_idx = np.argmin(np.abs(w - 2 * np.pi * cutoff))
    return gd[cutoff_idx] / fs  # Delay in seconds

# Function to calculate delay for the moving average filter
def calculate_moving_average_delay(window_size, fs):
    delay_samples = (window_size - 1) / 2
    return delay_samples / fs  # Delay in seconds

# Filter parameters
fs = 400.0         # Sample rate, Hz
cutoff = 2.0       # Low-pass cutoff frequency, Hz
damping = 0.7      # Damping factor for low-pass filter
window_size = 300   # Moving average window size

# Calculate delays
lowpass_delay = calculate_lowpass_group_delay(cutoff, fs, damping)
moving_average_delay = calculate_moving_average_delay(window_size, fs)
full_delay = lowpass_delay + moving_average_delay

print(f"Approximate delay from low-pass filter: {lowpass_delay:.4f} seconds")
print(f"Delay from moving average filter: {moving_average_delay:.4f} seconds")
print(f"Delay from both filters: {full_delay:.4f} seconds")

# Apply second-order low-pass filter
filtered_data_x = second_order_lowpass(accel_x, cutoff, fs, damping)
filtered_data_y = second_order_lowpass(accel_y, cutoff, fs, damping)
filtered_data_z = second_order_lowpass(accel_z, cutoff, fs, damping)

# Apply moving average filter to the low-pass filtered data
smoothed_data_x = moving_average(filtered_data_x, window_size=window_size)
smoothed_data_y = moving_average(filtered_data_y, window_size=window_size)
smoothed_data_z = moving_average(filtered_data_z, window_size=window_size)

print("Standard deviation: " + str(np.std(accel_x)))
print("Standard deviation: " + str(np.std(accel_y)))
print("Standard deviation: " + str(np.std(accel_z)))
print("Standard deviation: " + str(np.std(smoothed_data_x)))
print("Standard deviation: " + str(np.std(smoothed_data_y)))
print("Standard deviation: " + str(np.std(smoothed_data_z)))

# Plotting to visualize
# plt.plot(time, accel_x, label='Original Signal')
plt.plot(time, smoothed_data_x, label='Low-Pass + Moving Average')
plt.plot(time, smoothed_data_y, label='Low-Pass + Moving Average')
plt.plot(time, smoothed_data_z, label='Low-Pass + Moving Average')
plt.legend()
plt.show()
