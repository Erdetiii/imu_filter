import pandas as pd
import numpy as np
from matplotlib import pyplot as plt

# Set the figure size and layout
plt.rcParams["figure.figsize"] = [15.00, 7.00]
plt.rcParams["figure.autolayout"] = True

# Define columns and load the CSV file, dropping any rows with NaN values
columns = ["Seconds","Nanoseconds", "Accel_X"]
df = pd.read_csv("imu_data.csv", usecols=columns)

# Remove rows with NaN values
df = df.dropna()

df['Time'] = df['Seconds'] + df['Nanoseconds'] * 1e-9

# Convert the relevant columns to NumPy arrays
time = df["Time"].to_numpy()
accel_x = df["Accel_X"].to_numpy()

print("Standard deviation: " + str(np.std(accel_x)))

# Print the contents to verify the data after NaN removal
print("Contents in csv file:", df)

dt = 1/400 # Sample rate
n = len(accel_x) # Number of samples
fhat = np.fft.fft(accel_x,n) # FFT of signal

psd = fhat * np.conj(fhat) / n 
freq = (1/(dt*n)) * np.arange(n)
L = np.arange(1,np.floor(n/2),dtype='int')

indices = freq < 10 # Define cutoff frequency
freq_clean = freq * indices # Zero out all frequencies above cutoff
fhat = indices * fhat # Zero out small Fourier coeffs.
accel_x_filtered = np.fft.ifft(fhat) # Inverse FFT for filtered time signal

# Plot psd to find frequency with large power
plt.plot(freq[L], psd[L], label='noisy')
plt.xlim(freq[L[0]],freq[L[-1]])

# Add labels and legend
plt.xlabel('Frequency (Hz)')
plt.ylabel('Power')
plt.legend()

# Set the axis limits
#plt.ylim(-90, 90)
#plt.xlim(0,1)
#plt.xlim(604 + 1.7177620000e9,609 + 1.7177620000e9)

# Show the plot
plt.show()
