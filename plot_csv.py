import pandas as pd
import numpy as np
from matplotlib import pyplot as plt

# Set the figure size and layout
plt.rcParams["figure.figsize"] = [15.00, 7.00]
plt.rcParams["figure.autolayout"] = True

# Define columns and load the CSV file, dropping any rows with NaN values
columns = ["Seconds","Nanoseconds", "Accel_X", "Accel_Y", "Accel_Z"]
df = pd.read_csv("filtered_imu_data_test.csv", usecols=columns)

# Remove rows with NaN values
df = df.dropna()

df['Time'] = df['Seconds'] + df['Nanoseconds'] * 1e-9

# Convert the relevant columns to NumPy arrays
time = df["Time"].to_numpy()
accel_x = df["Accel_X"].to_numpy()
accel_y = df["Accel_Y"].to_numpy()
accel_z = df["Accel_Z"].to_numpy()

print("Standard deviation: " + str(np.std(accel_x)))
print("Standard deviation: " + str(np.std(accel_y)))
print("Standard deviation: " + str(np.std(accel_z)))

# Print the contents to verify the data after NaN removal
print("Contents in csv file:", df)

# Plot each acceleration axis over time
plt.plot(time, accel_x, label='Accel_X')
plt.plot(time, accel_y, label='Accel_Y')
plt.plot(time, accel_z, label='Accel_Z')

# Add labels and legend
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s²)')
plt.legend()

# Set the axis limits
#plt.ylim(-90, 90)
#plt.xlim(604.8 + 1.7177620000e9,604.9 + 1.7177620000e9)

# Show the plot
plt.show()
