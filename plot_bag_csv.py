import pandas as pd
import numpy as np
from matplotlib import pyplot as plt

# Set the figure size and layout
plt.rcParams["figure.figsize"] = [15.00, 7.00]
plt.rcParams["figure.autolayout"] = True

# Define columns and load the CSV file, dropping any rows with NaN values
columns = ["Seconds","Nanoseconds", "Accel_Y"]
df = pd.read_csv("imu_data.csv", usecols=columns)

# Remove rows with NaN values
df = df.dropna()

df['Time'] = df['Seconds'] + df['Nanoseconds'] * 1e-9

# Convert the relevant columns to NumPy arrays
time = df["Time"].to_numpy()
accel_x = df["Accel_Y"].to_numpy()

print("Standard deviation: " + str(np.std(accel_x)))

# Print the contents to verify the data after NaN removal
print("Contents in csv file:", df)

# Plot each acceleration axis over time
plt.plot(time, accel_x, label='Accel_Y')

# Add labels and legend
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s²)')
plt.legend()

# Set the axis limits
#plt.ylim(-90, 90)
#plt.xlim(604 + 1.7177620000e9,609 + 1.7177620000e9)

# Show the plot
plt.show()
