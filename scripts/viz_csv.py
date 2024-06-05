import matplotlib.pyplot as plt
import pandas as pd

# Load the CSV file
df = pd.read_csv('velcro_force_torque_data.csv')

# Convert the timestamp column to datetime format
df['Time'] = pd.to_datetime(df['Time'], unit='s')

# Set the timestamp as the index
df.set_index('Time', inplace=True)

# Plot the force and torque values
fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

axs[0].plot(df[['Fx', 'Fy', 'Fz']])
axs[0].set_title('Force (N)')
axs[0].set_ylabel('Force (N)')
axs[0].legend(['Fx', 'Fy', 'Fz'])

axs[1].plot(df[['Tx', 'Ty', 'Tz']])
axs[1].set_title('Torque (Nm)')
axs[1].set_ylabel('Torque (Nm)')
axs[1].legend(['Tx', 'Ty', 'Tz'])

plt.xlabel('Time')
plt.tight_layout()
plt.show()