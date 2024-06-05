import rtde_receive
import matplotlib.pyplot as plt
import time
import csv

robot_ip = "192.168.1.102"

[print("in ur robot") for _ in range(4)]
try:
    r_inter = rtde_receive.RTDEReceiveInterface(robot_ip)

except Exception as e:
    print(e)
    print(robot_ip)

# Set up CSV file
with open('force_torque_data.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["Time", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz"])  # header row

while True:
    data = r_inter.getActualTCPForce()
    force = data[:3]  # extract force values
    torque = data[3:]  # extract torque values
    timestamp = time.time()

    # Write data to CSV file
    with open('force_torque_data.csv', 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([timestamp, *force, *torque])

    # Wait for 1/20th of a second (20 Hz)
    time.sleep(0.05)