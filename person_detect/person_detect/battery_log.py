import subprocess
from datetime import datetime
import matplotlib.pyplot as plt
import time
import os
import csv
 
# ==== Battery Configuration ====
CAPACITY_AH = 5  # 5000 mAh
VOLTAGE_MAX = 16.6  # 4.15V * 4
VOLTAGE_MIN = 12.8  # 3.2V * 4
USE_CONSERVATIVE = False  # Use 80% of total battery to protect lifespan
 
# ==== Compute usable energy (Joules) ====
average_voltage = (VOLTAGE_MAX + VOLTAGE_MIN) / 2
usable_energy_joules = average_voltage * CAPACITY_AH * 3600  # Wh to J
 
if USE_CONSERVATIVE:
    usable_energy_joules *= 0.8
 
# ==== Tracking Lists ====
battery_levels = []
power_log = []
time_axis = []
total_energy_used = 0.0
start_time = None
 
csv_filename = "battery_log.csv"
csv_exists = os.path.exists(csv_filename)
 
csv_file = open(csv_filename, mode="w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["Timestamp", "ElapsedTime_s", "Power_W", "Energy_J", "BatteryLevel_%"])
 
# ==== Start tegrastats Process ====
proc = subprocess.Popen(["stdbuf", "-oL", "tegrastats", "--interval", "100"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
 
try:
    while True:
        line = proc.stdout.readline()
 
        if "VDD_IN" not in line:
            continue
 
        try:
            parts = line.strip().split()
            date, time_now = parts[0], parts[1]
            timestamp_str = f"{date} {time_now}"
            dt = datetime.strptime(timestamp_str, "%m-%d-%Y %H:%M:%S")
            if start_time is None:
                start_time = dt
 
            vdd_index = parts.index("VDD_IN")
            vdd = parts[vdd_index + 1]
            instantaneous, _ = vdd.split("/")
            power_mw = float(instantaneous.replace("mW", ""))
            power_w = power_mw / 1000.0
 
            # Energy in 100ms window (Δt = 0.1s): E = P * t
            energy_joules = power_w * 0.1  # J = W * s
 
            total_energy_used += energy_joules
            battery_pct = 100 * (1 - total_energy_used / usable_energy_joules)
            battery_pct = max(battery_pct, 0)  # Clamp at 0
 
            elapsed_sec = (dt - start_time).total_seconds()
 
            # Store for plotting
            power_log.append(power_w)
            battery_levels.append(battery_pct)
            time_axis.append(elapsed_sec)
 
            csv_writer.writerow([timestamp_str, f"{elapsed_sec:.2f}", f"{power_w:.3f}", f"{energy_joules:.3f}", f"{battery_pct:.2f}"])
 
            print(f"[Data] Time: {elapsed_sec:.1f}s | Energy_used: {energy_joules:.2f} J | Battery: {battery_pct:.2f}%")
 
        except Exception as e:
            print(f"[Error parsing line] {e}")
        time.sleep(0.1)  # Match 100ms interval
 
except KeyboardInterrupt:
    print("\n[INFO] Data collection stopped.")
    proc.terminate()
    csv_file.close()
 
    # ==== Plotting ====
    plt.figure(figsize=(10, 5))
    plt.plot(time_axis, battery_levels, label="Battery Level (%)", color='blue')
    plt.xlabel("Time (s)")
    plt.ylabel("Battery Level (%)")
    plt.title("Battery Drain Over Time")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()