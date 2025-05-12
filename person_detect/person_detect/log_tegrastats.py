import rclpy
from rclpy.node import Node
import subprocess
import time
import cv2
import csv
import threading
import psutil


class TegrastatsLogger(Node):
    def __init__(self):
        super().__init__('tegrastats_node')
        # Start tegrastats
        interval_ms = 100
        # interval_ms = 1000
        self.get_logger().info(f"Starting tegrastats with interval {interval_ms} ms")
        self.proc = subprocess.Popen(["tegrastats", "--interval", str(interval_ms)], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)

        self.prev_instantaneous_mW = 0
        self.prev_average_mW = 0
        self.prev_unix_time = 0
        self.energy_J = 0
        self.energy_total_J = 0

        self.i = True

        self.interface = 'wlP1p1s0'
        self.old_sent = 0
        self.old_recv = 0

        self.start_time_str = time.strftime("%d-%m-%Y_%H-%M-%S")
        self.output_file = f"tegrastats_{self.start_time_str}.csv"

        # ==== Battery Configuration ====
        self.capacity_Ah = 5  # 5000 mAh
        self.voltage_max = 16.6 # 4.15V * 4
        self.voltage_min = 12.8  # 3.2V * 4
        self.use_conservative = False  # Use 80% of total battery to protect lifespan

        self.battery_pct = 100
        
        # ==== Compute usable energy (Joules) ====
        self.average_voltage = (self.voltage_max + self.voltage_min) / 2
        self.usable_energy_joules = self.average_voltage * self.capacity_Ah * 3600  # Wh to J
        
        if self.use_conservative:
            self.usable_energy_joules *= 0.8

        self.csvfile = open(self.output_file, "w", newline='')
        self.writer = csv.writer(self.csvfile)
        self.writer.writerow(["current_time_sec", "vdd_mW", "vdd_avg_mW", "energy_J", "energy_total_J", "sent_rate_Bps", "recv_rate_Bps", "battery_pct"])
        self.csvfile.flush()

    def log_tegrastats(self):

        # Ensure interface exists
        if self.interface not in psutil.net_io_counters(pernic=True):
            raise ValueError(f"Interface '{self.interface}' not found. Available: {list(psutil.net_io_counters(pernic=True).keys())}")

        while True:
            line = self.proc.stdout.readline()

            if self.i:
                prev_line = line
                prev_parts = prev_line.strip().split()

                self.prev_unix_time = time.time()

                print(f"unix_time: {self.prev_unix_time}")

                prev_vdd_in_index = prev_parts.index("VDD_IN")
                prev_vdd = prev_parts[prev_vdd_in_index+1]
                
                prev_instantaneous_str, prev_average_str = prev_vdd.split("/")        
                self.prev_instantaneous_mW = float(prev_instantaneous_str.replace("mW", ""))
                self.prev_average_mW = float(prev_average_str.replace("mW", ""))

                old = psutil.net_io_counters(pernic=True)[self.interface]
                self.old_sent = old.bytes_sent
                self.old_recv = old.bytes_recv

                self.writer.writerow([self.prev_unix_time, self.prev_instantaneous_mW, self.prev_average_mW, self.energy_J, self.energy_total_J, 0, 0, self.battery_pct])

                self.i = False
            
            # Split the line into parts
            parts = line.strip().split()

            # print(parts)

            unix_time = time.time()

            vdd_in_index = parts.index("VDD_IN")
            vdd = parts[vdd_in_index+1]

            instantaneous_str, average_str = vdd.split("/")
            
            instantaneous_mW = float(instantaneous_str.replace("mW", ""))
            average_mW = float(average_str.replace("mW", ""))

            avg_power_mW = (self.prev_instantaneous_mW + instantaneous_mW) / 2
            elapsed_sec = unix_time - self.prev_unix_time

            self.energy_J = (avg_power_mW * elapsed_sec) / 1000
            self.energy_total_J += self.energy_J

            new = psutil.net_io_counters(pernic=True)[self.interface]
            new_sent = new.bytes_sent
            new_recv = new.bytes_recv
            sent_rate = (new_sent - self.old_sent) / elapsed_sec
            recv_rate = (new_recv - self.old_recv) / elapsed_sec

            self.battery_pct = 100 * (1 - self.energy_total_J / self.usable_energy_joules)

            self.battery_pct = max(self.battery_pct, 0)  # Clamp at 0

            self.writer.writerow([unix_time, instantaneous_mW, average_mW, self.energy_J, self.energy_total_J, sent_rate, recv_rate, self.battery_pct])

            print(f"current_time_sec: {unix_time}")
            print(f"Instantaneous VDD_IN: {instantaneous_mW} mW")
            print("prev_instantaneous_mW:", self.prev_instantaneous_mW)
            print("average_power_mW: ", avg_power_mW)
            print("elapsed_sec: ", elapsed_sec)
            print("energy_J: ", self.energy_J)
            print("new_sent: ", new_sent)
            print("old_sent: ", self.old_sent)
            print("sent_rate: ", sent_rate)
            print("new_recv: ", new_recv)
            print("old_recv: ", self.old_recv)
            print("recv_rate: ", recv_rate)
            print("energy_total_J: ", self.energy_total_J)
            print("usable_energy_joules: ", self.usable_energy_joules)
            print("battery_pct: ", self.battery_pct)
            
            self.prev_instantaneous_mW = instantaneous_mW
            self.prev_unix_time = unix_time
            self.old_sent = new_sent
            self.old_recv = new_recv 

    def close(self):
        if self.proc:
            self.proc.terminate()
        if self.csvfile:
            self.csvfile.close()
        print("[TegrastatsLogger] Logging stopped and resources cleaned up.")

def main(args=None):
    rclpy.init(args=args)
    node = TegrastatsLogger()

    logging_thread = threading.Thread(target=node.log_tegrastats)
    logging_thread.daemon = True  # This makes the thread exit when the main program ends
    logging_thread.start()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()