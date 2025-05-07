import rclpy
from rclpy.node import Node
import subprocess
import time
import cv2
import csv
import threading

class TegrastatsLogger(Node):
    def __init__(self):
        super().__init__('tegrastats_node')
        # Start tegrastats
        interval_ms = 100
        self.proc = subprocess.Popen(["tegrastats", "--interval", str(interval_ms)], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)

        self.prev_instantaneous_mW = 0
        self.prev_average_mW = 0
        self.prev_unix_time = 0
        self.energy_J = 0
        self.energy_total_J = 0

        self.i = True

        self.start_time_str = time.strftime("%d-%m-%Y_%H-%M-%S")
        self.output_file = f"tegrastats_{self.start_time_str}.csv"

        self.csvfile = open(self.output_file, "w", newline='')
        self.writer = csv.writer(self.csvfile)
        self.writer.writerow(["unix_timestamp_sec", "vdd_mW", "vdd_avg_mW", "energy_J", "energy_total_J"])
        self.csvfile.flush()

    def log_tegrastats(self):

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

                self.writer.writerow([self.prev_unix_time, self.prev_instantaneous_mW, self.prev_average_mW, self.energy_J, self.energy_total_J])

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

            self.writer.writerow([unix_time, instantaneous_mW, average_mW, self.energy_J, self.energy_total_J])

            print(f"unix_time: {unix_time}")
            print(f"Instantaneous VDD_IN: {instantaneous_mW} mW")
            print("prev_instantaneous_mW:", self.prev_instantaneous_mW)
            # print(f"Average VDD_IN: {average_mW} mW")
            print("average_power_mW: ", avg_power_mW)
            print("elapsed_sec: ", elapsed_sec)
            print("energy_J: ", self.energy_J)

            self.prev_instantaneous_mW = instantaneous_mW
            self.prev_unix_time = unix_time

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